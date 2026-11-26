"""Frame final do drone — peça única (corpo + braços + suportes de motor + nervuras + pés).

Script do Autodesk Fusion. Cria um componente novo "Frame final" no design aberto
(não altera nada do que já existe).

Convenção: o TOPO do frame fica em z = 0 e é totalmente plano (motores, ESCs, eletrônica
em cima). Nervuras e pés ficam para BAIXO. Imprimir DE CABEÇA PARA BAIXO (topo na mesa),
sem suportes: a face de cima sai lisa. Frente do drone = +Y (seta gravada no topo).

Uso: Fusion > Shift+S > Scripts > "+" > pasta ferramentas/frame_final > frame_final > Run.
Ajuste os PARÂMETROS abaixo e rode de novo (cada execução cria um componente novo).
"""
import math
import traceback

import adsk.core
import adsk.fusion

# ======================= PARÂMETROS (mm) =======================
MOTOR_XY = 80.0            # motores em (+-80, +-80) -> 226 mm na diagonal (igual ao frame atual)
PLACA = 3.0                # espessura do corpo e dos braços
LARG_BRACO = 16.0          # largura do braço
CORPO_RAIO = 34.0          # raio do disco central
PAD_RAIO = 13.0            # raio do suporte do motor
PAD_ESP = 4.0              # espessura do suporte do motor
FURO_MOTOR_D = 2.8         # 3 furos do motor (igual ao atual)
FURO_MOTOR_BC = 11.55      # diâmetro do círculo dos furos (igual ao atual)
FURO_MOTOR_ANG0 = 90.0     # ângulo do 1o furo (graus); os outros a +120 e +240
FURO_EIXO_D = 6.0          # furo central: eixo/clipe do motor (também deixa o pé oco)
NERV_LARG = 5.0            # nervura embaixo do braço (largura)
NERV_ALT = 7.0             # nervura: altura abaixo da placa
NERV_INICIO = 26.0         # distância ao centro onde a nervura começa
PE_D = 11.0                # pé: diâmetro externo (tubo; o furo do eixo o deixa oco)
PE_ALT = 24.0              # pé: altura total abaixo do topo
STANDOFF_R = 20.0          # 4 furos M3 na diagonal, a 20 mm do centro (espaçadores da eletrônica)
STANDOFF_D = 3.2
SLOT_W, SLOT_L = 2.4, 22.0  # 2 rasgos no centro para a fita da bateria
SLOT_Y = 22.0
FILETE_RAIZ = 8.0          # arredondamento na raiz dos braços (vista de cima)
FILETE_NERV = 2.0          # emenda nervura/placa
FILETE_PE = 3.0            # emenda pé/suporte do motor
FILETE_PAD = 4.0           # emenda braço/suporte do motor (vista de cima)
CHANFRO = 0.5              # chanfro nas arestas do topo
SETA_PROF = 0.6            # profundidade da seta "frente"
# ===============================================================

MM = 0.1   # a API trabalha em cm
TOL = 0.15


def P(x, y, z=0.0):
    return adsk.core.Point3D.create(x * MM, y * MM, z * MM)


def V(mm):
    return adsk.core.ValueInput.createByReal(mm * MM)


def diagonais():
    for sx, sy in ((1, 1), (-1, 1), (-1, -1), (1, -1)):
        yield sx, sy, sx / math.sqrt(2), sy / math.sqrt(2)


def retangulo(linhas, ax, ay, ux, uy, comp, larg):
    """Retângulo que começa em (ax,ay), segue a direção (ux,uy) por 'comp', com largura 'larg'."""
    vx, vy = -uy, ux
    h = larg / 2
    bx, by = ax + ux * comp, ay + uy * comp
    p = [(ax + vx * h, ay + vy * h), (bx + vx * h, by + vy * h),
         (bx - vx * h, by - vy * h), (ax - vx * h, ay - vy * h)]
    for i in range(4):
        linhas.addByTwoPoints(P(*p[i]), P(*p[(i + 1) % 4]))


def perfis_de(sketch):
    col = adsk.core.ObjectCollection.create()
    for i in range(sketch.profiles.count):
        col.add(sketch.profiles.item(i))
    return col


def extrudar(comp, sketch, dist_mm, op, corpo=None, para_baixo=True):
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(perfis_de(sketch), op)
    d = adsk.fusion.ExtentDirections.NegativeExtentDirection if para_baixo \
        else adsk.fusion.ExtentDirections.PositiveExtentDirection
    inp.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(V(dist_mm)), d)
    if corpo is not None:
        inp.participantBodies = [corpo]
    return ext.add(inp)


def cortar_passante(comp, sketch, corpo):
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(perfis_de(sketch), adsk.fusion.FeatureOperations.CutFeatureOperation)
    inp.setSymmetricExtent(V(80), True)
    inp.participantBodies = [corpo]
    return ext.add(inp)


def filete(comp, arestas, raio_mm):
    if arestas.count == 0:
        return
    fil = comp.features.filletFeatures
    inp = fil.createInput()
    try:
        inp.edgeSetInputs.addConstantRadiusEdgeSet(arestas, V(raio_mm), True)
    except AttributeError:
        inp.addConstantRadiusEdgeSet(arestas, V(raio_mm), True)
    fil.add(inp)


def z_da_aresta(e):
    return e.boundingBox.minPoint.z / MM, e.boundingBox.maxPoint.z / MM


def run(context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    avisos = []
    try:
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox("Abra (ou crie) um design no Fusion antes de rodar.")
            return
        design.designType = adsk.fusion.DesignTypes.ParametricDesignType
        occ = design.rootComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        comp = occ.component
        comp.name = "Frame final"
        xy = comp.xYConstructionPlane
        NOVO = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
        JUNTA = adsk.fusion.FeatureOperations.JoinFeatureOperation
        d_motor = MOTOR_XY * math.sqrt(2)

        # 1) Placa: disco central + 4 braços (topo em z = 0, espessura PLACA para baixo)
        sk = comp.sketches.add(xy); sk.name = "placa"
        sk.sketchCurves.sketchCircles.addByCenterRadius(P(0, 0), CORPO_RAIO * MM)
        r0 = CORPO_RAIO * 0.6
        for sx, sy, ux, uy in diagonais():
            retangulo(sk.sketchCurves.sketchLines, ux * r0, uy * r0, ux, uy, d_motor - r0, LARG_BRACO)
        corpo = extrudar(comp, sk, PLACA, NOVO).bodies.item(0)
        corpo.name = "frame"

        # 2) Suportes dos motores
        sk = comp.sketches.add(xy); sk.name = "suportes"
        for sx, sy, ux, uy in diagonais():
            sk.sketchCurves.sketchCircles.addByCenterRadius(P(sx * MOTOR_XY, sy * MOTOR_XY), PAD_RAIO * MM)
        extrudar(comp, sk, PAD_ESP, JUNTA, corpo)

        # 3) Nervuras embaixo dos braços, do corpo até o centro do motor (continuam no pé)
        sk = comp.sketches.add(xy); sk.name = "nervuras"
        for sx, sy, ux, uy in diagonais():
            retangulo(sk.sketchCurves.sketchLines, ux * NERV_INICIO, uy * NERV_INICIO, ux, uy,
                      d_motor - NERV_INICIO, NERV_LARG)
        extrudar(comp, sk, PLACA + NERV_ALT, JUNTA, corpo)

        # 4) Pés embaixo de cada motor
        sk = comp.sketches.add(xy); sk.name = "pes"
        for sx, sy, ux, uy in diagonais():
            sk.sketchCurves.sketchCircles.addByCenterRadius(P(sx * MOTOR_XY, sy * MOTOR_XY), PE_D / 2 * MM)
        extrudar(comp, sk, PE_ALT, JUNTA, corpo)

        # 5) Arredondamentos (antes dos furos, com a geometria ainda simples)
        # 5a) raiz dos braços: arestas verticais onde a borda do braço encontra o disco central
        sel = adsk.core.ObjectCollection.create()
        for e in corpo.edges:
            z0, z1 = z_da_aresta(e)
            c = e.boundingBox.minPoint
            if abs(z0 + PLACA) < TOL and abs(z1) < TOL and abs(math.hypot(c.x, c.y) / MM - CORPO_RAIO) < 2.0:
                sel.add(e)
        try:
            filete(comp, sel, FILETE_RAIZ)
        except Exception:
            avisos.append(f"filete da raiz dos braços ({sel.count} arestas) não aplicado")

        # 5b) emenda nervura/placa: linhas em z = -PLACA a meia-largura da nervura do eixo do braço
        sel = adsk.core.ObjectCollection.create()
        for e in corpo.edges:
            z0, z1 = z_da_aresta(e)
            if not (abs(z0 + PLACA) < TOL and abs(z1 + PLACA) < TOL):
                continue
            if e.geometry.objectType != adsk.core.Line3D.classType():
                continue
            b = e.boundingBox
            mx = (b.minPoint.x + b.maxPoint.x) / 2 / MM
            my = (b.minPoint.y + b.maxPoint.y) / 2 / MM
            for sx, sy, ux, uy in diagonais():
                ao_longo = mx * ux + my * uy
                lateral = abs(-mx * uy + my * ux)
                if abs(lateral - NERV_LARG / 2) < 0.3 and NERV_INICIO < ao_longo < d_motor - PAD_RAIO:
                    sel.add(e)
                    break
        try:
            filete(comp, sel, FILETE_NERV)
        except Exception:
            avisos.append(f"filete nervura/placa ({sel.count} arestas) não aplicado")

        # 5c) emenda pé/suporte: círculos de raio PE_D/2 em z = -PAD_ESP
        sel = adsk.core.ObjectCollection.create()
        for e in corpo.edges:
            z0, z1 = z_da_aresta(e)
            g = e.geometry
            if abs(z0 + PAD_ESP) < TOL and abs(z1 + PAD_ESP) < TOL and \
                    g.objectType in (adsk.core.Circle3D.classType(), adsk.core.Arc3D.classType()) and \
                    abs(g.radius / MM - PE_D / 2) < 0.2:
                sel.add(e)
        try:
            filete(comp, sel, FILETE_PE)
        except Exception:
            avisos.append(f"filete pé/suporte ({sel.count} arestas) não aplicado")

        # 5d) braço/suporte do motor: arestas verticais onde a borda do braço encontra o disco do motor
        sel = adsk.core.ObjectCollection.create()
        for e in corpo.edges:
            z0, z1 = z_da_aresta(e)
            c = e.boundingBox.minPoint
            if abs(z0 + PLACA) < TOL and abs(z1) < TOL:
                for sx, sy, ux, uy in diagonais():
                    if abs(math.hypot(c.x / MM - sx * MOTOR_XY, c.y / MM - sy * MOTOR_XY) - PAD_RAIO) < 0.5:
                        sel.add(e)
                        break
        try:
            filete(comp, sel, FILETE_PAD)
        except Exception:
            avisos.append(f"filete braço/suporte ({sel.count} arestas) não aplicado")

        # 6) Furos passantes: motores (3 + eixo, que também deixa o pé oco), M3 e rasgos da bateria
        sk = comp.sketches.add(xy); sk.name = "furos"
        cir = sk.sketchCurves.sketchCircles
        for sx, sy, ux, uy in diagonais():
            mx, my = sx * MOTOR_XY, sy * MOTOR_XY
            for k in range(3):
                a = math.radians(FURO_MOTOR_ANG0 + 120 * k)
                cir.addByCenterRadius(P(mx + FURO_MOTOR_BC / 2 * math.cos(a),
                                        my + FURO_MOTOR_BC / 2 * math.sin(a)), FURO_MOTOR_D / 2 * MM)
            cir.addByCenterRadius(P(mx, my), FURO_EIXO_D / 2 * MM)
            cir.addByCenterRadius(P(ux * STANDOFF_R, uy * STANDOFF_R), STANDOFF_D / 2 * MM)
        for yy in (SLOT_Y, -SLOT_Y):
            retangulo(sk.sketchCurves.sketchLines, -SLOT_L / 2, yy, 1, 0, SLOT_L, SLOT_W)
        cortar_passante(comp, sk, corpo)

        # 7) Seta "frente" (+Y) rebaixada no topo
        sk = comp.sketches.add(xy); sk.name = "seta frente"
        lin = sk.sketchCurves.sketchLines
        pts = [(0, 14), (-5, 6), (-1.6, 6), (-1.6, 0), (1.6, 0), (1.6, 6), (5, 6)]
        for i in range(len(pts)):
            lin.addByTwoPoints(P(*pts[i]), P(*pts[(i + 1) % len(pts)]))
        try:
            extrudar(comp, sk, SETA_PROF, adsk.fusion.FeatureOperations.CutFeatureOperation, corpo)
        except Exception:
            avisos.append("seta da frente não gravada")

        # 8) Chanfro leve em todas as arestas do topo (acabamento)
        sel = adsk.core.ObjectCollection.create()
        for e in corpo.edges:
            z0, z1 = z_da_aresta(e)
            if abs(z0) < TOL and abs(z1) < TOL:
                sel.add(e)
        try:
            ch = comp.features.chamferFeatures
            try:
                cin = ch.createInput2()
                cin.chamferEdgeSets.addEqualDistanceChamferEdgeSet(sel, V(CHANFRO), True)
            except AttributeError:
                cin = ch.createInput(sel, True)
                cin.setToEqualDistance(V(CHANFRO))
            ch.add(cin)
        except Exception:
            avisos.append("chanfro do topo não aplicado")

        vol = corpo.physicalProperties.volume  # cm3
        msg = (f"Frame final criado (peça única).\n\nVolume: {vol:.1f} cm³\n"
               f"Massa estimada em PETG: ~{vol * 1.27:.0f} g maciço, ~{vol * 1.27 * 0.65:.0f} g com "
               "4 paredes e 40% de preenchimento.\n\n"
               "Imprimir de CABEÇA PARA BAIXO (topo plano na mesa), sem suportes.")
        if avisos:
            msg += "\n\nAvisos:\n- " + "\n- ".join(avisos)
        ui.messageBox(msg, "Frame final")
    except Exception:
        if ui:
            ui.messageBox("Falhou:\n{}".format(traceback.format_exc()))
