"""Script do Autodesk Fusion: exporta massa, centro de massa, inércia e dimensões
do modelo aberto para fusion_props.json (na pasta deste script).

Uso: Fusion > Utilities > Scripts and Add-Ins (Shift+S) > "+" > selecione esta pasta
     > fusion_props > Run.
"""
import json
import os
import traceback

import adsk.core
import adsk.fusion


def run(context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox("Abra o design do drone antes de rodar o script.")
            return
        raiz = design.rootComponent
        props = raiz.getPhysicalProperties(adsk.fusion.CalculationAccuracy.HighCalculationAccuracy)
        ok, xx, yy, zz, xy, yz, xz = props.getXYZMomentsOfInertia()  # kg*cm^2
        caixa = raiz.boundingBox
        com = props.centerOfMass

        corpos = []
        for occ in raiz.allOccurrences:
            try:
                p = occ.getPhysicalProperties(adsk.fusion.CalculationAccuracy.LowCalculationAccuracy)
                c = p.centerOfMass
                corpos.append({"nome": occ.fullPathName, "massa_g": round(p.mass * 1000, 2),
                               "cg_mm": [round(c.x * 10, 1), round(c.y * 10, 1), round(c.z * 10, 1)]})
            except Exception:
                pass

        dados = {
            "design": app.activeDocument.name,
            "unidades": "massa g, comprimentos mm, inercia g*mm^2 (eixos do modelo)",
            "massa_g": round(props.mass * 1000, 2),
            "centro_massa_mm": [round(com.x * 10, 2), round(com.y * 10, 2), round(com.z * 10, 2)],
            "inercia_g_mm2": {k: round(v * 1000 * 100, 1) for k, v in
                              {"xx": xx, "yy": yy, "zz": zz, "xy": xy, "yz": yz, "xz": xz}.items()},
            "dimensoes_mm": [round((caixa.maxPoint.x - caixa.minPoint.x) * 10, 1),
                             round((caixa.maxPoint.y - caixa.minPoint.y) * 10, 1),
                             round((caixa.maxPoint.z - caixa.minPoint.z) * 10, 1)],
            "componentes": corpos,
        }
        destino = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fusion_props.json")
        with open(destino, "w", encoding="utf-8") as f:
            json.dump(dados, f, indent=2, ensure_ascii=False)
        ui.messageBox(f"Massa: {dados['massa_g']} g\nCG: {dados['centro_massa_mm']} mm\n\nSalvo em:\n{destino}")
    except Exception:
        if ui:
            ui.messageBox("Falhou:\n{}".format(traceback.format_exc()))
