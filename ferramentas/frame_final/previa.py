"""Pré-visualização do frame final (vista de cima + perfil do braço), sem precisar do Fusion.

    python ferramentas/frame_final/previa.py [saida.png]

Lê os PARÂMETROS direto de frame_final.py, então mostra exatamente o que o script vai criar
(exceto os arredondamentos e chanfros, que só o Fusion calcula).
"""
import math
import os
import re
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon, Rectangle

PASTA = os.path.dirname(os.path.abspath(__file__))
src = open(os.path.join(PASTA, "frame_final.py"), encoding="utf-8").read()
bloco = src.split("# ======================= PARÂMETROS (mm) =======================")[1].split("# ====")[0]
par = {}
exec(bloco, {}, par)
g = type("G", (), par)

d_motor = g.MOTOR_XY * math.sqrt(2)
diag = [(sx, sy, sx / math.sqrt(2), sy / math.sqrt(2)) for sx, sy in ((1, 1), (-1, 1), (-1, -1), (1, -1))]


def ret(ax_, ay, ux, uy, comp, larg):
    vx, vy = -uy, ux
    h = larg / 2
    bx, by = ax_ + ux * comp, ay + uy * comp
    return [(ax_ + vx * h, ay + vy * h), (bx + vx * h, by + vy * h), (bx - vx * h, by - vy * h), (ax_ - vx * h, ay - vy * h)]


fig, (a1, a2) = plt.subplots(1, 2, figsize=(15, 7), gridspec_kw={"width_ratios": [1.1, 1]})
cinza, escuro, nerv = "#8d99a6", "#3d4752", "#d9480f"
a1.add_patch(Circle((0, 0), g.CORPO_RAIO, color=cinza))
r0 = g.CORPO_RAIO * 0.6
for sx, sy, ux, uy in diag:
    a1.add_patch(Polygon(ret(ux * r0, uy * r0, ux, uy, d_motor - r0, g.LARG_BRACO), color=cinza))
    a1.add_patch(Polygon(ret(ux * g.NERV_INICIO, uy * g.NERV_INICIO, ux, uy, d_motor - g.NERV_INICIO, g.NERV_LARG),
                         fill=False, ec=nerv, ls="--", lw=1.2))
    mx, my = sx * g.MOTOR_XY, sy * g.MOTOR_XY
    a1.add_patch(Circle((mx, my), g.PAD_RAIO, color=cinza))
    a1.add_patch(Circle((mx, my), g.PE_D / 2, fill=False, ec=nerv, ls="--", lw=1.2))
    for k in range(3):
        a = math.radians(g.FURO_MOTOR_ANG0 + 120 * k)
        a1.add_patch(Circle((mx + g.FURO_MOTOR_BC / 2 * math.cos(a), my + g.FURO_MOTOR_BC / 2 * math.sin(a)),
                            g.FURO_MOTOR_D / 2, color="white"))
    a1.add_patch(Circle((mx, my), g.FURO_EIXO_D / 2, color="white"))
    a1.add_patch(Circle((ux * g.STANDOFF_R, uy * g.STANDOFF_R), g.STANDOFF_D / 2, color="white"))
for yy in (g.SLOT_Y, -g.SLOT_Y):
    a1.add_patch(Polygon(ret(-g.SLOT_L / 2, yy, 1, 0, g.SLOT_L, g.SLOT_W), color="white"))
a1.add_patch(Polygon([(0, 14), (-5, 6), (-1.6, 6), (-1.6, 0), (1.6, 0), (1.6, 6), (5, 6)], color=escuro))
for sx, sy, nome in ((-1, 1, "M1"), (1, 1, "M2"), (1, -1, "M3"), (-1, -1, "M4")):
    a1.text(sx * (g.MOTOR_XY + 19), sy * (g.MOTOR_XY + 19), nome, ha="center", va="center", fontsize=11)
a1.set_xlim(-110, 110); a1.set_ylim(-110, 110); a1.set_aspect("equal")
a1.set_title("Vista de cima (topo plano) — tracejado laranja: nervura e pé por baixo")
a1.set_xlabel("x (mm)"); a1.set_ylabel("y (mm)  — frente = +Y (seta)")

# Perfil ao longo de um braço (corte pela diagonal)
a2.add_patch(Rectangle((0, -g.PLACA), d_motor + g.PAD_RAIO, g.PLACA, color=cinza))
a2.add_patch(Rectangle((d_motor - g.PAD_RAIO, -g.PAD_ESP), 2 * g.PAD_RAIO, g.PAD_ESP, color=cinza))
a2.add_patch(Rectangle((g.NERV_INICIO, -(g.PLACA + g.NERV_ALT)), d_motor - g.NERV_INICIO, g.NERV_ALT, color=nerv))
a2.add_patch(Rectangle((d_motor - g.PE_D / 2, -g.PE_ALT), g.PE_D, g.PE_ALT - g.PAD_ESP, color=nerv))
a2.add_patch(Rectangle((d_motor - g.FURO_EIXO_D / 2, -g.PE_ALT), g.FURO_EIXO_D, g.PE_ALT, color="white"))
a2.add_patch(Rectangle((d_motor - 7, 0), 14, 11, fill=False, ec=escuro, ls=":"))
a2.text(d_motor, 13, "motor", ha="center", fontsize=9)
a2.axhline(-g.PE_ALT, color="#999", lw=0.8)
a2.text(2, -g.PE_ALT - 3, "chão", fontsize=9, color="#666")
a2.set_xlim(-5, d_motor + 25); a2.set_ylim(-g.PE_ALT - 8, 18); a2.set_aspect("equal")
a2.set_title("Perfil do braço (corte na diagonal): placa + nervura + pé oco")
a2.set_xlabel("distância ao centro (mm)"); a2.set_ylabel("z (mm)")

# Estimativa de volume (sem filetes): soma das partes menos sobreposições grosseiras
area_disco = math.pi * g.CORPO_RAIO ** 2
area_bracos = 4 * g.LARG_BRACO * (d_motor - g.CORPO_RAIO)
v = (area_disco + area_bracos) * g.PLACA
v += 4 * math.pi * g.PAD_RAIO ** 2 * (g.PAD_ESP - g.PLACA) + 4 * (math.pi * g.PAD_RAIO ** 2 - g.LARG_BRACO * g.PAD_RAIO) * g.PLACA * 0.5
v += 4 * g.NERV_LARG * g.NERV_ALT * (d_motor - g.NERV_INICIO)
v += 4 * math.pi * ((g.PE_D / 2) ** 2 - (g.FURO_EIXO_D / 2) ** 2) * (g.PE_ALT - g.PAD_ESP)
v /= 1000
fig.suptitle(f"Frame final (peça única) — volume ~{v:.0f} cm³ → ~{v*1.27:.0f} g PETG maciço, "
             f"~{v*1.27*0.65:.0f} g impresso (4 paredes, 40%)", fontsize=12)
plt.tight_layout()
saida = sys.argv[1] if len(sys.argv) > 1 else os.path.join(PASTA, "previa.png")
plt.savefig(saida, dpi=105)
print(f"{saida} | volume ~{v:.1f} cm3")
