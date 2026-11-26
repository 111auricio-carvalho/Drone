"""
Leitura de referência do drone — SÓ LÊ, não altera nenhuma configuração.

Comandos enviados ao drone: apenas 'ganhos', 'estado' e 'trim' (sem valor), que só mostram.
Tudo é salvo em logs/leitura_<data>_<hora>/ (resumo.txt + CSVs).

Etapas (rode na ordem, cada uma é opcional):
    python ferramentas/leitura.py COM4 repouso        # drone parado na mesa, 15 s: ruído, nível, loop
    python ferramentas/leitura.py COM4 vib            # teste de vibração 740 (GIRA MOTORES, segure o drone)
    python ferramentas/leitura.py COM4 voo            # grava voo até Ctrl+C e analisa deriva/equilíbrio

COM4 = controle (via rádio); também funciona com o drone na USB.
'vib' é o único passo que gira motores; ele pede confirmação digitada.
"""
import argparse
import datetime
import math
import os
import sys
import time

COLUNAS = ["ms", "flags", "thr", "roll", "pitch", "gR", "gP", "gY", "spR", "spP",
           "m1", "m2", "m3", "m4", "escala", "oscR", "oscP", "loopHz"]
RAIZ = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")


def abrir(porta):
    import serial
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = porta, 115200, 0.1
    s.dtr = False   # não resetar o ESP32 ao abrir
    s.rts = False
    s.open()
    time.sleep(0.3)
    s.reset_input_buffer()
    return s


def ler_por(s, segundos, enviar=None):
    """Lê linhas por N segundos. Retorna (mensagens '#', amostras T)."""
    if enviar:
        s.write((enviar + "\n").encode())
    msgs, amostras, buf, fim = [], [], "", time.time() + segundos
    while time.time() < fim:
        buf += s.read(4096).decode(errors="replace")
        *linhas, buf = buf.split("\n")
        for l in linhas:
            l = l.strip()
            if l.startswith("#"):
                msgs.append(l)
            elif l.startswith("T,") and l.count(",") == len(COLUNAS):
                try:
                    amostras.append(dict(zip(COLUNAS, map(float, l.split(",")[1:]))))
                except ValueError:
                    pass
    return msgs, amostras


def rms(v):
    return math.sqrt(sum(x * x for x in v) / len(v)) if v else 0.0


def media(v):
    return sum(v) / len(v) if v else 0.0


def inclinacao(ts, ys):
    """Coeficiente angular (unidade de y por segundo) por mínimos quadrados."""
    n = len(ts)
    if n < 3:
        return 0.0
    mt, my = media(ts), media(ys)
    den = sum((t - mt) ** 2 for t in ts)
    return sum((t - mt) * (y - my) for t, y in zip(ts, ys)) / den if den else 0.0


def salvar_csv(pasta, nome, amostras):
    with open(os.path.join(pasta, nome), "w") as f:
        f.write(",".join(COLUNAS) + "\n")
        for a in amostras:
            f.write(",".join(str(a[c]) for c in COLUNAS) + "\n")


def config_atual(s, out):
    msgs, _ = ler_por(s, 1.2, "ganhos")
    m2, _ = ler_por(s, 1.0, "estado")
    m3, _ = ler_por(s, 1.0, "trim")
    out("== Configuração atual (somente leitura) ==")
    for l in msgs + m2 + m3:
        out("  " + l)


def etapa_repouso(s, out, pasta):
    out("\n== Repouso: drone PARADO na mesa, 15 s ==")
    msgs, a = ler_por(s, 15)
    salvar_csv(pasta, "repouso.csv", a)
    if not a:
        out("  Sem telemetria. O drone está ligado e o rádio conectado?")
        return
    out(f"  amostras {len(a)} | loop {media([x['loopHz'] for x in a]):.0f} Hz")
    out(f"  ângulo médio (nível que o sensor vê): R {media([x['roll'] for x in a]):+.2f}  P {media([x['pitch'] for x in a]):+.2f} graus")
    out(f"  ruído do giro (RMS): R {rms([x['gR'] for x in a]):.2f}  P {rms([x['gP'] for x in a]):.2f}  Y {rms([x['gY'] for x in a]):.2f} graus/s")
    t = [(x["ms"] - a[0]["ms"]) / 1000 for x in a]
    out(f"  deriva do ângulo parado: R {inclinacao(t, [x['roll'] for x in a]):+.3f}  P {inclinacao(t, [x['pitch'] for x in a]):+.3f} graus/s (ideal ~0)")
    for l in msgs:
        out("  " + l)


def etapa_vib(s, out):
    out("\n== Vibração por motor (vib 740) ==")
    print("ATENÇÃO: os motores vão girar (cada um 2,5 s, depois os 4 juntos).")
    print("Segure o drone FIRME pelo corpo, braços livres, mãos longe das hélices.")
    if input("Digite SIM para girar os motores: ").strip().upper() != "SIM":
        out("  pulado pelo usuário")
        return
    s.write(b"vib 740\n")
    msgs, buf, fim = [], "", time.time() + 25
    while time.time() < fim:
        buf += s.read(4096).decode(errors="replace")
        *linhas, buf = buf.split("\n")
        msgs += [l.strip() for l in linhas if l.startswith("# VIB") or l.startswith("#    angulo")]
        if any("VIB concluido" in m or "VIB abortado" in m for m in msgs):
            break
    if not any("VIB concluido" in m or "VIB abortado" in m for m in msgs):
        s.write(b"parar\n")   # só se o teste não terminou sozinho: garante motores parados
    for l in msgs:
        out("  " + l)


def etapa_voo(s, out, pasta):
    out("\n== Voo: gravando até Ctrl+C ==")
    print("Voe normalmente (de preferência no modo LEVE, pairando). Ctrl+C ao terminar, já desarmado.")
    msgs, a, buf = [], [], ""
    try:
        while True:
            m, x = ler_por(s, 0.5)
            msgs += m
            a += x
            if x:
                u = x[-1]
                print(f"\r  [{'ARM' if int(u['flags']) & 1 else '---'}] thr {u['thr']:.0f} R {u['roll']:+5.1f} P {u['pitch']:+5.1f}   ", end="", flush=True)
    except KeyboardInterrupt:
        print()
    salvar_csv(pasta, "voo.csv", a)
    for l in msgs:
        if any(k in l for k in ("ARMADO", "Desarmado", "Trim", "Notch", "Bias")):
            out("  " + l)
    analisar_voo(a, out)


def analisar_voo(a, out):
    voo = [x for x in a if int(x["flags"]) & 1 and x["thr"] > 650]
    if len(voo) < 50:
        out("  Poucos dados em voo para analisar.")
        return
    t = [(x["ms"] - voo[0]["ms"]) / 1000 for x in voo]
    g = lambda k: [x[k] for x in voo]
    dir_esq = [(x["m2"] + x["m3"]) - (x["m1"] + x["m4"]) for x in voo]
    tras_frente = [(x["m3"] + x["m4"]) - (x["m1"] + x["m2"]) for x in voo]
    out(f"  tempo em voo ~{len(voo) * 0.02:.0f} s")
    out(f"  ângulo médio R {media(g('roll')):+.2f} P {media(g('pitch')):+.2f} | erro RMS R {rms([x['spR'] - x['roll'] for x in voo]):.2f} P {rms([x['spP'] - x['pitch'] for x in voo]):.2f} graus")
    out(f"  vibração (RMS taxa) R {rms(g('gR')):.0f} P {rms(g('gP')):.0f} Y {rms(g('gY')):.0f} graus/s")
    out(f"  motores médios M1 {media(g('m1')):.0f} M2 {media(g('m2')):.0f} M3 {media(g('m3')):.0f} M4 {media(g('m4')):.0f}")
    out(f"  desequilíbrio médio: direita-esquerda {media(dir_esq):+.0f} | trás-frente {media(tras_frente):+.0f} (contagens)")
    # Indicadores do problema de "ângulo andando" visto com o sensor na espuma:
    pairando = [i for i, x in enumerate(voo) if abs(x["thr"] - 736) < 15]
    if len(pairando) > 100:
        tp = [t[i] for i in pairando]
        out(f"  pairando ({len(pairando) * 0.02:.0f} s): ângulo estimado anda R {inclinacao(tp, [voo[i]['roll'] for i in pairando]):+.3f} P {inclinacao(tp, [voo[i]['pitch'] for i in pairando]):+.3f} graus/s"
            f" | motores divergem dir-esq {inclinacao(tp, [dir_esq[i] for i in pairando]):+.2f} trás-frente {inclinacao(tp, [tras_frente[i] for i in pairando]):+.2f} contagens/s")
        out("  REFERÊNCIA com espuma (voo 224108): ângulo médio R +9.47 P -6.68 | dir-esq +229 trás-frente +205"
            " | ângulo anda R +0.19 P -0.06 graus/s | motores divergem +1.22 / +0.76 contagens/s")
        out("  Sem espuma, o esperado é tudo perto de 0 (estimador saudável, motores equilibrados).")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("porta")
    ap.add_argument("etapas", nargs="*", default=["repouso"], choices=["repouso", "vib", "voo"])
    args = ap.parse_args()

    pasta = os.path.join(RAIZ, "logs", datetime.datetime.now().strftime("leitura_%Y%m%d_%H%M%S"))
    os.makedirs(pasta, exist_ok=True)
    resumo = open(os.path.join(pasta, "resumo.txt"), "w", encoding="utf-8")

    def out(l):
        print(l)
        resumo.write(l + "\n")
        resumo.flush()

    s = abrir(args.porta)
    try:
        out(f"Leitura {datetime.datetime.now():%Y-%m-%d %H:%M} pela porta {args.porta}")
        config_atual(s, out)
        for e in args.etapas:
            {"repouso": lambda: etapa_repouso(s, out, pasta),
             "vib": lambda: etapa_vib(s, out),
             "voo": lambda: etapa_voo(s, out, pasta)}[e]()
    finally:
        s.close()
        resumo.close()
        print(f"\nSalvo em {os.path.abspath(pasta)}")


if __name__ == "__main__":
    main()
