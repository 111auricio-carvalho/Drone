"""Ajuste ao vivo: mantém a porta do controle aberta, mostra métricas a cada 2 s
e envia comandos escritos num arquivo (uma linha = um comando), sem fechar a porta.

    python ferramentas/ao_vivo.py COM4 <arquivo_de_comandos>

Saída (stdout), uma linha por janela de 2 s:
    M t=12.0 arm=1 thr=835 | erroR 0.7 erroP 1.6 | vibR 6 vibP 9 | oscR 1.2Hz oscP 0.8Hz | P -0.6 alvoP -0.5 | fr-tr +30
Mensagens do drone começam com '#'. O CSV vai para logs/aovivo_<data>.csv.
"""
import collections
import datetime
import math
import os
import sys
import time

import serial

COL = ["ms", "flags", "thr", "roll", "pitch", "gR", "gP", "gY", "spR", "spP",
       "m1", "m2", "m3", "m4", "escala", "oscR", "oscP", "loopHz"]


def rms(v):
    return math.sqrt(sum(x * x for x in v) / len(v)) if v else 0.0


def osc_hz(v, dur):
    if len(v) < 10 or dur <= 0:
        return 0.0
    m = sum(v) / len(v)
    s = [x - m for x in v]
    return sum(1 for i in range(1, len(s)) if (s[i - 1] < 0) != (s[i] < 0)) / 2 / dur


def main():
    porta, arq_cmd = sys.argv[1], sys.argv[2]
    raiz = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "logs")
    os.makedirs(raiz, exist_ok=True)
    csvp = os.path.join(raiz, datetime.datetime.now().strftime("aovivo_%Y%m%d_%H%M%S.csv"))
    csv = open(csvp, "w")
    csv.write(",".join(COL) + "\n")
    open(arq_cmd, "a").close()
    lidos = os.path.getsize(arq_cmd)   # ignora comandos antigos

    s = serial.Serial()
    s.port, s.baudrate, s.timeout = porta, 115200, 0.05
    s.dtr = False
    s.rts = False
    s.open()
    s.write(b"log 2\n")
    print(f"# ao vivo em {porta}; CSV {os.path.abspath(csvp)}; comandos de {arq_cmd}", flush=True)

    buf, janela, t_ini, t_jan = "", [], time.time(), time.time()
    while True:
        try:
            buf += s.read(4096).decode(errors="replace")
        except (serial.SerialException, OSError) as e:
            print(f"# [serial] {e} -- reconectando", flush=True)
            try:
                s.close()
            except Exception:
                pass
            while True:
                time.sleep(1)
                try:
                    s.open(); break
                except (serial.SerialException, OSError):
                    pass
            continue
        *linhas, buf = buf.split("\n")
        for l in linhas:
            l = l.strip()
            if l.startswith("#"):
                print(l, flush=True)
            elif l.startswith("T,") and l.count(",") == len(COL):
                try:
                    a = dict(zip(COL, map(float, l.split(",")[1:])))
                except ValueError:
                    continue
                janela.append(a)
                csv.write(",".join(str(a[c]) for c in COL) + "\n")

        # comandos novos no arquivo
        try:
            tam = os.path.getsize(arq_cmd)
            if tam > lidos:
                with open(arq_cmd, encoding="utf-8") as f:
                    f.seek(lidos)
                    novos = f.read()
                lidos = tam
                for c in novos.splitlines():
                    c = c.strip()
                    if c:
                        s.write((c + "\n").encode())
                        print(f"> {c}", flush=True)
        except OSError:
            pass

        if time.time() - t_jan >= 2.0 and janela:
            csv.flush()
            w = janela
            janela, t_jan = [], time.time()
            arm = int(w[-1]["flags"]) & 1
            voo = [x for x in w if x["thr"] > 780]
            if not voo:
                print(f"M t={time.time()-t_ini:.0f} arm={arm} thr={w[-1]['thr']:.0f} | (no chao)", flush=True)
                continue
            dur = (voo[-1]["ms"] - voo[0]["ms"]) / 1000 or 0.02
            g = lambda k: [x[k] for x in voo]
            print(f"M t={time.time()-t_ini:.0f} arm={arm} thr={sum(g('thr'))/len(voo):.0f} | "
                  f"erroR {rms([x['spR']-x['roll'] for x in voo]):.1f} erroP {rms([x['spP']-x['pitch'] for x in voo]):.1f} | "
                  f"vibR {rms(g('gR')):.0f} vibP {rms(g('gP')):.0f} | "
                  f"oscR {osc_hz(g('roll'), dur):.1f}Hz oscP {osc_hz(g('pitch'), dur):.1f}Hz | "
                  f"P {sum(g('pitch'))/len(voo):+.1f} alvoP {sum(g('spP'))/len(voo):+.1f} | "
                  f"fr-tr {sum((x['m1']+x['m2'])-(x['m3']+x['m4']) for x in voo)/len(voo):+.0f}", flush=True)


if __name__ == "__main__":
    main()
