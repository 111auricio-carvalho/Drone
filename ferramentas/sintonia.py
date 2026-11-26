"""
Ferramenta de sintonia do drone (feedback em tempo real).

Conecte pela USB o DRONE (bancada) ou o CONTROLE (em voo, telemetria via ESP-NOW).

    python ferramentas/sintonia.py COM3            # console + log CSV
    python ferramentas/sintonia.py COM3 --grafico  # + gráfico ao vivo
    python ferramentas/sintonia.py --analisar logs/voo_xxx.csv

No console, digite comandos para o drone: ajuda, ganhos, rp 0.25, rd 0.0015, salvar...
Ctrl+C encerra e mostra a análise do voo com sugestões de ajuste.
"""
import argparse
import collections
import csv
import datetime
import math
import os
import sys
import threading
import time

COLUNAS = ["ms", "flags", "thr", "roll", "pitch", "gR", "gP", "gY", "spR", "spP",
           "m1", "m2", "m3", "m4", "escala", "oscR", "oscP", "loopHz"]
PASTA_LOGS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "logs")


def interpretar(linha):
    partes = linha.strip().split(",")
    if len(partes) != len(COLUNAS) + 1 or partes[0] != "T":
        return None
    try:
        return {c: float(v) for c, v in zip(COLUNAS, partes[1:])}
    except ValueError:
        return None


def rms(valores):
    return math.sqrt(sum(v * v for v in valores) / len(valores)) if valores else 0.0


def analisar(amostras):
    voo = [a for a in amostras if int(a["flags"]) & 1 and a["thr"] > 575]
    print("\n=========== ANALISE ===========")
    print(f"amostras: {len(amostras)}  |  em voo (armado, throttle > idle): {len(voo)}")
    if amostras:
        print(f"loop: {amostras[-1]['loopHz']:.0f} Hz")
    if len(voo) < 20:
        print("Poucos dados em voo. Use 'log 2' (50 Hz) e voe/segure o drone com throttle.")
        return
    erroR = [a["spR"] - a["roll"] for a in voo]
    erroP = [a["spP"] - a["pitch"] for a in voo]
    print(f"erro de angulo RMS:   roll {rms(erroR):5.1f} graus | pitch {rms(erroP):5.1f} graus")
    print(f"taxa giro RMS:        roll {rms([a['gR'] for a in voo]):5.0f} | pitch {rms([a['gP'] for a in voo]):5.0f} | yaw {rms([a['gY'] for a in voo]):5.0f} graus/s")
    satur = sum(1 for a in voo if max(a["m1"], a["m2"], a["m3"], a["m4"]) >= 1023
                or min(a["m1"], a["m2"], a["m3"], a["m4"]) <= 535) / len(voo)
    osc = sum(1 for a in voo if int(a["flags"]) & 8) / len(voo)
    print(f"motores saturados:    {satur * 100:.0f}% do tempo")
    print(f"oscilacao detectada:  {osc * 100:.0f}% do tempo  |  escala final {voo[-1]['escala']:.2f}")
    hz = [max(a["oscR"], a["oscP"]) for a in voo if int(a["flags"]) & 8]
    if hz:
        print(f"frequencia tipica do tremor: {sorted(hz)[len(hz) // 2]:.0f} Hz")

    print("\nSugestoes:")
    sugestoes = []
    if osc > 0.1:
        f = sorted(hz)[len(hz) // 2] if hz else 0
        if f >= 15:
            sugestoes.append("Tremor rapido (>=15 Hz): reduza rd 30% e/ou dhz (ex.: dhz 20). Confira helices/vibracao.")
        else:
            sugestoes.append("Oscilacao media (6-15 Hz): reduza rp 20%. Se persistir, reduza ang.")
    if voo[-1]["escala"] < 0.99:
        sugestoes.append(f"O auto-ajuste baixou os ganhos para {voo[-1]['escala']:.2f}: rode 'aplicar' e 'salvar' se o voo ficou bom.")
    if osc <= 0.1 and max(rms(erroR), rms(erroP)) > 4:
        sugestoes.append("Resposta lenta/derivando sem tremor: aumente ang 15% ou ri 20%.")
    if satur > 0.2:
        sugestoes.append("Motores saturando muito: ganhos altos demais ou throttle de pairar perto do limite.")
    media = [sum(a[m] for a in voo) / len(voo) for m in ("m1", "m2", "m3", "m4")]
    if max(media) - min(media) > 40:
        sugestoes.append(f"Motores desbalanceados em media (M1..M4 = {', '.join(f'{m:.0f}' for m in media)}): "
                         "centro de gravidade deslocado ou motor/helice mais fraco.")
    if rms([a["gY"] for a in voo]) > 60:
        sugestoes.append("Yaw girando muito: se o drone gira sozinho e acelera, a orientacao do yaw pode estar invertida (rode 'orient').")
    for s in sugestoes or ["Nada critico. Aumente rp em passos de 10% ate aparecer tremor e volte um passo."]:
        print(" -", s)


def ler_csv(caminho):
    with open(caminho, newline="") as f:
        return [{c: float(r[c]) for c in COLUNAS} for r in csv.DictReader(f)]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("porta", nargs="?", help="porta serial, ex.: COM3")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--grafico", action="store_true", help="grafico ao vivo (matplotlib)")
    ap.add_argument("--analisar", metavar="CSV", help="so analisa um log salvo")
    ap.add_argument("--cmd", action="append", default=[], help="comando enviado ao drone ao conectar (repetivel)")
    args = ap.parse_args()

    if args.analisar:
        analisar(ler_csv(args.analisar))
        return
    if not args.porta:
        ap.error("informe a porta (ex.: COM3) ou --analisar arquivo.csv")

    import serial  # pip install pyserial

    os.makedirs(PASTA_LOGS, exist_ok=True)
    caminho = os.path.join(PASTA_LOGS, datetime.datetime.now().strftime("voo_%Y%m%d_%H%M%S.csv"))
    arq = open(caminho, "w", newline="")
    escritor = csv.writer(arq)
    escritor.writerow(COLUNAS)

    # Sem DTR/RTS para não resetar o ESP32 ao abrir a porta
    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = args.porta, args.baud, 0.2
    ser.dtr = False
    ser.rts = False
    ser.open()

    amostras = []
    janela = collections.deque(maxlen=500)
    rodando = threading.Event()
    rodando.set()

    def leitor():
        ultimo_status = 0
        while rodando.is_set():
            try:
                linha = ser.readline().decode(errors="replace").strip()
            except (serial.SerialException, OSError) as e:
                # o controle pode reiniciar/reenumerar a USB: tenta reconectar em vez de sair
                print(f"\n[serial] {e} -- reconectando...")
                try:
                    ser.close()
                except Exception:
                    pass
                while rodando.is_set():
                    time.sleep(1.0)
                    try:
                        ser.open()
                        ser.write(b"log 2\n")
                        print("[serial] reconectado")
                        break
                    except (serial.SerialException, OSError):
                        continue
                continue
            if not linha:
                continue
            a = interpretar(linha)
            if a is None:
                print(f"\r{linha}")
                continue
            amostras.append(a)
            janela.append(a)
            escritor.writerow([a[c] for c in COLUNAS])
            if len(amostras) % 25 == 0:
                arq.flush()
            if time.time() - ultimo_status > 0.5 and not args.grafico:
                ultimo_status = time.time()
                estado = "ARM" if int(a["flags"]) & 1 else "---"
                osc = " OSC!" if int(a["flags"]) & 8 else ""
                print(f"\r[{estado}] R{a['roll']:6.1f} P{a['pitch']:6.1f} gY{a['gY']:6.0f} "
                      f"M {a['m1']:.0f}/{a['m2']:.0f}/{a['m3']:.0f}/{a['m4']:.0f} "
                      f"esc {a['escala']:.2f}{osc} {a['loopHz']:.0f}Hz   ", end="", flush=True)

    def teclado():
        while rodando.is_set():
            try:
                cmd = input()
            except EOFError:
                break  # sem terminal (ex.: rodando em segundo plano): só não lê comandos
            except KeyboardInterrupt:
                rodando.clear()
                break
            if cmd.strip():
                ser.write((cmd.strip() + "\n").encode())

    threading.Thread(target=leitor, daemon=True).start()
    threading.Thread(target=teclado, daemon=True).start()
    time.sleep(0.3)
    for c in ["log 2"] + args.cmd + ["ganhos"]:
        ser.write((c + "\n").encode())
        time.sleep(0.3)
    print(f"Gravando em {os.path.abspath(caminho)}. Digite comandos (ajuda). Ctrl+C para sair.")

    try:
        if args.grafico:
            import matplotlib.pyplot as plt
            from matplotlib.animation import FuncAnimation
            fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
            fig.canvas.manager.set_window_title("Drone - telemetria")

            def atualizar(_):
                if not janela:
                    return
                d = list(janela)
                t = [(a["ms"] - d[-1]["ms"]) / 1000 for a in d]
                for ax in (ax1, ax2, ax3):
                    ax.cla()
                ax1.plot(t, [a["roll"] for a in d], label="roll")
                ax1.plot(t, [a["spR"] for a in d], "--", label="alvo roll")
                ax1.plot(t, [a["pitch"] for a in d], label="pitch")
                ax1.plot(t, [a["spP"] for a in d], "--", label="alvo pitch")
                ax1.set_ylabel("graus"); ax1.legend(loc="upper left", fontsize=8)
                ax2.plot(t, [a["gR"] for a in d], label="taxa roll")
                ax2.plot(t, [a["gP"] for a in d], label="taxa pitch")
                ax2.plot(t, [a["gY"] for a in d], label="taxa yaw")
                ax2.set_ylabel("graus/s"); ax2.legend(loc="upper left", fontsize=8)
                for m in ("m1", "m2", "m3", "m4"):
                    ax3.plot(t, [a[m] for a in d], label=m.upper())
                ax3.set_ylabel("PWM"); ax3.set_xlabel("s"); ax3.legend(loc="upper left", fontsize=8)
                a = d[-1]
                fig.suptitle(f"escala {a['escala']:.2f}  osc R {a['oscR']:.0f} Hz / P {a['oscP']:.0f} Hz  loop {a['loopHz']:.0f} Hz")

            anim = FuncAnimation(fig, atualizar, interval=200, cache_frame_data=False)  # noqa: F841
            plt.show()
        else:
            while rodando.is_set():
                time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        rodando.clear()
        time.sleep(0.3)
        ser.close()
        arq.close()
        analisar(amostras)
        print(f"\nLog: {os.path.abspath(caminho)}")


if __name__ == "__main__":
    main()
