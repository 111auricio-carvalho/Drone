# Drone ESP32-S3

Quadricóptero (~100 g), com ESP32-S3, MPU-6050 e 4 ESCs em PWM de 500 Hz, pilotado por um controle ESP32 via ESP-NOW.

| Pasta | Conteúdo |
|---|---|
| `drone/drone.ino` | Controladora de voo: PID em cascata, telemetria, sintonia ao vivo, auto-redução de ganho |
| `controle/controle.ino` | Rádio (joysticks e botões) e ponte de telemetria PC ⇄ drone |
| `ferramentas/sintonia.py` | Console no PC: log CSV, gráfico ao vivo, análise do voo com sugestões |
| `ferramentas/fusion_props/` | Script do Fusion que exporta massa, CG e inércia do modelo 3D |
| `legado/` | Versões originais, só para comparação |

## Compilar e gravar

Placa: **ESP32S3 Dev Module**, com **USB CDC On Boot: Enabled** (sem isso o `Serial` não aparece na USB).

```
arduino-cli compile -u -p COM3 --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc drone
```

## Controle de voo

```
ângulo ──(ang)──► taxa desejada ──PID (rp, ri, rd, D filtrado em dhz)──► mixer X ──► motores
yaw: PID de taxa (yp, yi)
```

Convenção dos eixos usada pelo mixer: **roll + = lado direito desce**, **pitch + = nariz desce**, **yaw + = nariz gira para a direita**.

## Antes do primeiro voo (sem hélices!)

1. Ligue o drone na USB e rode `python ferramentas/sintonia.py COM3`.
2. Digite `orient` e faça os 2 movimentos pedidos: lado direito para baixo e volte (segure parado), depois nariz para baixo e volte. Os eixos são detectados e conferidos com a gravidade. O yaw é deduzido do eixo vertical. Tudo é gravado na flash.
   Orientação medida neste drone: `roll=-Y pitch=+X yaw=-Z` (MPU girado 90°, face para cima).
3. Confira: inclinando o lado direito para baixo, `R` deve ficar **positivo**; com o nariz para baixo, `P` deve ficar **positivo**.

## Sintonia com feedback

- **Na bancada:** drone na USB.
- **Em voo:** controle na USB. A telemetria chega pelo ESP-NOW e os comandos digitados seguem para o drone.

| Comando | Função |
|---|---|
| `ganhos` / `estado` | mostra configuração e estado atual |
| `ang` `rp` `ri` `rd` `yp` `yi` `dhz` `imax` `<valor>` | altera o ganho na hora |
| `auto 0/1` | liga/desliga a redução automática de ganho ao detectar tremor |
| `aplicar` | incorpora a escala automática em `rp`/`rd` |
| `salvar` / `padrao` | grava na flash / volta aos valores padrão |
| `log 0/1/2` | telemetria desligada / 10 Hz / 50 Hz |
| `cal` / `orient` | recalibra o giroscópio / calibração guiada de orientação |

Em voo, o detector de oscilação mede o tremor (≥ 6 Hz e > 40 °/s). Quando detecta, reduz a escala de `rp`/`rd` em 15% por segundo, até no mínimo 0,35. Ao sair do programa (Ctrl+C), ele analisa o log e sugere ajustes.
