# ANTIGRAVITY.md — instruções para o agente: ajuste ao vivo do drone

Este arquivo explica ao agente do Antigravity como **ajustar os ganhos do drone durante o voo**, usando a mesma ferramenta e as mesmas regras de segurança que já funcionaram neste projeto. Leia tudo antes de enviar qualquer comando.

## 1. O sistema em 30 segundos

- **Drone:** ESP32-S3 + MPU-6050, 4 motores brushless com ESC PWM, ~120 g, frame de PETG. Firmware em [drone/drone.ino](drone/drone.ino), branch `main`.
- **Controle (rádio):** ESP32-S3 + joysticks, firmware em [controle/controle.ino](controle/controle.ino). Ligado no PC pela USB, ele é a **ponte**: repassa ao drone, por ESP-NOW, as linhas de texto que recebe, e devolve a telemetria (`T,...`) e as mensagens (`# ...`) do drone.
- **Portas:** identifique sempre pelo **MAC**, nunca pelo número da COM.
  - Drone: `D8:85:AC:D4:3A:94`. Na USB só para gravar firmware; em voo, fica sem cabo.
  - Controle: `7C:4F:AD:1D:EF:D4`. É por ele que se faz o ajuste ao vivo.
  - Para descobrir a COM, use o "pai" PnP da porta: `Get-PnpDeviceProperty -InstanceId <id> -KeyName DEVPKEY_Device_Parent` (ele contém o MAC).
- **Python:** `C:\Python315\python.exe`, com `pyserial` instalado.

## 2. A ferramenta: `ferramentas/ao_vivo.py`

```
python -u ferramentas/ao_vivo.py COM4 <arquivo_de_comandos.txt>
```

- **Mantém a porta do controle aberta o tempo todo.** Não abra e feche a porta durante o voo: isso pode reiniciar o controle, e o drone desarma por perda de sinal.
- **Envia comandos:** cada linha **acrescentada** ao arquivo de comandos vai para o drone (ex.: `echo "rd 0.0016" >> cmd.txt`). Linhas que já existiam quando a ferramenta abriu são ignoradas.
- **Saída (stdout):** a cada 2 s, uma linha de métricas, mais as mensagens do drone:
  ```
  M t=82 arm=1 thr=839 | erroR 4.4 erroP 3.8 | vibR 56 vibP 39 | oscR 2.0Hz oscP 1.2Hz | P -0.1 alvoP -0.4 | fr-tr +13
  # rd = 0.0018            <- confirmação do drone
  > rd 0.0018             <- comando que a ferramenta enviou
  ```
  - `arm=1`: armado. `thr`: throttle médio (PWM; o drone **paira em ~815–840**).
  - `erroR/erroP`: erro RMS de ângulo, em graus. **Bom: < 1,5. Ruim: > 3.**
  - `vibR/vibP`: RMS da taxa de giro, em °/s. **Bom: < 15. Ruim: > 40. Perigo: > 80.**
  - `oscR/oscP`: frequência do balanço. 1–3 Hz é balanço lento, típico de laço de ângulo ou de integrador.
  - `P` × `alvoP`: pitch medido × pitch pedido. Se o `P` ficar abaixo do alvo, o nariz está erguido e o drone vai **para trás**.
  - `fr-tr`: (M1+M2) − (M3+M4). Positivo significa que a frente trabalha mais, ou seja, o peso está para a frente.
- O CSV completo fica em `logs/aovivo_<data>.csv` (50 Hz).
- Rode a ferramenta **em segundo plano** e leia a saída continuamente.

## 3. Comandos do drone (texto, um por linha)

| Comando | Faixa usada | Efeito |
|---|---|---|
| `ang <v>` | 2,0–3,0 | ganho do laço de ângulo; mais alto corrige mais forte e balança mais |
| `rp <v>` | 0,16–0,26 | P do laço de taxa (roll e pitch) |
| `ri <v>` | 1,0–2,0 | I do laço de taxa: compensa desequilíbrio. **2,5 piorou muito** |
| `rd <v>` | 0,0008–0,0018 | D do laço de taxa: amortece. **Foi o que mais ajudou** |
| `np <v>` / `nr <v>` | −1,0 a 0 | ajuste fino de nível (pitch/roll, em graus); + = frente / + = direita |
| `ganhos`, `estado`, `trim` | — | só leitura |
| `salvar` | — | grava na flash. **Só com autorização (ver §5)** |

Sem `salvar`, as mudanças valem até o drone reiniciar. **Se a bateria for trocada, o drone volta ao que está salvo**, e você precisa reaplicar os valores da sessão.

## 4. Protocolo de ajuste ao vivo

1. **Antes de começar**, envie `ganhos` e `estado`. Confira:
   - `armado=0`;
   - a última `Calibrado: giro off=(...)` perto de **(−2..−6, −12..−13, −8..−11)** e `acel` com z ≈ 1,07–1,08 g.
   - Se o zero do giro estiver muito fora (dezenas ou centenas) ou o z estiver longe de 1 g, **o drone se mexeu durante a calibração**. Peça ao piloto para deixá-lo parado e nivelado e apertar o **verde (AUX3)**. **Nunca deixe voar com calibração ruim.**
2. **Peça ao usuário UMA aprovação com limites explícitos** (os da tabela acima) antes do voo. Depois disso, você pode ajustar durante o voo dentro desses limites, sem perguntar a cada passo.
3. Com o drone voando, **observe ~6 s sem mexer** para ter a referência.
4. **Mude UM ganho por vez**, em passo pequeno (~10–20%), e espere **pelo menos 2–3 janelas (4–6 s) com o throttle perto do pairar** antes de julgar.
   - Comandos do piloto no joystick 2 (o `alvoP` mudando) e subidas de throttle **contaminam o erro**. Compare só trechos parecidos.
   - As confirmações (`# rp = ...`) podem chegar atrasadas, em lote. Confira se o valor foi aplicado antes de julgar o efeito.
5. **Se piorar, volte imediatamente ao valor anterior.**
6. **Avise o usuário em uma linha a cada mudança:** o que mudou, por quê e o resultado.
7. **Encerramento:** quando o usuário mandar **"Ç"**, pare de ajustar, confira `armado=0` e envie `salvar` e `ganhos`. Sem "Ç", **não salve**.

## 5. Regras de segurança (obrigatórias)

- **Nunca** envie `salvar`, `trim 0`, `cal`, `vib`, `orient` ou mudanças fora dos limites aprovados **sem aprovação explícita** do usuário para aquela ação.
- **Nunca** abra, feche ou reabra a porta do controle com o drone armado. Use uma única sessão do `ao_vivo.py`.
- `vib`/`vibm` **giram os motores**. Só com o drone **segurado pelo corpo** e com a confirmação do usuário.
- Se `vibR/vibP > 80` ou `erro > 8`, **mande o usuário pousar e desarmar (botão vermelho)** e pare de ajustar.
- Depois de pousar, lembre de **desarmar logo**: armado no chão, o drone pode tombar.
- Para gravar firmware, use **um único comando** que compila e grava (`arduino-cli compile -u -p COMx ...`). Compilar e gravar em terminais diferentes já gravou um binário antigo.

## 6. O que já se sabe (26/11/2026)

- **Configuração salva e aprovada** ("O último voo foi perfeito", 27,4 s estável em `thr=835–840`, `erroR 1,4–2,1°`, `erroP 0,4–1,5°`, `vibR 6–9 °/s`, `vibP 3–11 °/s`): `ang 2.5`, `rp 0.20`, `ri 1.25`, `rd 0.0018`, `np -0.5`, `nr 0.0`, trim aprendido `R 1.0 P -3.4 Y 28.7`, `thov 840`, `tmax 960`, modo LEVE com o joystick 2 até 5° (`aleve 5`). Reduzir `ri` de `1.5` para `1.25` junto com `rd 0.0018` eliminou o balanço lento de ~1 Hz no pairar.
- **Estável com o throttle abaixo de ~830** (erro < 1°, vibração 5–10 °/s). **Com 835–900 aparecem surtos de vibração (50–130 °/s)** em qualquer combinação de ganhos testada (ang 2–2,5, rp 0,17–0,23, ri 1,0–2,5, rd 0,0012–0,0018). **Isso é mecânico, não de sintonia:** não tente resolver só com ganhos.
- **No último voo, o M2 (dianteiro direito) foi o motor mais exigido** (até 1005 com throttle 929), com o nariz caindo mesmo com a frente ~78 acima. **Suspeita: a hélice ou o motor do M2 perdendo empuxo** (hélice frouxa ou patinando, motor ruim). Antes de sintonizar de novo, peça a inspeção:
  - aperto das hélices e dos parafusos dos motores, começando pelo M2 e pelo M1;
  - emendas e trincas do frame consertado;
  - firmeza da cruz da eletrônica sobre as colunas (se ela balança, o sensor balança junto).
- **Peso para a frente** (fr-tr de +25 a +50): o drone tende a ir **para trás**. A solução definitiva é mover a bateria ~1 cm para trás. O `np` só compensa parcialmente.
- O teste de vibração com motor único (`vibm1`..`vibm4 <pwm>`) só existe na branch `pouso-automatico`, não no `main`. No `main`, use `vib 740` ou `vib 820`, com o drone segurado pelo corpo.

## 7. Arquivos úteis

- [ferramentas/ao_vivo.py](ferramentas/ao_vivo.py): ajuste ao vivo (esta seção).
- [ferramentas/leitura.py](ferramentas/leitura.py): leituras só de leitura (`repouso`, `vib --confirmado`, `voo`).
- [ferramentas/sintonia.py](ferramentas/sintonia.py): gráfico ao vivo e análise de log.
- [README.md](README.md) e [PLANO.md](PLANO.md): visão geral e plano.
