# Plano — próximas sessões

## Situação (25/11/2026)

O sensor (MPU-6050) estava sobre uma espuma macia. Com ela, ele balançava em relação ao frame, e isso causava vibração, inclinação falsa e o ângulo estimado "andando" em voo. Depois que a espuma foi retirada, o drone **pairou baixo, estável e suave**. Ainda deriva levemente para um lado, mas tenta compensar.

- **Firmware aprovado**: branch `main`, igual ao que está gravado no drone.
- **Mudança planejada, não gravada**: branch `estimador-bias-giro`, um estimador do desvio do giro em voo.

## Amanhã, etapa 1: leituras sem mudar nada

Regra: **não alterar nenhuma configuração antes de terminar as leituras.** A ferramenta só envia comandos de leitura.

1. Ligue o drone na bateria, com o controle ligado na USB do PC.
2. Drone parado numa superfície plana: `python ferramentas/leitura.py COM4 repouso`
3. Drone segurado firme pelo corpo, com os braços livres: `python ferramentas/leitura.py COM4 vib` (gira os motores; pede confirmação).
4. Voo de 20–30 s no modo LEVE, pairando baixo, sem mexer no analógico 2: `python ferramentas/leitura.py COM4 voo` (Ctrl+C depois de desarmar).
5. Anote **para onde e com que velocidade** ele deriva.

Cada leitura fica em `logs/leitura_<data>_<hora>/resumo.txt`, que já traz a comparação com o voo de referência com espuma (224108).

## Etapa 2: decidir com base nas leituras

| O que a leitura mostrar | O que fazer depois |
|---|---|
| Ângulo estimado parado em voo (anda < 0,05 °/s) e motores sem divergir | Não aplicar o estimador de desvio. Corrigir a deriva leve com `nr`/`np` em passos de 0,5°. |
| Ângulo ainda andando (> 0,1 °/s) ou motores divergindo continuamente | Gravar a branch `estimador-bias-giro` e repetir a mesma leitura de voo para comparar. |
| Vibração de algum motor > ~50 °/s no `vib` | Mecânica primeiro: hélice ou motor daquele braço. |
| Desequilíbrio direita-esquerda ou trás-frente grande | Centralizar a bateria (centro de gravidade) antes de mexer no firmware. |

## Etapa 3: depois de estável

1. **Ajuste de nível** (`nr`/`np`) até a deriva ser mínima.
2. **Ganhos**: subir `rp`/`rd` em passos de 10%, um voo por passo, até aparecer tremor; depois voltar um passo.
3. **Limite de throttle** (`tmax`, hoje 860): subir aos poucos, observando se volta o evento violento em potência alta.
4. **Opcional**: para segurar posição e altura de verdade, um sensor de fluxo óptico (PMW3901) e um sensor de distância (VL53L1X).
