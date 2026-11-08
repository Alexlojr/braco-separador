# Próximo passo — câmera e homografia

Estado em 08/11/2026. A calibração do braço está **fechada**
(`"calibrado": true`). O que falta para a coleta automática é a câmera.

Este arquivo é para ser lido no Pi, durante o trabalho.

---

## Onde paramos

| | |
|---|---|
| elos | do CAD: `l2 = l3 = 80,0`, `l1 = 49,5`, deslocamento radial `57,8` |
| base | offset 0, sentido +1, 0–180 |
| ombro | offset 200, sentido −1, 90–180 |
| cotovelo | offset 180, sentido +1, 70–160 |
| garra | 120 aberta, 137 fechada |
| limite acoplado | `s2 ≥ 285 − 1,5·s1 + 15`, validado em 5 poses |
| faixa útil da área de coleta | **115 a 190 mm** do eixo da base |

Os 115–190 mm são a interseção entre conseguir pegar a 25 mm de altura e
conseguir sobrevoar a 70 mm. Fora dessa faixa, uma das duas falha.

---

## Passo 0 — no Pi, trazer o código

```bash
cd ~/braco-separador
git pull
./setup.sh
```

O `setup.sh` instala `python3-picamera2` e `rpicam-apps` pelo apt (não existem
no pip), cria o venv com `--system-site-packages` para enxergá-los, e instala
o `requirements.txt`.

Depois, em toda sessão nova:

```bash
cd ~/braco-separador && source venv/bin/activate
```

Conferir que a correção de orientação veio junto:

```bash
python -c "from src.vision.fonte import rotacao_configurada; print(rotacao_configurada())"
```

Tem que imprimir **180**. A câmera está montada de cabeça para baixo e o giro
é aplicado na captura, então todo o resto vê a imagem em pé.

---

## Passo 1 — a câmera liga?

```bash
rpicam-hello --timeout 2000
```

Se não aparecer imagem, o problema é de cabo ou de habilitação, e nada do
resto adianta. Cabo CSI tem lado: os contatos vão virados para a placa.

---

## Passo 2 — fixar a câmera

Ela precisa ficar **imóvel em relação à mesa e à base do braço**. Se qualquer
um dos três se mexer depois, a calibração morre e tem que refazer.

Enquadrar de modo que a área de coleta apareça **inteira**, com alguma folga
nas bordas. A detecção automática procura o maior quadrilátero convexo da
imagem — placa cortada pela borda do quadro não vira quadrilátero.

Vale conferir o enquadramento antes de parafusar:

```bash
rpicam-jpeg -o /tmp/enquadramento.jpg --timeout 2000
```

---

## Passo 3 — medir a placa (quatro números, uma vez)

Com régua, na mesa:

| número | o que é |
|---|---|
| `--largura` | largura da área de coleta, em mm |
| `--altura` | profundidade dela, em mm |
| `--x0` | X do canto mais **próximo da base** e mais à **esquerda** |
| `--y0` | Y desse mesmo canto |

Convenção: origem no **eixo de giro da base**, **+X para a direita**, **+Y
para a frente**. O canto perto-esquerda de uma placa de 300×200 mm centrada à
frente da base, começando a 60 mm, seria `--x0 -150 --y0 60`.

Antes de fixar as placas, confira contra a faixa útil:

```bash
python mapa_alcance.py --z 25
python mapa_alcance.py --z 70
```

O `#` tem que cobrir a área de coleta nos **dois**. Um ponto que aparece em
z=25 mas não em z=70 é um ponto onde o braço pega e não consegue sobrevoar.

---

## Passo 4 — calibrar

```bash
python calibrar_camera.py --largura 300 --altura 200 --x0 -150 --y0 60 --conferir
```

(com os seus números no lugar dos do exemplo)

### 4a. ABRA A PRÉVIA

O script salva `config/previa_calibracao.png` com os quatro cantos marcados e
a coordenada de cada um escrita ao lado.

**Confira que o canto marcado como perto da base é mesmo o perto da base.**

Se a correspondência estiver girada, rode de novo com `--rotacao 1` (ou 2, ou
3). Isso **não aparece em número nenhum**: com 4 pontos a matriz fica exata de
qualquer jeito, e o erro só se manifesta no braço indo para o lugar errado.

### 4b. O residual dos 4 cantos não vale nada

O script vai imprimir algo perto de zero e dizer isso ele mesmo. Uma
homografia tem 8 graus de liberdade e cada par de pontos dá 2 equações: com 4
pares o sistema é exatamente determinado, e a matriz passa pelos pontos
estejam eles certos ou errados.

### 4c. O residual que vale

O `--conferir` pede pontos que **não entraram no cálculo**. Marque dois ou
três pontos na área de coleta, meça cada um com régua, e localize o pixel de
cada um na prévia. Informe no formato `u,v,x,y`.

O erro médio relatado aí é o número de verdade — e é ele que vai no relatório,
onde hoje está o `[PREENCHER]` da seção 4.4.3. Critério: **5 mm**.

---

## Passo 5 — ensaio sem energia

```bash
python coletar.py --dry-run
```

Roda a cadeia inteira (câmera → YOLO → homografia → cinemática → comandos) e
imprime os `SET` que seriam enviados, sem mover nada.

O que olhar: se os ângulos fazem sentido, e se aparece alguma recusa de pose.
Recusa aqui é melhor que descobrir depois com o braço em movimento.

---

## Passo 6 — primeira coleta de verdade

Antes de ligar a alimentação:

- o braço tem que estar **perto da pose de repouso**. No boot ele fica solto
  por ~550 ms e depois assume a pose de uma vez, sem suavizar;
- mão no `STOP`;
- `SPEED 40` para a primeira rodada.

```bash
python coletar.py
```

---

## O que ainda está em aberto

**A abertura da garra.** O curso é de 17° (120 a 137). A peça tem extremidades
de 50 mm e tronco de 20 mm, e a aproximação é vertical — os dedos precisam
passar **por fora dos 50 mm** antes de descer no tronco.

```
SET 3 120
```

e meça a abertura entre os dedos com régua. **Se não passar de 50 mm, a
aproximação vertical não funciona com essa peça**, e aí é afinar as
extremidades ou aumentar o curso da garra, se houver folga mecânica além do
120.

Isso é independente da câmera e pode ser medido a qualquer momento.

**O modelo ainda usa classes COCO.** O `coletar.py` detecta com os pesos base
do YOLOv8n. Treinar nos objetos do projeto é a etapa seguinte, e não bloqueia
a calibração da homografia.

**A garra não fechava com o braço estendido.** Melhorou muito depois de sair
do lote e perder a suavização, mas se voltar a acontecer, o `AMP` com ela
tremendo separa subalimentação (corrente baixa) de travamento mecânico
(corrente alta).
