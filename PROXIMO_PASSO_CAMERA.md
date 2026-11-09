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
| garra | abre em 105 (90 mm), fecha em 140 | limites da junta 100–140 |
| repouso | base **50**, ombro 120, cotovelo 120, garra 105 — garra a (72, 86), 59 mm de altura |
| limite acoplado | `s2 ≥ 285 − 1,5·s1 + 15`, validado em 5 poses |
| faixa útil da área de coleta | **121 a 188 mm** do eixo da base |

Medido em 09/11/2026 com a sequência de entrada lateral montada. Só a pose
de pega iria de 92 a 194 mm; o que sobe o limite interno para 121 é o ponto de
**descida ao lado**, 25 mm mais perto do eixo, que cai no trecho onde o braço
não dobra o suficiente.

A borda de trás da placa está a 130 mm, com 9 mm de folga.

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
a coordenada de cada um escrita ao lado. Vale nos três modos — automático,
`--clique` e, quando houver imagem, no manual.

**Confira que o canto marcado como perto da base é mesmo o perto da base**, e
que os sinais de X batem com o que você vê.

No automático, correspondência girada se corrige com `--rotacao 1` (ou 2, ou
3). No `--clique`, basta clicar um canto e digitar os milímetros de outro.

Nos dois casos isso **não aparece em número nenhum**: com 4 pontos a matriz
fica exata de qualquer jeito, e o erro só se manifesta no braço indo para o
lugar errado. Nem os pontos de conferência pegam, se tiverem sido medidos com
a mesma troca na cabeça.

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

## Passo 4 — FEITO em 09/11/2026

Calibrado no modo `--clique` (a detecção automática não pegou a placa: branco
sobre branco, 2 níveis de cinza de diferença).

Área de coleta: 100 x 100 mm, canto perto-esquerda em (-50, 130).

Erro nos pontos de conferência, que são os que não entraram no cálculo:

| ponto medido | reprojetado | erro |
|---|---|---|
| (0, 150) | (-0,5 , 151,0) | 1,1 mm |
| (-40, 185) | (-40,5 , 184,5) | 0,7 mm |
| (34, 210) | (33,4 , 212,1) | 2,2 mm |

**Médio 1,3 mm, máximo 2,2 mm.** Critério era 5 mm.

O erro cresce com a distância porque a câmera está oblíqua e a borda de trás
tem menos pixels por milímetro. O máximo de 2,2 mm coincide com a resolução
do PCA9685 na extremidade do braço (0,46 graus por contagem, ~2 mm): visão e
atuação erram da mesma ordem, e melhorar só uma não melhora o conjunto.

Refazer é obrigatório se a câmera, a área de coleta ou a base do braço se
moverem.

---

## Passo 5 — ensaio sem energia

```bash
python coletar.py --dry-run
```

Roda a cadeia inteira (câmera → YOLO → homografia → cinemática → comandos) e
imprime os `SET` que seriam enviados, sem mover nada.

O que olhar: se os ângulos fazem sentido, e se aparece alguma recusa de pose.
Recusa aqui é melhor que descobrir depois com o braço em movimento.

A sequência tem **9 etapas**, e não é top-down:

```
abre -> sobrevoa 30 mm ANTES do alvo -> desce ao lado, a 20 mm ->
entra no tronco subindo -> fecha -> sobe -> leva -> solta -> HOME
```

O desvio existe por causa do chapéu de 50 mm da peça: descer no tronco é
descer no chapéu. O recuo é **radial**, mesmo ângulo de base, então na etapa
de entrada a base fica parada e a garra avança na direção em que os dedos
abrem — por fora do chapéu, não por cima.

A altura do sobrevoo é escolhida na hora. Perto da base o envelope é um
corredor estreito (num raio de 100 mm o braço só alcança de 20 a 40 mm de
altura), então pedir 70 mm fixo recusaria os alvos próximos. O `--dry-run`
imprime a altura que usou em cada caso.

Alvo que não aceita nem o recuo mínimo de 25 mm — metade do chapéu — é
recusado. Recuar menos seria descer sobre o chapéu, que é o que a sequência
existe para evitar. A cobertura da área de coleta é a mesma de antes, 55%
dos pontos, e o que ficou de fora já estava de fora pelo alcance.

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

**A abertura da garra — RESOLVIDO em 08/11/2026.** Em `SET 3 100` as pinças
ficam retas, com ~100 mm de ponta a ponta. A peça tem extremidades de 50 mm,
então passa com folga de 50 mm e a aproximação vertical é viável. Era o único
item que podia inviabilizar a aproximação por cima.

A **operação abre em 105**, que dá 90 mm sem forçar. O 100 fica como
`lim_min` e nada o usa: ali as pinças ficam retas, o quadrilátero da garra
para no ponto de inflexão, a direção de fechamento fica ambígua e um esbarrão
vira o elo para o outro lado — com o comando de fechar empurrando para o lado
errado. Os 5° de recuo saem desse ponto.

**A pegada do dedo na descida.** Com 90 mm de abertura, cada dedo avança
45 mm para o lado do centro de preensão. Na descida sobre uma peça encostada
na borda da área de coleta, ou perto da parede do depósito, o dedo chega lá
antes da peça. Se isso aparecer, a saída é fechar um pouco o `garra_aberta_graus`:
a peça precisa de 50 mm, então há 40 mm para gastar.

**O portão de estabilidade foi medido no Pi, em 09/11/2026.** A taxa de
inferência ficou em ~3 FPS a imgsz 640, com 27% dos intervalos passando de
1 s. Daí o `intervalo_maximo_s = 2.5` no `braco.json`: em 1,0 s um quarto dos
quadros zerava a janela. Se você mudar o `--imgsz`, vale reconferir — o
`coletar.py` imprime a taxa e avisa se ela sozinha impedir o portão de fechar.

**A base do repouso está em 50, não em 90.** Com 90 a garra para 18 mm antes
da borda da placa, a 59 mm de altura, e numa vista oblíqua ela se projeta em
cima do canto próximo — o objeto que estiver ali não é detectado. Em 50 a garra
vai para (72, 86), fora da borda +X da placa. Se ainda aparecer no quadro, 40
ou 30 afastam mais; o depósito está do outro lado (base 129) e não atrapalha.

Para conferir sem adivinhar, com o braço energizado:

```
SET 0 50
SET 1 120
SET 2 120
```

e olhe o painel. A placa precisa aparecer inteira e limpa.

**O que o sistema procura está no `braco.json`, seção `visao`**, e os tres
consumidores leem de la: o `coletar.py`, o painel e o `src/camera.py`.

```json
"visao": { "classe_alvo": "mouse", "confianca_minima": 0.5 }
```

Trocar o objeto de teste e editar essa linha. O `--classe` do `coletar.py`
continua existindo para um teste rapido sem mexer no arquivo.

Uma classe so, nao uma lista: a mao que ajusta a peca pontua como `person`
acima do proprio objeto, a caixa troca de objeto no meio da janela, e no fim
o braco iria buscar a mao.

**O modelo ainda usa classes COCO.** O `coletar.py` detecta com os pesos base
do YOLOv8n. Treinar nos objetos do projeto é a etapa seguinte, e não bloqueia
a calibração da homografia.

**A garra não fechava com o braço estendido.** Melhorou muito depois de sair
do lote e perder a suavização, mas se voltar a acontecer, o `AMP` com ela
tremendo separa subalimentação (corrente baixa) de travamento mecânico
(corrente alta).
