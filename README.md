# Braço Separador Inteligente

Braço robótico que detecta objetos por visão computacional e os separa
automaticamente. TCC de Engenharia da Computação — UniSALESIANO Araçatuba.

Arquitetura mestre-escravo: a **Raspberry Pi 5** roda o YOLO, calcula as
coordenadas e serve a dashboard; o **Arduino Nano** recebe comandos por serial
e aciona os servos via PCA9685.

## Estrutura

```
app.py                        dashboard Streamlit: câmera ao vivo + YOLO
coletar.py                    pipeline de coleta ponta a ponta (headless)
calibrar_camera.py            gera config/homografia.json a partir de 4 pontos
mapa_alcance.py               desenha em ASCII o que o braço alcança
pegada_manual.py              pega e solta em ângulo de servo, sem calibração
setup.sh                      instala dependências no Raspberry Pi
painel.sh                     sobe o Streamlit, separado do setup
MEDICAO_OFFSET.md             roteiro de medição de offset/sentido na bancada
PROXIMO_PASSO_CAMERA.md       passo a passo da calibração da câmera, no Pi

config/
  braco.json                  medidas e calibração — o arquivo do dia a dia
  homografia.json             gerado pela calibração (não versionado)

firmware/
  braco_firmware/             sketch único: operação + testes de bancada
  diagnostico/                sketch mínimo: pulso cru, para quando nada se move

models/
  yolov8n.pt                  pesos base; ainda não treinado nos objetos

src/
  arm/
    kinematics.py             cinemática inversa + tradução para ângulo de servo
    coleta.py                 sequência de pega
    mock_arm_serial.py        simula os servos; encaminha para o Arduino se houver
  vision/
    area_coleta.py            acha os cantos da área de coleta por bordas
    fonte.py                  orientação da câmera, aplicada na captura
    deteccao.py               escolhe e desenha a detecção do YOLO
    homografia.py             pixel -> mm
    estabilidade.py           decide quando a coordenada pode ser usada
    quadro_publicado.py       passa o frame da coleta para o painel
  camera.py                   visualizador standalone, sem Streamlit

tests/
  test_area_coleta.py         detecção dos cantos, em cena sintética
  test_firmware.py            dispatch, perfil de velocidade, tabela DEMO
  test_estabilidade.py        portão de estabilidade
  test_kinematics_acoplamento.py  paralelogramo: ângulo absoluto vs relativo
  test_pipeline.py            corrente visão -> braço, sem câmera
  test_quadro_publicado.py    publicação de frame entre processos
```

Rodar os testes (nenhum precisa de hardware):

```bash
for t in tests/test_*.py; do python "$t" || break; done
```

## As camadas, e por que estão separadas

**`Braco`** (em `kinematics.py`) só conhece geometria: comprimento dos elos e
cinemática. Não sabe o que é servo.

**`CalibracaoServo`** só conhece a montagem: offset, sentido de giro e limites
físicos de cada junta. Não sabe o que é cinemática.

Se o braço for remontado ou trocado, **só a segunda camada muda**. É por isso
que nenhuma dimensão de braço aparece hardcoded em lugar nenhum do código:
tudo vem do `config/braco.json`.

## Convenção angular

Mantenha idêntica em qualquer porte deste código:

- `theta_base` — rotação no plano XY, 0° = eixo +X
- `theta_ombro` — ângulo do elo L2 em relação à horizontal
- `theta_cotovelo` — ângulo do elo L3 **relativo** ao L2 (0° = esticado)

Nas coordenadas da mesa, a base do braço é a origem: **+X** para a direita,
**+Y** para a frente.

## Protocolo serial

Texto, uma linha por comando, **115200 baud**. O `MockArmSerial` e o
`braco_firmware.ino` falam o mesmo protocolo, então trocar simulação por
hardware real não muda nada acima da serial.

Operação — é só isto que o lado Python usa:

```
SET <canal> <ângulo>   ->  OK SET <canal> <ângulo>
GET <canal>            ->  POS <canal> <ângulo>
GET ALL                ->  POS ALL 0:<a0> 1:<a1> ...
HOME                   ->  OK HOME
BUSY                   ->  BUSY 0 | BUSY 1
STOP                   ->  OK STOP
<inválido>             ->  ERR ...
```

Bancada — existem para a calibração do passo 2:

```
DEMO [<graus_s>]         roda a sequência de pega e solta sozinho, sem PC
SWEEP <id> <de> <ate>    vai e volta na janela até um STOP
LIM                      mostra os limites de todas as juntas
LIM <id> <min> <max>     redefine o limite de uma junta
SPEED <graus_s>          velocidade média, 1–150 (padrão 60 °/s)
DETACH <id>|ALL          corta o PWM: dá para mexer a junta com a mão
ATTACH <id>|ALL          religa o PWM na posição atual
RAW <id> <pulso>         escreve a contagem de pulso crua
PULSO <id> <min> <max>   guarda a calibração de pulso achada com RAW
HELP                     lista os comandos
```

Sensor de corrente (ACS712 em série com o V+ da garra, lido em A0):

```
AMP                      AMP <corrente_A> <adc_medio> <adc_zero>
ZERO                     mede o zero do sensor agora (garra parada)
SENS <mV_por_A>          185 = versão 5 A, 100 = 20 A, 66 = 30 A
LIMAMP [<amperes>]       limite de travamento; 0 desliga; sem argumento mostra
```

Mensagens não solicitadas que o firmware pode emitir:

```
STALL <canal> <corrente>     a garra travou; o firmware está recuando
OK ALIVIO <canal> <angulo>   recuou até a corrente cair; parou aqui
ERR STALL_PERSISTE ...       recuou 15° e a corrente não caiu; desistiu
```

Canais: `0` base, `1` ombro, `2` cotovelo, `3` garra.

**Sketch único, de propósito.** Antes havia um firmware de produção e um de
bancada, com baudrates diferentes — trocar de sketch e esquecer disso faz a
serial devolver lixo e parecer defeito de hardware. Agora não há o que trocar.

## Sincronia entre o Python e o braço

O firmware suaviza os movimentos, então a duração deixou de ser fixa: fechar a
garra (100°) leva quase o triplo de descer até o objeto (33°). Por isso a
sequência de coleta **pergunta ao firmware quando ele parou** (`BUSY`) em vez
de esperar um tempo fixo.

Isso depende de o lado Python conseguir ouvir o Arduino, o que antes não
acontecia: o `readline()` do `MockArmSerial` devolvia sempre a resposta
simulada mesmo com hardware conectado, então um `ERR` do firmware nunca
aparecia. Agora, havendo Arduino, a resposta dele é a que vale.

`pausa_s` no `braco.json` deixou de ser a espera do movimento e passou a ser
só o tempo de assentamento depois da chegada — a estrutura de acrílico ainda
oscila um pouco quando o servo para.

## Alívio de travamento da garra

O ACS712 mede só a corrente do servomotor da garra. Quando ela passa de
`limite_stall_amp` por mais de 300 ms, o firmware assume que a garra fechou
sobre algo que não cede e **recua o ângulo comandado um grau por vez**, no
sentido de onde veio, até a corrente cair abaixo de 80% do limite. Onde ela
cai é o ponto de contato sem excesso de força, que para uma peça retida por
encaixe é exatamente a pega desejada.

Por que recuar, e não só parar: o firmware não tem realimentação de posição.
Se a garra está fisicamente travada em 105° com o comando em 110°, parar o
comando em 110° não resolve nada, o servomotor continua forçando. Só recuar o
comando até a corrente cair encontra o ponto real.

Salvaguardas: janela de graça de 300 ms para não confundir pico de partida com
travamento; recuo máximo de 15°, depois do qual desiste e avisa
(`ERR STALL_PERSISTE`), porque corrente que não cai com recuo não é
travamento mecânico; e uma trava que impede reentrar em alívio depois de
desistir, enquanto a corrente não normalizar. Sem essa trava, a garra recuaria
15° de cada vez até o limite da junta, soltando a peça.

**Vem desligado** (`limite_stall_amp: 0`). Para calibrar: mande `AMP` com a
garra fechando sobre a peça e veja o valor normal de pega; ponha o limite uns
30% acima disso e abaixo do valor que aparece com a garra forçando.

O zero do sensor é medido no boot, 600 ms depois de os servomotores assentarem,
e pode ser refeito com `ZERO`. Não é 511,5: cada peça tem offset próprio.

## Movimento suavizado

Todo `SET` é interpolado por uma curva *smootherstep* (`6p⁵ − 15p⁴ + 10p³`),
cuja primeira e segunda derivadas são zero nas duas pontas. Na prática: o
servo **parte do repouso e chega ao repouso**, sem degrau de velocidade.

Isso existe porque interpolar em linha reta — um `for` com `delay`, ou um
passo fixo por ciclo — suaviza o meio do movimento mas não as pontas. A
velocidade ainda salta de zero para o valor cheio na partida e cai a zero na
chegada, e aceleração instantânea é exatamente o que se sente como tranco.
É por isso que o tranco aparecia principalmente no retorno e no fim do sweep.

Duas consequências que valem saber:

- **`SPEED` é a velocidade MÉDIA.** `SPEED 60` quer dizer que um movimento de
  60° leva 1 segundo. O pico no meio do trajeto é 1,875× a média — por isso o
  teto é 150 °/s: acima disso o servo satura, não acompanha a curva, e a
  suavização deixa de valer.
- **Movimentos disparados juntos chegam juntos.** `SET`s recebidos dentro de
  50 ms contam como um lote e recebem todos a mesma duração. Sem isso, cada
  junta chegaria num instante diferente e a garra faria no ar um caminho
  diferente do que a cinemática planejou.

`STOP` é a exceção deliberada: ele congela na hora, sem desacelerar. É freio
de emergência, e desacelerar suave numa emergência não é o que se quer.

## Ordem de montagem

Cada etapa depende da anterior estar certa. Pular uma faz a seguinte falhar de
um jeito difícil de diagnosticar.

**1. Medir os elos** → `config/braco.json`, campo `elos_mm` — **já resolvido**

Os comprimentos saíram do CAD oficial do MeArm V0.41 em 01/11, lendo os eixos
dos furos de pivô com as transformações de montagem resolvidas. Não precisa
medir de novo. `l2 = l3 = 80,0`.

`l1 = 49,5` e `offset_radial = 57,8` não são medidas diretas de peça nenhuma,
são dobras: como o paralelogramo mantém a garra nivelada, o trecho punho →
ponto de preensão é um vetor constante no mundo e soma direto nas constantes
do modelo. O detalhe está em `_elos_medidos`, dentro do próprio `braco.json`,
e em `MEDICAO_OFFSET.md`.

Alcance resultante: 218 mm no máximo, 210 mm na altura da superfície.

**2. Achar os limites das juntas** → `config/braco.json`, campo `juntas`

Com o `braco_firmware` carregado, junta por junta. Baixe a velocidade antes
(`SPEED 10`) e deixe o `STOP` à mão:

- `minimo`/`maximo` — use `SWEEP <id> <de> <ate>` em janelas de 20°, **nunca**
  de ponta a ponta. Se a junta bater num limite mecânico no meio do curso, o
  servo trava e continua puxando corrente. É assim que se descasca engrenagem
  de MG90S. Ao ouvir o servo forçar, mande `STOP`.
- `DETACH <id>` solta a junta para você achar o curso com a mão antes de
  acionar o servo — é o jeito mais seguro de começar.
- `offset` — que ângulo de servo deixa o elo no 0° matemático
- `sentido` — mande 90, depois 110; se o elo foi para o lado contrário do
  esperado, use `-1`

Neste braço o cotovelo é acionado por paralelogramo, então o ângulo dele tende
a ser medido em relação à horizontal, não ao elo do ombro. Isso costuma
aparecer como um `offset`/`sentido` contra-intuitivo — é esperado, não é erro
de medida.

Ao terminar, marque `"calibrado": true`.

**3. Conferir o alcance** → `python mapa_alcance.py`

Rode antes de fixar a área de coleta em definitivo. Os símbolos importam:

- muitos `x` (limite de junta) → limites apertados demais, ou área longe da base
- muitos `.` (fora do envelope) → o braço é curto para essa área

**4. Fixar a câmera e calibrar** → `python calibrar_camera.py`

A câmera está montada de cabeça para baixo, e isso é corrigido em
`camera.rotacao_graus` no `braco.json`, hoje em `180`. O giro é aplicado no
instante da captura, nos quatro lugares que abrem câmera, então tudo abaixo
— homografia, detecção, painel, prévia — vê a imagem em pé.

Se a câmera for remontada em pé, volte o campo para `0`. Nos dois casos,
**recalibre**: girar o quadro muda o pixel de cada ponto, e uma homografia
antiga passa a apontar errado sem avisar.

A área de coleta é um retângulo, então os cantos dela não precisam ser
clicados: a câmera acha sozinha, por detecção de bordas. O que você informa
são as dimensões da placa e onde ela está:

```bash
python calibrar_camera.py --largura 300 --altura 200 --x0 -150 --y0 60 --conferir
```

`x0`/`y0` são o canto mais próximo da base e mais à esquerda, em mm, com
origem no eixo da base (+X direita, +Y frente). Quatro números que só mudam
se a maquete for remontada.

**O script salva uma prévia em `config/previa_calibracao.png`. Abra.** A
correspondência entre canto da imagem e canto da mesa pode sair girada se a
câmera estiver montada de lado, e isso **não aparece em número nenhum** — com
4 pontos a matriz fica exata de qualquer jeito, e o erro só se manifesta no
braço indo para o lugar errado. Se estiver girada, repita com `--rotacao 1`
(ou 2, ou 3).

Pelo mesmo motivo, o residual que o script relata sobre os 4 cantos é ~0 por
construção: 8 equações para as 8 incógnitas da matriz. Ele não valida nada.
Quem valida é o `--conferir`, que pede pontos medidos com régua que não
entraram no cálculo. É esse número que vale como erro de calibração, e é ele
que deve ser citado no relatório.

Se a detecção não pegar a placa — contraste ruim, placa cortada no quadro —
tem `--clique` e `--manual` com o fluxo antigo.

A calibração vale enquanto câmera, mesa e base não se moverem. Se qualquer um
sair do lugar, refaça.

**5. Dry-run** → `python coletar.py --dry-run`

Mostra as coordenadas detectadas e os comandos que sairiam, sem abrir a
serial. Confira se os milímetros batem com onde o objeto está de verdade
antes de energizar os servos.

**6. Primeira coleta real**, com a mão no interruptor da fonte.

**7. Treinar o YOLO** nos objetos do projeto. Por último de propósito: é a
etapa mais demorada, e detecção perfeita não adianta se o braço não alcança.

## A dashboard

Só visão: câmera ao vivo com as bounding boxes desenhadas e contagem dos
objetos que passam.

**O que procurar vem da seção `visao` do `braco.json`**, o mesmo arquivo que o
`coletar.py` lê:

```json
"visao": { "classe_alvo": "mouse", "confianca_minima": 0.5 }
```

Um lugar só, de propósito. Antes a classe estava na linha de comando da coleta
e numa lista fixa no `app.py`, e trocar o objeto de teste num deles deixava o
outro procurando outra coisa: a detecção era descartada antes de qualquer
desenho, o painel não mostrava caixa nenhuma, e nada na tela dizia por quê.
A barra lateral agora mostra a classe e o limiar em uso.

**Quando o `coletar.py` está rodando, o painel mostra o quadro publicado por
ele**, já com a caixa desenhada, e não roda YOLO próprio. Um processo por vez
consegue a câmera no Pi, e pagar a inferência duas vezes na mesma máquina não
traria nada.

**A dashboard não move o braço.** Isso é deliberado: enquanto a maquete não
está montada e a homografia não está calibrada, acionar servo por interface
gráfica só esconde o que está acontecendo. Durante a montagem, use as
ferramentas de linha de comando:

```
python src/arm/mock_arm_serial.py    terminal interativo: SET / GET / HOME
python mapa_alcance.py               o que o braço alcança, sem hardware
python calibrar_camera.py            gera a homografia
```

A coleta de verdade é o `coletar.py`, que roda sem interface.

## O pipeline de coleta

`python coletar.py` liga a corrente inteira:

```
captura frame
  -> YOLO detecta o objeto de maior confiança
  -> portão de estabilidade acumula detecções até o objeto parar
  -> homografia converte a base da caixa em milímetros
  -> cinemática inversa converte em ângulos de servo
  -> sequência de coleta manda os SET pela serial
  -> rearma e espera o próximo objeto
```

**Por que esperar o objeto parar.** Duas razões, e a segunda é a menos óbvia:
com o movimento suavizado o braço leva ~2 s para chegar, então um objeto ainda
escorregando seria agarrado onde estava e não onde está; e mesmo com o objeto
totalmente parado a caixa do YOLO treme alguns pixels de frame para frame, e
uma detecção isolada carrega esse ruído inteiro para dentro da coordenada.

O portão resolve os dois: guarda as últimas `janela` detecções e só libera
quando todas cabem dentro de `tolerancia_mm`. Se cabem, o objeto está parado
**e** a mediana delas é uma coordenada melhor que qualquer uma sozinha. Os
parâmetros ficam na seção `estabilidade` do `braco.json`.

A tolerância é medida em milímetros e não em pixels de propósito: a grandeza
que importa é física — "o objeto se mexeu menos que a folga da garra" — e o
mesmo número de pixels vale distâncias diferentes conforme a posição na mesa.

**A sequência de pega não é top-down.** A peça do projeto é um haltere: um
chapéu de 50 mm sobre um tronco de 20 mm. Descer no tronco é descer no chapéu,
então a garra recua no raio, desce ao lado e entra de fora para dentro:

```
abre -> sobrevoa 30 mm antes do alvo -> desce ao lado, 5 mm abaixo da pega ->
entra no tronco subindo -> fecha -> sobe -> leva -> solta -> HOME
```

O recuo é **radial**, mesmo ângulo de base. Com a base parada na etapa de
entrada, a garra avança exatamente na direção em que os dedos abrem; um recuo
lateral a faria chegar de través e bater na peça com a face externa dos dedos.

O trecho final sobe. Errar para baixo empurra a peça contra a mesa ou a tomba;
errar para cima só fecha a garra um pouco acima do meio do tronco.

Isso **encurta a área útil**: a sequência completa só fecha entre 121 e 188 mm
do eixo, contra 92 a 194 mm da pose de pega sozinha. Quem sobe o limite
interno é o ponto de descida, 25 mm mais perto do eixo, que cai no trecho onde
o braço não dobra o suficiente. A borda de trás da área de coleta está a
130 mm, com 9 mm de folga.

**`python coletar.py --dry-run`** mostra os comandos sem abrir a serial. É o
passo 5 do roteiro. O script se recusa a rodar sem `"calibrado": true` e sem
`config/homografia.json`, porque apontar o braço com placeholder é exatamente
como se descasca engrenagem.

## Decisões que valem saber

**A garra fica fora da cadeia de IK.** São 4 servos, mas só 3 definem a posição
da ponta. A garra só abre e fecha, então a IK resolve 3 incógnitas, não 4.

**Não há PID, e é intencional.** MG90S e SG90 têm controle de posição interno
(potenciômetro no eixo). O microcontrolador manda o ângulo-alvo; o controle
fino é do próprio servo. PID externo só faria sentido com motor DC cru.

**IK analítica por lei dos cossenos, não iterativa.** Com 3 juntas a solução
fechada é exata e mais barata que FABRIK ou Jacobiana.

**Os dois ramos da IK são resolvidos, e quem escolhe são os limites.** Uma
cadeia planar de dois elos alcança cada ponto por duas poses espelhadas; a lei
dos cossenos dá o módulo do ângulo e o sinal escolhe o ramo. Até 03/11 só
existia o ramo positivo, e isso recusava a área de coleta inteira: com a
referência do ombro em 200° o elo nunca desce abaixo de 20° da horizontal, e o
ramo positivo pedia ombro em −74° para qualquer alvo na altura da mesa. Quem
leva a garra até a superfície é o antebraço apontando para baixo, que é o ramo
negativo. `resolver_ik_ramos()` devolve os dois e `pose_para_servos()` fica com
o primeiro que couber nos limites.

**Os ângulos da garra são configuração, não dedução.** `garra_aberta_graus` e
`garra_fechada_graus` no `braco.json`. Deixá-los em `null` faz a coleta cair no
mínimo/máximo da junta, que é chute em dois sentidos: qual extremo fecha
depende de como a garra foi montada — montada ao contrário, ela abre na hora
de agarrar — e ir sempre até o limite mecânico ignora que as peças de teste
são impressas com região de preensão dimensionada para um curso específico.
Com *form closure* a retenção é geométrica; apertar além do encaixe só aquece
o servo.

**A coleta valida o plano inteiro antes de mover.** Se alguma pose da sequência
for impossível, nada é enviado — evita o braço parar no meio do trajeto com o
objeto preso na garra.

**A homografia usa a base da bounding box, não o centro.** A base é onde o
objeto encosta na mesa, que é o plano da calibração. Usar o centro erra cada
vez mais conforme o objeto é mais alto.

**O portão descarta por intervalo entre detecções, não por idade da
amostra.** A diferença parece cosmética e não é: por idade, fechar a janela
exige `janela / tempo` detecções por segundo — 5 em 1 s dá 4 FPS — e abaixo
disso o portão **nunca** fecha, com o objeto imóvel e dispersão zero, porque a
amostra mais velha expira antes de a última chegar. O YOLOv8n no Pi 5 faz 2,6
a 3 FPS, e o contador ficava preso em 4/5. Medido por intervalo, a taxa de
inferência decide quanto tempo o portão leva, não se ele fecha.

**Uma classe por vez na coleta, não uma lista.** A mão que ajusta a peça
pontua como `person` acima do próprio objeto, e `melhor_deteccao` devolve a
caixa de maior confiança: a caixa troca de objeto no meio da janela e a
sequência terminaria mandando o braço buscar a mão.

**Detecção estática, não rastreamento.** O objeto está parado quando é
detectado, então 5–8 FPS do YOLOv8n no Pi 5 bastam: uma inferência
bem-sucedida já dá as coordenadas. É o argumento que justifica o Raspberry Pi 5
em vez de uma Jetson.

## O que ainda não está resolvido

Atualizado em 09/11/2026.

- **O modelo ainda usa classes COCO**, não os objetos do projeto. O teste de
  bancada roda com `mouse` (0,98 de confiança); o celular ficava em 0,40 a
  0,56, porque o COCO espera ver celular na mão e não largado numa placa
  clara. Treinar nas peças do projeto é a etapa que falta.
- **A base da bounding box não é o centro do objeto.** A homografia converte o
  pixel mais baixo da caixa, que numa vista oblíqua é a borda do objeto virada
  para a câmera — não o meio do apoio. O viés é de meia profundidade do
  objeto, e para uma peça de 50 mm seriam 25 mm, dez vezes o erro da
  homografia. Falta a medição de régua que confirma o sinal e o tamanho; a
  correção é somar meia profundidade na direção radial.
- **O alívio de travamento nunca rodou com a garra real.** A leitura do ACS712
  e a lógica de alívio foram testadas só por simulação. `limite_stall_amp`
  está em 0: a corrente de pega medida até agora chega a 0,5 A, mas sem a
  corrente de travamento o limite seria arbitrário.
- **A coleta nunca rodou com os servos energizados.** O `--dry-run` fechou
  ponta a ponta em 09/11, com câmera, YOLO, homografia, portão e cinemática.
  O que falta é o passo 6, com a mão no interruptor.
- **O tempo mecânico da garra é estimativa.** `VEL_MECANICA_GARRA = 300 °/s`
  dimensiona o `BUSY`; se o fechamento real for mais lento, o lado Python
  manda a pose seguinte com a garra a meio curso. Só aparece na coleta de
  verdade.
- **O modo de falha da garra não foi reproduzido desde a correção.** Ela não
  fechava com o braço estendido: 17° de curso a 10 °/s davam menos de uma
  contagem do PCA9685 por tique, e a garra tremia sem andar. Tirar a
  suavização resolveu, e o curso hoje é de 40°. Se voltar, o `AMP` separa
  subalimentação (corrente baixa) de travamento mecânico (corrente alta).

### Resolvido desde a versão anterior deste arquivo

- Os limites de junta divergiam entre `braco.json`, o `.ino` e o
  `pegada_manual.py`. O `.ino` é a fonte, e `tests/test_firmware.py` falha se
  os três divergirem.
- A referência angular do cotovelo era dedução (46°, da pose em que a garra
  encosta na superfície) e estava errada. Medida na bancada: offset 180,
  sentido +1.
- A corrente completa câmera → braço nunca tinha rodado. Rodou em 09/11, em
  `--dry-run`.
- A homografia foi calibrada: 1,3 mm de erro médio e 2,2 mm máximo em três
  pontos de conferência, contra um critério de 5 mm.
