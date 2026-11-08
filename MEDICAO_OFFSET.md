# O que medir no braço

Estado em 05/11/2026.

## ESTE ROTEIRO JÁ FOI PERCORRIDO

Sobrou uma conferência de dez segundos. O resto está em `config/braco.json`.

| parte | estado |
|---|---|
| 0 — sentido e comprimentos | resolvido, CAD |
| 2 — ombro | **resolvido**, referência 200° |
| 3 — antebraço | descartada: as leituras não passaram na autoconferência |
| 4 — base | **resolvido**, referência 0° |
| 5 — limite acoplado | **medido**, virou a reta `s2 ≥ 285 − 1,5·s1` |

O cotovelo não saiu da parte 3: saiu da parte 5, por dedução geométrica a
partir da pose em que a garra encosta na superfície. Referência **46°**.

**O que falta:** `SET 2 136` deve deixar o antebraço na vertical apontando
para baixo. Se deixar, a referência 46 está certa e dá para marcar
`"calibrado": true`. Se ficar inclinado, o erro em graus soma direto no 46.

Opcional, para confirmar a reta de colisão com um terceiro ponto: em
`SET 1 115` a reta prevê colisão em `SET 2 112`.

O texto abaixo é o roteiro original, mantido porque explica o porquê de cada
medida e porque a parte 3 pode precisar ser refeita se a conferência falhar.

---

## Resumo

O CAD do MeArm V0.41 resolveu os comprimentos. **Você não precisa mais
medir elo nenhum.** Falta só uma coisa: em que ângulo de servo cada junta
está. Para isso são 2 números por pose, com régua, num papel.

---

## Parte 0 — o que já está resolvido

**Sentido**: base +1, ombro −1, cotovelo −1. Já no `config/braco.json`.

**A foto espelhada não atrapalhou.** Espelhar não muda o tamanho do ângulo
com a mesa; só troca qual lado é a frente, e isso você já sabe.

**Os comprimentos, agora do CAD.** Abri o STEP montado, resolvi as
transformações de montagem e li os eixos dos furos de pivô:

| medida | valor |
|---|---|
| ombro → cotovelo | **80,0 mm** |
| cotovelo → punho | **80,0 mm** |
| base → ombro (horizontal) | 14,7 mm |
| punho → ponto de preensão | 43,1 mm à frente, 4,4 mm abaixo |
| altura do eixo do ombro | 53,9 mm acima da placa |

As suas medidas de régua batem todas: l1 50 contra 53,9; os 130 mm do
cotovelo à ponta do dedo contra 130,2 do CAD; e os "3 a 4 cm" contra os
28,2 mm entre os furos de fixação do SG90 — que é mesmo o que a sua régua
pegou, e não um elo.

O `l2 = 120` que estava no config era estimativa de foto e estava errado
por 40 mm.

---

## Parte 1 — montagem (faz uma vez)

Prenda uma folha de papel em pé (parede, caixa, livro grosso) ao lado do
braço, rente a ele, com a borda de baixo paralela à mesa.

Em cada pose, marque a lápis onde está cada junta. Depois tire o papel e
meça sentado.

| nome | onde é |
|---|---|
| **ombro** | o eixo que sai do servo 1 |
| **cotovelo** | o parafuso lá no alto, no bico do "Λ" |
| **punho** | o parafuso onde o bloco da garra prende no antebraço |

---

## Parte 2 — ombro

`SPEED 20`. Mão no `STOP`.

1. `SET 1 110` → marque o **ombro** (O) e o **cotovelo** (C1).
2. `SET 1 170` → marque o **cotovelo** de novo (C2).

| medida | mm **pra cima** | mm **pra frente** |
|---|---|---|
| de O até C1 | | |
| de O até C2 | | |

- **pra cima** = na direção do teto; se for pra baixo, sinal de menos.
- **pra frente** = na direção da área de coleta; se for pra trás, menos.

**Confere sozinho:** `pra cima² + pra frente²` tem que dar **80²** = 6400.
Ou seja, a diagonal entre as duas marcas tem que dar 80 mm. Se não der, a
marcação escorregou e vale refazer. Isso é a vantagem de o CAD já saber o
comprimento: a própria medida se audita.

---

## Parte 3 — antebraço

Mesma coisa, agora entre o **cotovelo** e o **punho**. Com `SET 1 110`:

1. `SET 2 100` → marque cotovelo e punho.
2. `SET 2 170` → marque cotovelo e punho.
3. Agora `SET 1 170`, mantendo `SET 2 170` → marque de novo.

| medida | mm **pra cima** | mm **pra frente** |
|---|---|---|
| cotovelo → punho, ombro 110 / cotovelo 100 | | |
| cotovelo → punho, ombro 110 / cotovelo 170 | | |
| cotovelo → punho, ombro 170 / cotovelo 170 | | |

Mesma conferência: a diagonal tem que dar **80 mm**.

A linha 3 é de graça e testa o acoplamento. Se der igual à linha 2, o
servo 2 manda no ângulo do antebraço **com o chão** — que é o que o CAD
indica e o que o código assume. Se der diferente, eu troco uma linha.

> Cuidado com ombro alto junto de cotovelo baixo — foi assim que bateu
> (`SET 1 130` + `SET 2 45`). As combinações acima têm folga, mas chegue
> nelas em passos de 20°.

---

## Parte 4 — base (um número só)

`SET 0 <valor>` até o braço apontar reto para a área de coleta,
perpendicular à borda da frente da placa. Anote o valor.

---

## Parte 5 — limite acoplado (o seu teste)

Para cada posição do ombro, o **menor servo 2 que ainda não encosta**. Com
`DETACH`, à mão, antes de deixar o servo tentar.

| servo 1 | menor servo 2 seguro | onde encostou |
|---|---|---|
| 100 | | |
| 130 | | |
| 160 | | |
| 180 | | |

"Onde encostou" é uma palavra: *no servo 1*, *na placa*, *na base de
coleta*. Cada caso pede código diferente:

- bateu **no próprio braço** → vira um limite na diferença entre os dois
  servos;
- bateu **numa peça fixa** → o limite 70 que você já achou basta;
- bateu **na área de coleta** → depende de altura, e o certo é trava de
  piso, não limite de junta.

---

## O que mudou no alcance

Com os números certos o braço é **menor** do que o config supunha:

| | antes (chute) | agora (CAD) |
|---|---|---|
| alcance máximo | 250 mm | **218 mm** |
| alcance na altura da mesa | — | **210 mm** |

Então, quando o offset sair, a primeira coisa é rodar o `mapa_alcance.py`
e conferir se as placas de coleta cabem em 210 mm do eixo da base. Se não
couberem, é só aproximar as placas — não tem jeito de esticar o braço.

---

## Apêndice — por que os números do config ficaram estranhos

`l1 = 49,5` e `offset_radial = 57,8` não são medidas diretas de nada. São
dobras.

A garra do MeArm **não gira com o antebraço**: o paralelogramo mantém ela
nivelada. Então o trecho punho → ponto de preensão é um vetor **constante**
no mundo, não um terceiro elo que gira. E constante soma direto nas
constantes do modelo:

```
offset_radial = 14,7 (base→ombro) + 43,1 (punho→preensão) = 57,8
l1            = 53,9 (altura do ombro) − 4,4 (queda da garra) = 49,5
l2 = l3 = 80
```

Com essa dobra o modelo de dois elos que já existe fica exato — conferi
contra a pose do CAD e deu diferença de 0,08 mm. Nenhuma linha de código
precisou mudar.

Serve pro texto do TCC, aliás: é um caso em que entender o mecanismo
(paralelogramo) simplificou o modelo em vez de complicar.

**Uma coisa ainda não modelada:** o plano do braço passa 4,0 mm ao lado do
eixo de giro da base. Isso joga a garra uns 4 mm pro lado, sempre no mesmo
sentido. É da ordem da tolerância da peça; dá pra corrigir depois, se o
erro aparecer nos testes.
