#!/usr/bin/env python3
"""
Detecção automática da área de coleta, sem câmera.

A imagem de teste é sintética: uma placa retangular de dimensões conhecidas,
projetada em perspectiva por uma homografia que o próprio teste escolheu.
Como se sabe de onde a placa veio, dá para cobrar os cantos de volta em pixel
e, depois, cobrar que a homografia estimada reproduza pontos que NÃO entraram
no cálculo.

Esse último ponto é o que importa: com 4 pares o residual sobre os próprios
cantos é zero por construção, e um teste que olhasse só para ele passaria
mesmo com a detecção errada.
"""

import sys
from pathlib import Path

import cv2
import numpy as np

RAIZ = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(RAIZ))

from src.vision.area_coleta import (  # noqa: E402
    AreaNaoEncontrada,
    _ordenar_cantos,
    avaliar_com_pontos,
    cantos_em_mm,
    detectar_area,
    detectar_quadrilateros,
)
from src.vision.homografia import Homografia  # noqa: E402

falhas = []


def checa(condicao, descricao):
    if not condicao:
        falhas.append(descricao)


# --- cena sintética --------------------------------------------------------
# Placa de 300 x 200 mm, com o canto perto-esquerda em (-150, 60) mm.
LARGURA_MM, ALTURA_MM = 300.0, 200.0
X0_MM, Y0_MM = -150.0, 60.0

LARG_PX, ALT_PX = 640, 480

# Cantos em pixel escolhidos à mão, em perspectiva (o lado de cima, que é o
# mais longe da base, aparece mais estreito, como numa câmera inclinada).
CANTOS_PX_REAIS = np.array([
    [180.0, 120.0],   # superior-esquerdo
    [460.0, 120.0],   # superior-direito
    [530.0, 360.0],   # inferior-direito
    [110.0, 360.0],   # inferior-esquerdo
])


def cena():
    """Fundo escuro com a placa clara desenhada nos cantos acima."""
    img = np.full((ALT_PX, LARG_PX, 3), 40, dtype=np.uint8)
    cv2.fillPoly(img, [CANTOS_PX_REAIS.astype(np.int32)], (200, 200, 200))
    # Ruído leve: sem ele o Canny trabalha numa imagem limpa demais para
    # dizer qualquer coisa sobre o comportamento na bancada.
    ruido = np.random.default_rng(0).normal(0, 4, img.shape)
    return np.clip(img.astype(np.float64) + ruido, 0, 255).astype(np.uint8)


img = cena()

# --- 1. acha a placa -------------------------------------------------------
try:
    cantos = detectar_area(img)
    erro_cantos = np.abs(cantos - CANTOS_PX_REAIS).max()
    checa(erro_cantos <= 4.0,
          f"cantos detectados erraram {erro_cantos:.1f} px (tolerancia 4)")
except AreaNaoEncontrada as e:
    cantos = None
    checa(False, f"nao achou a placa: {e}")

# --- 2. ordem dos cantos ---------------------------------------------------
if cantos is not None:
    sup_esq, sup_dir, inf_dir, inf_esq = cantos
    checa(sup_esq[0] < sup_dir[0], "superior-esquerdo deveria estar a esquerda do direito")
    checa(sup_esq[1] < inf_esq[1], "superior-esquerdo deveria estar acima do inferior")
    checa(inf_esq[0] < inf_dir[0], "inferior-esquerdo deveria estar a esquerda do direito")

# --- 3. imagem sem placa nenhuma nao pode inventar uma ---------------------
vazio = np.full((ALT_PX, LARG_PX, 3), 40, dtype=np.uint8)
try:
    detectar_area(vazio)
    checa(False, "imagem vazia deveria levantar AreaNaoEncontrada")
except AreaNaoEncontrada:
    pass

# --- 4. a homografia reproduz pontos que nao entraram no calculo -----------
if cantos is not None:
    mm = cantos_em_mm(LARGURA_MM, ALTURA_MM, X0_MM, Y0_MM)
    h = Homografia.de_pontos([tuple(p) for p in cantos], mm)

    # Residual sobre os proprios cantos: tem que ser ~0 SEMPRE. Isto nao e
    # uma qualidade da calibracao, e uma propriedade de 4 pontos para 8
    # incognitas. O teste cobra isso para documentar o fato.
    checa(h.meta["erro_residual_mm"] < 0.01,
          f"residual nos 4 cantos deveria ser ~0, veio {h.meta['erro_residual_mm']}")

    # A validacao de verdade: pontos de conferencia independentes. Usa-se a
    # homografia EXATA da cena (a que leva mm -> pixel de fato) para gerar
    # pares verdadeiros em posicoes que nao sao canto.
    verdadeira = cv2.getPerspectiveTransform(
        np.array(mm, dtype=np.float32),
        CANTOS_PX_REAIS.astype(np.float32),
    )
    amostras_mm = [
        (X0_MM + 0.5 * LARGURA_MM, Y0_MM + 0.5 * ALTURA_MM),
        (X0_MM + 0.25 * LARGURA_MM, Y0_MM + 0.75 * ALTURA_MM),
        (X0_MM + 0.80 * LARGURA_MM, Y0_MM + 0.20 * ALTURA_MM),
    ]
    amostras_px = []
    for xm, ym in amostras_mm:
        p = verdadeira @ np.array([xm, ym, 1.0])
        amostras_px.append((p[0] / p[2], p[1] / p[2]))

    medio, maximo, _ = avaliar_com_pontos(h, amostras_px, amostras_mm)
    checa(maximo < 5.0,
          f"erro em pontos de conferencia deveria ficar abaixo de 5 mm, veio {maximo:.2f}")

# --- 5. a rotacao muda o mapeamento, e isso NAO aparece no residual --------
if cantos is not None:
    girado = cantos_em_mm(LARGURA_MM, ALTURA_MM, X0_MM, Y0_MM, rotacao=1)
    h_girado = Homografia.de_pontos([tuple(p) for p in cantos], girado)
    checa(h_girado.meta["erro_residual_mm"] < 0.01,
          "o residual nao distingue rotacao errada — se distinguisse, "
          "a premissa do modulo estaria errada")
    # O CENTRO nao serve de sonda: uma permutacao ciclica dos 4 cantos mantem
    # o centroide no lugar, entao o centro mapeia igual nas duas. Quem
    # denuncia a rotacao e um canto.
    canto_ok = h.pixel_para_mm(*cantos[0])
    canto_girado = h_girado.pixel_para_mm(*cantos[0])
    checa(np.hypot(canto_ok[0] - canto_girado[0], canto_ok[1] - canto_girado[1]) > 100,
          f"rotacao errada deveria mandar o canto para outro lugar: "
          f"{canto_ok} contra {canto_girado}")

# --- 6. varios quadrilateros saem do maior para o menor --------------------
multi = cena()
cv2.fillPoly(multi, [np.array([[40, 40], [120, 40], [120, 110], [40, 110]])], (210, 210, 210))
quads = detectar_quadrilateros(multi, area_minima_frac=0.005)
if len(quads) >= 2:
    areas = [cv2.contourArea(q.astype(np.float32)) for q in quads]
    checa(areas == sorted(areas, reverse=True), "quadrilateros deveriam vir do maior para o menor")
else:
    checa(False, f"deveria achar pelo menos 2 quadrilateros, achou {len(quads)}")


# --- 7. rotacao do quadro na origem ---------------------------------------
from src.vision.fonte import aplicar_rotacao, rotacao_configurada  # noqa: E402

checa(rotacao_configurada() in (0, 90, 180, 270),
      "rotacao_graus da config deveria ser 0, 90, 180 ou 270")

# Girar duas vezes 180 tem que devolver o original, pixel a pixel. E o 180
# tem que ser mesmo um 180: o canto superior-esquerdo vai para o
# inferior-direito.
girado = aplicar_rotacao(img, 180)
checa(np.array_equal(aplicar_rotacao(girado, 180), img),
      "girar 180 duas vezes deveria devolver o quadro original")
checa(girado.shape == img.shape, "180 nao pode mudar a resolucao")
checa(np.array_equal(girado[0, 0], img[-1, -1]),
      "no giro de 180 o canto superior-esquerdo vira o inferior-direito")
checa(aplicar_rotacao(img, 0) is img, "rotacao 0 deveria devolver o mesmo objeto")
checa(aplicar_rotacao(img, 90).shape[:2] == img.shape[:2][::-1],
      "90 graus deveria trocar largura por altura")

# A deteccao tem que achar a placa no quadro girado tambem — e nos cantos
# espelhados, que e o que prova que girar na origem resolve de verdade.
cantos_girados = detectar_area(girado)
esperado = _ordenar_cantos(np.array([
    [LARG_PX - 1 - u, ALT_PX - 1 - v] for u, v in CANTOS_PX_REAIS
]))
checa(np.abs(cantos_girados - esperado).max() <= 4.0,
      "no quadro girado os cantos deveriam ser os espelhados")


for f in falhas:
    print(f"  FALHA: {f}")
print("OK — deteccao da area de coleta" if not falhas else f"\n{len(falhas)} falha(s)")
raise SystemExit(1 if falhas else 0)
