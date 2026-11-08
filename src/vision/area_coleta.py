"""
Acha a área de coleta sozinho, por detecção de bordas.

Para que serve: a calibração da homografia pedia clicar em 4 pontos e digitar
8 medidas de régua. Isso é trabalhoso de refazer, e a calibração precisa ser
refeita toda vez que a câmera, a área de coleta ou a base do braço se mexem.

A área de coleta é um retângulo de dimensões conhecidas. Então não há motivo
para clicar nos cantos: a câmera consegue achá-los. O que sobra para o humano
é dizer o TAMANHO da placa e ONDE ela está em relação à base — quatro números
que não mudam enquanto a maquete não for remontada.

    cantos_px = detectar_area(frame)          # 4 cantos, em pixel
    cantos_mm = cantos_em_mm(300, 200, -150, 60)   # os mesmos 4, em mm

E a homografia sai desses dois conjuntos.


SOBRE O ERRO RESIDUAL COM 4 PONTOS
----------------------------------
Uma homografia tem 8 graus de liberdade, e cada par de pontos fornece 2
equações. Com 4 pares o sistema é exatamente determinado: a matriz passa
pelos 4 pontos SEM ERRO, por construção.

Ou seja: com 4 pontos o erro residual de reprojeção é sempre ~0, e não prova
nada sobre a qualidade da calibração. Se os 4 cantos forem detectados no
lugar errado, o residual continua 0 e o braço aponta errado em silêncio.

Por isso `avaliar_com_pontos()` existe separado. Ela recebe pontos de
CONFERÊNCIA — medidos com régua, que não entraram no cálculo — e devolve o
erro de verdade. É esse número que vale como validação, e é esse que deve ser
citado como erro residual em qualquer relato do sistema.
"""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np


class AreaNaoEncontrada(Exception):
    pass


def _ordenar_cantos(pontos: np.ndarray) -> np.ndarray:
    """
    Põe 4 cantos na ordem: superior-esquerdo, superior-direito,
    inferior-direito, inferior-esquerdo — em coordenadas de IMAGEM.

    O truque é que, num quadrilátero convexo, a soma u+v é mínima no canto
    superior-esquerdo e máxima no inferior-direito, enquanto a diferença u-v
    é máxima no superior-direito e mínima no inferior-esquerdo. Vale para
    qualquer perspectiva que não chegue a girar a placa além de 45°.
    """
    pontos = np.asarray(pontos, dtype=np.float64).reshape(4, 2)
    soma = pontos.sum(axis=1)
    dif = pontos[:, 0] - pontos[:, 1]
    return np.array([
        pontos[np.argmin(soma)],   # superior-esquerdo
        pontos[np.argmax(dif)],    # superior-direito
        pontos[np.argmax(soma)],   # inferior-direito
        pontos[np.argmin(dif)],    # inferior-esquerdo
    ])


def detectar_quadrilateros(frame, area_minima_frac=0.02, max_resultados=10):
    """
    Todos os quadriláteros convexos da imagem, do maior para o menor.

    Devolve uma lista de arrays (4, 2) já ordenados por `_ordenar_cantos`.
    Lista vazia se não achou nada — quem chama decide se isso é erro.
    """
    if frame.ndim == 3:
        cinza = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    else:
        cinza = frame
    cinza = cv2.GaussianBlur(cinza, (5, 5), 0)

    # Limiares do Canny a partir da mediana da imagem, em vez de fixos: a
    # iluminação da bancada muda entre o dia e a noite, e limiar fixo que
    # funciona numa condição some na outra.
    mediana = float(np.median(cinza))
    baixo = int(max(0, 0.66 * mediana))
    alto = int(min(255, 1.33 * mediana))
    bordas = cv2.Canny(cinza, baixo, alto)

    # Fecha falhas na borda: placa de acrílico reflete, e o reflexo corta a
    # linha em pedaços que o findContours não junta sozinho.
    bordas = cv2.dilate(bordas, np.ones((3, 3), np.uint8), iterations=1)

    contornos, _ = cv2.findContours(bordas, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    area_imagem = cinza.shape[0] * cinza.shape[1]
    area_minima = area_minima_frac * area_imagem

    achados = []
    for c in sorted(contornos, key=cv2.contourArea, reverse=True):
        if cv2.contourArea(c) < area_minima:
            break
        aprox = cv2.approxPolyDP(c, 0.02 * cv2.arcLength(c, True), True)
        if len(aprox) == 4 and cv2.isContourConvex(aprox):
            achados.append(_ordenar_cantos(aprox))
            if len(achados) >= max_resultados:
                break
    return achados


def diagnostico(frame, destino) -> str:
    """
    Salva o que a câmera viu e o que o detector enxergou dela.

    Falha de detecção sem imagem é adivinhação: não dá para saber se o
    problema é enquadramento, contraste, foco ou luz. As duas imagens
    respondem isso de olho — a de bordas mostra exatamente onde o algoritmo
    acha que existe aresta, e se a placa não aparecer lá, não há o que
    ajustar em parâmetro.
    """
    destino = Path(destino)
    destino.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(destino), frame)

    cinza = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    cinza = cv2.GaussianBlur(cinza, (5, 5), 0)
    mediana = float(np.median(cinza))
    bordas = cv2.Canny(cinza, int(max(0, 0.66 * mediana)), int(min(255, 1.33 * mediana)))
    bordas = cv2.dilate(bordas, np.ones((3, 3), np.uint8), iterations=1)
    cv2.imwrite(str(destino.with_name(destino.stem + "_bordas.png")), bordas)
    return str(destino)


def detectar_area(frame, area_minima_frac=0.02):
    """O maior quadrilátero da imagem. Levanta AreaNaoEncontrada se não houver."""
    achados = detectar_quadrilateros(frame, area_minima_frac, max_resultados=1)
    if not achados:
        raise AreaNaoEncontrada(
            "nenhum quadrilátero convexo acima do tamanho mínimo. "
            "Confira o enquadramento e o contraste entre a placa e o fundo."
        )
    return achados[0]


def cantos_em_mm(largura_mm, altura_mm, x0_mm, y0_mm, rotacao=0):
    """
    Os 4 cantos da placa em coordenadas da mesa, na mesma ordem da imagem.

    A placa é um retângulo de `largura_mm` x `altura_mm`, com o canto mais
    PRÓXIMO da base e mais à ESQUERDA em (x0_mm, y0_mm). Convenção da mesa:
    origem no eixo da base, +X para a direita, +Y para a frente.

    `rotacao` gira a correspondência entre canto da imagem e canto da mesa,
    em passos de 90°. Serve para quando a câmera está montada de lado ou de
    cabeça para baixo. Com a câmera olhando de cima, base embaixo do quadro,
    o valor é 0.

    Atenção: a homografia fica exata para QUALQUER rotação, porque são 4
    pontos para 8 incógnitas. A rotação errada não aparece no residual; só
    aparece no braço indo para o lugar errado. Confira olhando a prévia.
    """
    x1, y1 = x0_mm + largura_mm, y0_mm + altura_mm
    # Ordem da imagem: sup-esq, sup-dir, inf-dir, inf-esq.
    # Na mesa, "superior" na imagem é o lado LONGE da base, ou seja +Y.
    cantos = [
        (x0_mm, y1),   # superior-esquerdo  -> longe, esquerda
        (x1, y1),      # superior-direito   -> longe, direita
        (x1, y0_mm),   # inferior-direito   -> perto, direita
        (x0_mm, y0_mm),  # inferior-esquerdo -> perto, esquerda
    ]
    r = rotacao % 4
    return cantos[r:] + cantos[:r]


def avaliar_com_pontos(homografia, pixels, milimetros):
    """
    Erro de reprojeção em pontos que NÃO entraram no cálculo da matriz.

    Devolve (erro_medio_mm, erro_maximo_mm, [erro de cada ponto]).

    É esta a medida que vale como validação da calibração. O residual sobre
    os próprios 4 cantos é estruturalmente zero — ver o cabeçalho do módulo.
    """
    erros = []
    for (u, v), (xm, ym) in zip(pixels, milimetros):
        x, y = homografia.pixel_para_mm(u, v)
        erros.append(float(np.hypot(x - xm, y - ym)))
    if not erros:
        return 0.0, 0.0, []
    return sum(erros) / len(erros), max(erros), erros


def desenhar_previa(frame, cantos_px, cantos_mm):
    """
    Cópia do frame com o quadrilátero e a coordenada de cada canto escrita.

    Existe porque a rotação errada não aparece em número nenhum. Olhar a
    imagem e conferir que o canto marcado como perto-esquerda é mesmo o
    perto-esquerda é a única verificação que pega esse erro.
    """
    vis = frame.copy() if frame.ndim == 3 else cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    pts = np.asarray(cantos_px, dtype=np.int32).reshape(-1, 1, 2)
    cv2.polylines(vis, [pts], True, (0, 255, 0), 2)
    for i, ((u, v), (xm, ym)) in enumerate(zip(cantos_px, cantos_mm), start=1):
        u, v = int(u), int(v)
        cv2.circle(vis, (u, v), 6, (0, 0, 255), -1)
        cv2.putText(vis, f"{i}: ({xm:.0f},{ym:.0f})mm", (u + 8, v - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    return vis
