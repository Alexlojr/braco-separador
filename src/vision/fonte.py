"""
Orientação da câmera, num lugar só.

A câmera ficou montada de cabeça para baixo. Isso dá para resolver de dois
jeitos, e eles não são equivalentes:

1. Só na homografia (`calibrar_camera.py --rotacao 2`). A matriz absorve o
   giro sem esforço — uma rotação de 180° é uma transformação projetiva como
   qualquer outra, e as coordenadas saem certas. Mas a IMAGEM continua de
   cabeça para baixo em todo lugar: no painel, nas caixas desenhadas pela
   detecção, na prévia da calibração. E o YOLO passa a ver os objetos
   invertidos em relação a como foram anotados.

2. Girar o quadro na origem, que é o que este módulo faz. Todo mundo abaixo
   vê a imagem em pé, a homografia é calibrada normalmente com `--rotacao 0`,
   e não sobra nenhum lugar onde alguém precise lembrar da inversão.

O segundo é melhor exatamente porque não precisa ser lembrado. Um ajuste que
mora em quatro lugares é um ajuste que vai ser esquecido em um deles.

Custo: um `cv2.rotate` por quadro, que num 640x480 é uma cópia de memória e
não aparece ao lado da inferência. Na Picamera2 dá para pedir o giro ao ISP
(`libcamera.Transform(hflip=1, vflip=1)`) e sair de graça, mas aí o caminho
da webcam USB precisaria de outro tratamento — não vale a divergência.

ATENÇÃO: girar o quadro invalida uma homografia já calibrada, porque os
pixels mudam de lugar. Mexeu aqui, recalibre.
"""

from __future__ import annotations

import json
from pathlib import Path

import cv2

_CONFIG_PADRAO = Path(__file__).resolve().parents[2] / "config" / "braco.json"

_MAPA = {
    90: cv2.ROTATE_90_CLOCKWISE,
    180: cv2.ROTATE_180,
    270: cv2.ROTATE_90_COUNTERCLOCKWISE,
}

_cache: dict[str, int] = {}


def rotacao_configurada(caminho=None) -> int:
    """
    Quantos graus girar o quadro, lido de `camera.rotacao_graus` na config.

    Devolve 0 se o arquivo não existir ou não trouxer o campo: câmera em pé
    é o caso normal, e não é para o pipeline quebrar por falta de uma chave
    opcional.
    """
    caminho = Path(caminho) if caminho else _CONFIG_PADRAO
    chave = str(caminho)
    if chave in _cache:
        return _cache[chave]

    graus = 0
    try:
        dados = json.loads(caminho.read_text(encoding="utf-8"))
        graus = int(dados.get("camera", {}).get("rotacao_graus", 0)) % 360
    except (OSError, ValueError, TypeError):
        graus = 0

    if graus not in (0, 90, 180, 270):
        raise ValueError(
            f"camera.rotacao_graus = {graus}; só 0, 90, 180 ou 270 são aceitos. "
            "Giro arbitrário cortaria o quadro e mudaria a resolução."
        )

    _cache[chave] = graus
    return graus


def aplicar_rotacao(frame, graus: int | None = None):
    """Gira o quadro. `graus=None` usa o que estiver na config."""
    if frame is None:
        return None
    if graus is None:
        graus = rotacao_configurada()
    if graus == 0:
        return frame
    return cv2.rotate(frame, _MAPA[graus])


def limpar_cache():
    """Para os testes, e para quem edita a config com o processo de pé."""
    _cache.clear()
