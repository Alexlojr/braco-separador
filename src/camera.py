"""
Visualizador de câmera sem Streamlit — janela do OpenCV, `q` para sair.

Serve para conferir enquadramento, foco e iluminação direto no monitor do Pi,
sem subir a dashboard nem abrir navegador. É o jeito mais rápido de responder
"a câmera está enxergando a área de coleta inteira?" na hora de fixar o stand.

O critério de detecção vem de src/vision/deteccao.py, o mesmo que a dashboard
e o coletar.py usam — antes esta lógica era uma cópia, e as cópias divergiam.

    python src/camera.py
"""

import sys
from pathlib import Path

import cv2
from picamera2 import Picamera2
from ultralytics import YOLO

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from src.vision.fonte import aplicar_rotacao  # noqa: E402
from src.vision.deteccao import (  # noqa: E402
    INTERVALO_DETECCAO,
    carregar_preferencias,
    desenhar,
    caminho_modelo,
    melhor_deteccao,
    resolver_classe,
)

CAMINHO_CONFIG = Path(__file__).resolve().parent.parent / "config" / "braco.json"
CAMINHO_MODELO = caminho_modelo(CAMINHO_CONFIG, CAMINHO_CONFIG.parent.parent)


def main() -> int:
    modelo = YOLO(str(CAMINHO_MODELO))
    pedido, confianca = carregar_preferencias(CAMINHO_CONFIG)
    alvo = resolver_classe(pedido, modelo.names)
    print(f"Procurando {modelo.names[alvo]} ({alvo}), confiança >= {confianca:.2f}")

    picam2 = Picamera2()
    picam2.configure(
        picam2.create_preview_configuration(
            main={"format": "RGB888", "size": (640, 480)}
        )
    )
    picam2.start()

    contador = 0
    ultima = None

    try:
        while True:
            frame = aplicar_rotacao(picam2.capture_array())
            contador += 1

            if contador % INTERVALO_DETECCAO == 0:
                resultado = modelo(frame, classes=[alvo], verbose=False)[0]
                ultima = melhor_deteccao(resultado, modelo.names, confianca)

            if ultima is not None:
                desenhar(frame, ultima)

            cv2.imshow("Camera", frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    finally:
        picam2.stop()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
