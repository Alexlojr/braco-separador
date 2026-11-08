#!/usr/bin/env python3
"""
Calibra a homografia câmera -> mesa.

Faça isso UMA VEZ depois de fixar câmera, mesa e base do braço — e refaça
sempre que qualquer um dos três se mexer.

Modo automático (o normal): a área de coleta é um retângulo, e a câmera acha
os cantos dela sozinha por detecção de bordas. Você só informa o tamanho da
placa e onde ela está em relação à base — quatro números que não mudam
enquanto a maquete não for remontada.

    python calibrar_camera.py --largura 300 --altura 200 --x0 -150 --y0 60

Convenção da mesa: origem no eixo da base, +X para a direita, +Y para a
frente. `x0`/`y0` são o canto da placa mais PRÓXIMO da base e mais à
ESQUERDA.

Modo manual (quando a detecção não pega a placa, por contraste ruim ou por a
placa estar parcialmente fora do quadro):

    python calibrar_camera.py --clique    # clica em 4 pontos na imagem
    python calibrar_camera.py --manual    # digita os pixels na mão


SOBRE O ERRO RESIDUAL
---------------------
Com 4 pontos, o residual sobre os próprios pontos de calibração é sempre ~0:
são 8 equações para as 8 incógnitas da matriz, então ela passa exatamente por
eles, estejam certos ou errados. Esse número não valida nada.

A validação de verdade é `--conferir`, que pede pontos medidos com régua que
NÃO entraram no cálculo e relata o erro neles. É esse o número que vale citar
como erro de calibração.

O arquivo gerado (config/homografia.json) é lido pela aba de coleta.
"""

import argparse
import sys
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(BASE_DIR))

from src.vision.area_coleta import (  # noqa: E402
    AreaNaoEncontrada,
    avaliar_com_pontos,
    cantos_em_mm,
    desenhar_previa,
    detectar_area,
)
from src.vision.fonte import aplicar_rotacao  # noqa: E402
from src.vision.homografia import Homografia  # noqa: E402

SAIDA = BASE_DIR / "config" / "homografia.json"
PREVIA = BASE_DIR / "config" / "previa_calibracao.png"


def capturar_frame():
    """Um frame da Picamera2; cai para webcam USB se a Picamera2 não existir."""
    try:
        from picamera2 import Picamera2

        cam = Picamera2()
        cam.configure(
            cam.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)})
        )
        cam.start()
        import time

        time.sleep(2)  # deixa o auto-exposure estabilizar antes de capturar
        frame = cam.capture_array()
        cam.stop()
        return aplicar_rotacao(frame)
    except ImportError:
        import cv2

        cap = cv2.VideoCapture(0)
        ok, frame = cap.read()
        cap.release()
        if not ok:
            raise RuntimeError("não consegui capturar frame de nenhuma câmera")
        return aplicar_rotacao(frame)


def pedir_mm(n=4):
    print(f"\nDigite as coordenadas REAIS dos {n} pontos, em mm, relativas à base.")
    print("Formato: x,y   (ex: -100,40)\n")
    pontos = []
    for i in range(n):
        while True:
            try:
                bruto = input(f"  ponto {i + 1} (x,y em mm): ").strip()
                x, y = (float(v) for v in bruto.split(","))
                pontos.append((x, y))
                break
            except (ValueError, TypeError):
                print("    formato inválido, use x,y — ex: -100,40")
    return pontos


def coletar_por_clique(frame):
    import cv2

    cliques = []
    janela = "Clique nos 4 pontos de referencia (ESC cancela)"

    def ao_clicar(evento, x, y, flags, param):
        if evento == cv2.EVENT_LBUTTONDOWN and len(cliques) < 4:
            cliques.append((x, y))
            print(f"  ponto {len(cliques)}: pixel ({x}, {y})")

    cv2.namedWindow(janela)
    cv2.setMouseCallback(janela, ao_clicar)
    print("\nClique nos 4 pontos de referência, na ordem em que vai medi-los.")

    while True:
        vis = frame.copy()
        for i, (cx, cy) in enumerate(cliques, 1):
            cv2.circle(vis, (cx, cy), 6, (0, 0, 255), -1)
            cv2.putText(vis, str(i), (cx + 10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.imshow(janela, vis)

        tecla = cv2.waitKey(20) & 0xFF
        if tecla == 27:
            cv2.destroyAllWindows()
            return None
        if len(cliques) == 4:
            cv2.imshow(janela, vis)
            cv2.waitKey(400)
            break

    cv2.destroyAllWindows()
    return cliques


def coletar_manual():
    print("\nDigite os pixels dos 4 pontos, formato u,v")
    pontos = []
    for i in range(4):
        while True:
            try:
                bruto = input(f"  ponto {i + 1} (u,v em pixels): ").strip()
                u, v = (float(x) for x in bruto.split(","))
                pontos.append((u, v))
                break
            except (ValueError, TypeError):
                print("    formato inválido, use u,v — ex: 320,240")
    return pontos


def pedir_pontos_conferencia():
    """
    Pontos medidos com régua que NÃO entram no cálculo da matriz.

    São eles que dão um erro residual com significado. Com 4 pontos de
    calibração, a matriz passa exatamente por eles por construção, então o
    residual lá é sempre zero e não distingue calibração boa de ruim.
    """
    print("\nPontos de conferência (Enter vazio encerra).")
    print("Para cada um: o pixel na imagem e a medida de régua em mm.")
    pixels, milimetros = [], []
    while True:
        bruto = input(f"  ponto {len(pixels) + 1} — u,v,x,y: ").strip()
        if not bruto:
            break
        try:
            u, v, x, y = (float(t) for t in bruto.split(","))
        except (ValueError, TypeError):
            print("    formato inválido, use u,v,x,y — ex: 320,240,-10,150")
            continue
        pixels.append((u, v))
        milimetros.append((x, y))
    return pixels, milimetros


def main():
    ap = argparse.ArgumentParser(description="Calibra a homografia câmera->mesa")
    ap.add_argument("--largura", type=float, help="largura da área de coleta, em mm")
    ap.add_argument("--altura", type=float, help="profundidade da área de coleta, em mm")
    ap.add_argument("--x0", type=float, help="X do canto perto-esquerda, em mm")
    ap.add_argument("--y0", type=float, help="Y do canto perto-esquerda, em mm")
    ap.add_argument("--rotacao", type=int, default=0, choices=[0, 1, 2, 3],
                    help="gira a correspondência canto-imagem / canto-mesa em passos de 90°")
    ap.add_argument("--clique", action="store_true", help="clicar nos 4 pontos em vez de detectar")
    ap.add_argument("--manual", action="store_true", help="digitar pixels em vez de clicar")
    ap.add_argument("--conferir", action="store_true",
                    help="pedir pontos de conferência e relatar o erro neles")
    ap.add_argument("--saida", default=str(SAIDA), help="onde salvar o JSON")
    args = ap.parse_args()

    print("=" * 62)
    print("CALIBRAÇÃO DA HOMOGRAFIA CÂMERA -> MESA")
    print("=" * 62)

    automatico = not (args.clique or args.manual)
    if automatico and None in (args.largura, args.altura, args.x0, args.y0):
        print("\nO modo automático precisa das dimensões da área de coleta:")
        print("  --largura <mm> --altura <mm> --x0 <mm> --y0 <mm>")
        print("\nMeça a placa com régua e o canto dela mais próximo da base,")
        print("relativo ao eixo da base (+X direita, +Y frente). São quatro")
        print("números, e só mudam se a maquete for remontada.")
        print("\nSem câmera ou sem contraste, use --clique ou --manual.")
        return 1

    frame = None
    if not args.manual:
        print("\nCapturando frame da câmera...")
        try:
            frame = capturar_frame()
        except Exception as e:
            print(f"\nNão consegui usar a câmera ({e}).")
            print("Rode com --manual para digitar os pixels na mão.")
            return 1

    if automatico:
        try:
            cantos = detectar_area(frame)
        except AreaNaoEncontrada as e:
            print(f"\nNão achei a área de coleta: {e}")
            print("Use --clique para marcar os cantos na mão.")
            return 1
        pixels = [(float(u), float(v)) for u, v in cantos]
        milimetros = cantos_em_mm(args.largura, args.altura, args.x0, args.y0, args.rotacao)
        print("\nÁrea detectada:")
        for i, ((u, v), (xm, ym)) in enumerate(zip(pixels, milimetros), start=1):
            print(f"  canto {i}: pixel ({u:6.0f}, {v:6.0f})  ->  ({xm:7.1f}, {ym:7.1f}) mm")
    elif args.clique:
        pixels = coletar_por_clique(frame)
        if pixels is None:
            print("\nCancelado.")
            return 1
        milimetros = pedir_mm(len(pixels))
    else:
        pixels = coletar_manual()
        milimetros = pedir_mm(len(pixels))

    try:
        h = Homografia.de_pontos(pixels, milimetros, meta={"pixels": pixels, "mm": milimetros})
    except Exception as e:
        print(f"\nFalha ao calcular a homografia: {e}")
        return 1

    if automatico and frame is not None:
        import cv2

        PREVIA.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(PREVIA), desenhar_previa(frame, pixels, milimetros))
        print(f"\nPrévia salva em: {PREVIA}")
        print("  ABRA ESSA IMAGEM antes de confiar na calibração. Confira que o")
        print("  canto marcado como estando perto da base é mesmo o perto da base.")
        print("  Se a correspondência estiver girada, rode de novo com --rotacao 1")
        print("  (ou 2, ou 3). Uma rotação errada NÃO aparece em número nenhum:")
        print("  a matriz fica exata do mesmo jeito e o braço vai para o lugar errado.")

    erro = h.meta["erro_residual_mm"]
    print(f"\nResidual nos próprios pontos de calibração: {erro} mm")
    if len(pixels) == 4:
        print("  Esse número é ~0 por construção — 4 pontos para 8 incógnitas —")
        print("  e não valida a calibração. Use --conferir para o erro de verdade.")
    elif erro > 5:
        print("  ATENÇÃO: erro alto. Vale refazer antes de usar para valer.")

    h.salvar(args.saida)
    print(f"\nSalvo em: {args.saida}")

    if args.conferir:
        pix_c, mm_c = pedir_pontos_conferencia()
        if pix_c:
            medio, maximo, cada = avaliar_com_pontos(h, pix_c, mm_c)
            print("\nErro nos pontos de conferência:")
            for (u, v), (xm, ym), e in zip(pix_c, mm_c, cada):
                x, y = h.pixel_para_mm(u, v)
                print(f"  pixel({u:.0f},{v:.0f}) -> ({x:7.1f},{y:7.1f}) mm | "
                      f"régua ({xm},{ym}) | erro {e:5.1f} mm")
            print(f"\n  médio {medio:.1f} mm   máximo {maximo:.1f} mm")
            print("  " + ("Dentro do critério de 5 mm." if maximo <= 5
                          else "ACIMA do critério de 5 mm. Vale refazer."))
            h.meta["erro_conferencia_mm"] = {"medio": medio, "maximo": maximo}
            h.salvar(args.saida)
        else:
            print("\nNenhum ponto de conferência informado.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
