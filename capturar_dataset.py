#!/usr/bin/env python3
"""
Captura e rotula as imagens de treino pela MESMA camera que opera o sistema.

COMO SE USA

    python capturar_dataset.py --classe mouse --autorotular
    python capturar_dataset.py --classe peca

Abre uma janela com a camera ao vivo. Voce posiciona o objeto olhando a
janela e aperta ESPACO:

  - com --autorotular, o modelo base acha o objeto e grava a caixa sozinho.
    Serve para o mouse, que e classe do COCO. Se ele nao achar, a imagem
    congela e voce desenha a caixa;
  - sem --autorotular, a imagem congela na hora e voce ARRASTA uma caixa em
    volta do objeto com o mouse do computador. ENTER confirma, C cancela.

Foto e rotulo saem juntos, no formato e na pasta que o treino espera. Nao ha
etapa de rotular depois.

  ESPACO  captura        D  apaga a ultima        Q  sai

Depois, a validacao, em POSICOES DIFERENTES das do treino:

    python capturar_dataset.py --classe mouse --conjunto val --autorotular
    python capturar_dataset.py --classe peca  --conjunto val

E por fim, na maquina onde o treino vai rodar:

    python capturar_dataset.py --escrever-yaml

SEM TELA NO PI (so SSH)

A janela precisa de area de trabalho: monitor ou compartilhamento de tela do
Pi Connect. Sem isso o script cai sozinho no modo terminal: ENTER captura, e
a camera ao vivo aparece no PAINEL (app.py), no navegador. Nesse modo a peca
fica sem rotulo, e voce rotula depois, em qualquer maquina com tela - o PC
serve, so precisa de `pip install opencv-python`:

    python capturar_dataset.py --rotular

que passa por todas as fotos sem rotulo, uma a uma, para desenhar a caixa.

POR QUE ASSIM

Pela camera do Pi, e nao com celular: o detector so vai ver esta camera,
nesta altura, com esta lente e esta luz. Foto em perspectiva arbitraria e um
dominio que nao existe na operacao.

O que deve variar e a POSICAO na placa (centro, bordas, cantos, perto do
limite de alcance) e a ORIENTACAO (em pe, deitada, tombada).

Treino e validacao sao separados POR POSICAO, escolhida na linha de comando.
Quadros da mesma cena sao quase identicos, e uma divisao aleatoria jogaria
copias do mesmo quadro nos dois lados: a validacao mediria memorizacao e
devolveria nota alta que nao significa nada.

FORMATO: YOLO. Um .txt por imagem, mesmo nome, em labels/. Uma linha por
objeto: `classe cx cy w h`, normalizado de 0 a 1.

A imagem e gravada com a MESMA convencao de canais que a inferencia consome,
para o modelo treinar no que vai ver. E e o quadro CRU: nada do que aparece
desenhado na janela vai para o arquivo.
"""

import argparse
import sys
import threading
import time
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(BASE_DIR))

RAIZ = BASE_DIR / "dataset"
CAMINHO_MODELO_BASE = BASE_DIR / "models" / "yolov8n.pt"

# Ordem das classes no data.yaml. O indice e o numero gravado no rotulo:
# acrescentar no fim e seguro, reordenar invalida todo rotulo ja escrito.
CLASSES = ["peca", "mouse"]

CONJUNTOS = {"train": "train", "treino": "train", "val": "val",
             "validacao": "val", "validação": "val"}

JANELA = "captura - ESPACO salva, D apaga, Q sai"

TECLAS_SALVAR = (32, 13, 10)      # espaco; Enter no Windows e no Linux
TECLAS_SAIR = (ord("q"), ord("Q"), 27)
TECLAS_APAGAR = (ord("d"), ord("D"))


# --------------------------------------------------------------- sem cv2

def caixa_para_yolo(indice, x1, y1, x2, y2, largura, altura):
    """A linha do rotulo YOLO, com a caixa presa dentro da imagem."""
    x1, x2 = sorted((max(0.0, min(x1, largura)), max(0.0, min(x2, largura))))
    y1, y2 = sorted((max(0.0, min(y1, altura)), max(0.0, min(y2, altura))))
    cx = (x1 + x2) / 2 / largura
    cy = (y1 + y2) / 2 / altura
    w = (x2 - x1) / largura
    h = (y2 - y1) / altura
    return f"{indice} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}\n"


def indice_pelo_nome(arquivo):
    """`peca_train_007.jpg` -> 0. O prefixo e a classe, por construcao."""
    prefixo = Path(arquivo).stem.split("_")[0]
    return CLASSES.index(prefixo) if prefixo in CLASSES else None


def pastas(conjunto):
    imgs = RAIZ / "images" / conjunto
    lbls = RAIZ / "labels" / conjunto
    imgs.mkdir(parents=True, exist_ok=True)
    lbls.mkdir(parents=True, exist_ok=True)
    return imgs, lbls


def proximo_nome(imgs, classe, conjunto):
    """O primeiro numero livre, para nao sobrescrever apos um 'd'."""
    n = 1
    while (imgs / f"{classe}_{conjunto}_{n:03d}.jpg").exists():
        n += 1
    return f"{classe}_{conjunto}_{n:03d}"


def sem_rotulo(conjunto, classe=None):
    imgs, lbls = pastas(conjunto)
    padrao = f"{classe}_*.jpg" if classe else "*.jpg"
    return sorted(p for p in imgs.glob(padrao)
                  if not (lbls / f"{p.stem}.txt").exists())


def escrever_yaml():
    """data.yaml com caminho absoluto: o Ultralytics resolve relativo ao
    diretorio de datasets dele, nao ao do projeto. Por isso se gera NA
    maquina do treino."""
    RAIZ.mkdir(parents=True, exist_ok=True)
    nomes = "\n".join(f"  {i}: {n}" for i, n in enumerate(CLASSES))
    texto = (f"# Gerado por capturar_dataset.py\n"
             f"path: {RAIZ.as_posix()}\n"
             f"train: images/train\n"
             f"val: images/val\n\n"
             f"names:\n{nomes}\n")
    destino = RAIZ / "data.yaml"
    destino.write_text(texto, encoding="utf-8")
    print(f"Escrito {destino}\n\n{texto}")

    for conjunto in ("train", "val"):
        imgs, _ = pastas(conjunto)
        por_classe = {c: len(list(imgs.glob(f"{c}_*.jpg"))) for c in CLASSES}
        faltando = len(sem_rotulo(conjunto))
        resumo = ", ".join(f"{c} {n}" for c, n in por_classe.items())
        print(f"  {conjunto}: {resumo}"
              + (f"  — {faltando} SEM ROTULO" if faltando else ""))
    if any(sem_rotulo(c) for c in ("train", "val")):
        print("\nRode `python capturar_dataset.py --rotular` antes de treinar:")
        print("foto sem rotulo entra no treino como quadro VAZIO e ensina o")
        print("modelo a nao detectar nada.")


# --------------------------------------------------------------- com cv2

def texto(cv2, img, linhas, cor=(255, 255, 255)):
    """Faixa escura no topo com texto. Sem acento: a fonte do OpenCV nao tem."""
    altura_linha = 22
    cv2.rectangle(img, (0, 0), (img.shape[1], 8 + altura_linha * len(linhas)),
                  (0, 0, 0), -1)
    for i, linha in enumerate(linhas):
        cv2.putText(img, linha, (8, 20 + i * altura_linha),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, cor, 1, cv2.LINE_AA)


def pedir_caixa(cv2, frame, titulo):
    """Congela o quadro e deixa arrastar a caixa. None se cancelar."""
    guia = frame.copy()
    texto(cv2, guia, [titulo, "ARRASTE a caixa | ENTER confirma | C cancela"],
          (0, 255, 255))
    x, y, w, h = cv2.selectROI(JANELA, guia, showCrosshair=True,
                               fromCenter=False)
    if w < 4 or h < 4:
        return None
    return (float(x), float(y), float(x + w), float(y + h))


def detectar(modelo, coco, frame, confianca):
    """(x1, y1, x2, y2, conf) do melhor objeto da classe, ou None."""
    from src.vision.deteccao import melhor_deteccao
    resultado = modelo(frame, classes=[coco], imgsz=640, conf=confianca,
                       verbose=False)[0]
    det = melhor_deteccao(resultado, modelo.names, confianca)
    if det is None:
        return None
    return (det.x1, det.y1, det.x2, det.y2, det.confianca)


def gravar(cv2, frame, caixa, indice, img_path, lbl_path):
    altura, largura = frame.shape[:2]
    if not cv2.imwrite(str(img_path), frame):
        return False
    lbl_path.write_text(caixa_para_yolo(indice, *caixa, largura, altura),
                        encoding="utf-8")
    return True


def mostrar_confirmacao(cv2, frame, caixa, legenda):
    vis = frame.copy()
    x1, y1, x2, y2 = (int(v) for v in caixa)
    cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
    texto(cv2, vis, [f"SALVO - {legenda}", "D apaga se a caixa ficou errada"],
          (0, 255, 0))
    cv2.imshow(JANELA, vis)
    cv2.waitKey(600)


def janela_disponivel(cv2):
    # Checa o display ANTES de pedir a janela: no Linux sem tela (SSH), o
    # backend Qt do OpenCV costuma abortar o processo inteiro em vez de
    # levantar excecao - o try abaixo nem chegaria a pegar.
    import os
    if sys.platform.startswith("linux") and not (
            os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        return False
    try:
        cv2.namedWindow(JANELA, cv2.WINDOW_AUTOSIZE)
        return True
    except cv2.error:
        return False


# ------------------------------------------------------------------ modos

def capturar_com_janela(cv2, capturar, classe, conjunto, indice,
                        modelo, coco, confianca):
    imgs, lbls = pastas(conjunto)
    gravadas = []
    aviso, aviso_ate = "", 0.0

    while True:
        frame = capturar()
        vis = frame.copy()
        total = len(list(imgs.glob(f"{classe}_*.jpg")))
        linhas = [f"{classe} | {conjunto} | {total} salvas",
                  "ESPACO salva   D apaga a ultima   Q sai"]
        if aviso and time.monotonic() < aviso_ate:
            linhas.append(aviso)
        texto(cv2, vis, linhas)
        cv2.imshow(JANELA, vis)
        tecla = cv2.waitKey(1) & 0xFF

        if tecla in TECLAS_SAIR:
            break

        if tecla in TECLAS_APAGAR:
            if gravadas:
                img, lbl = gravadas.pop()
                img.unlink(missing_ok=True)
                lbl.unlink(missing_ok=True)
                aviso = f"apagado {img.name}"
            else:
                aviso = "nada para apagar nesta sessao"
            aviso_ate = time.monotonic() + 2.0
            print(f"  {aviso}")
            continue

        if tecla not in TECLAS_SALVAR:
            continue

        # O quadro que vai para o arquivo e ESTE, cru, do instante da tecla.
        caixa, legenda = None, ""
        if modelo is not None:
            det = detectar(modelo, coco, frame, confianca)
            if det is not None:
                caixa, legenda = det[:4], f"automatico, confianca {det[4]:.2f}"
            else:
                print("  modelo nao achou — desenhe a caixa")
        if caixa is None:
            caixa = pedir_caixa(cv2, frame, f"{classe}: desenhe a caixa")
            legenda = "manual"
            if caixa is None:
                aviso, aviso_ate = "cancelado - nada salvo", time.monotonic() + 2.0
                print(f"  {aviso}")
                continue

        nome = proximo_nome(imgs, classe, conjunto)
        img_path, lbl_path = imgs / f"{nome}.jpg", lbls / f"{nome}.txt"
        if not gravar(cv2, frame, caixa, indice, img_path, lbl_path):
            print(f"  ERRO: nao consegui gravar {img_path}")
            continue
        gravadas.append((img_path, lbl_path))
        print(f"  {img_path.name}  ({legenda})")
        mostrar_confirmacao(cv2, frame, caixa, legenda)

    cv2.destroyAllWindows()
    return len(gravadas)


def capturar_no_terminal(cv2, capturar, classe, conjunto, indice,
                         modelo, coco, confianca):
    """Sem area de trabalho: ENTER no terminal, camera ao vivo no painel."""
    from src.vision.quadro_publicado import limpar, publicar

    imgs, lbls = pastas(conjunto)
    ultimo = {"frame": None}
    parar = threading.Event()

    # Uma thread so mexe na camera, e publica ~5 quadros por segundo para o
    # painel. A principal fica bloqueada no input() e pega o mais recente.
    def laco():
        while not parar.is_set():
            f = capturar()
            ultimo["frame"] = f
            publicar(f, {"rotulo": f"captura: {classe} ({conjunto})",
                         "coletados": len(list(imgs.glob(f"{classe}_*.jpg")))})
            time.sleep(0.2)

    t = threading.Thread(target=laco, daemon=True)
    t.start()
    print("Sem janela: acompanhe a camera pelo PAINEL (app.py) no navegador.")
    print("ENTER captura | d apaga a ultima | q sai\n")

    gravadas = []
    try:
        while True:
            try:
                tecla = input("> ").strip().lower()
            except EOFError:
                break
            if tecla == "q":
                break
            if tecla == "d":
                if gravadas:
                    img, lbl = gravadas.pop()
                    img.unlink(missing_ok=True)
                    lbl.unlink(missing_ok=True)
                    print(f"  apagado {img.name}")
                continue
            frame = ultimo["frame"]
            if frame is None:
                print("  camera ainda abrindo, tente de novo")
                continue
            frame = frame.copy()
            nome = proximo_nome(imgs, classe, conjunto)
            img_path, lbl_path = imgs / f"{nome}.jpg", lbls / f"{nome}.txt"

            det = detectar(modelo, coco, frame, confianca) if modelo else None
            if det is not None:
                gravar(cv2, frame, det[:4], indice, img_path, lbl_path)
                print(f"  {img_path.name}  (automatico, {det[4]:.2f})")
            else:
                if not cv2.imwrite(str(img_path), frame):
                    print(f"  ERRO: nao consegui gravar {img_path}")
                    continue
                print(f"  {img_path.name}  SEM ROTULO — rotular depois")
            gravadas.append((img_path, lbl_path))
    except KeyboardInterrupt:
        print()
    finally:
        parar.set()
        t.join(timeout=2)
        limpar()
    return len(gravadas)


def rotular_pendentes(cv2):
    """Passa por toda foto sem rotulo, em train e val, para desenhar a caixa."""
    pendentes = sem_rotulo("train") + sem_rotulo("val")
    if not pendentes:
        print("Nenhuma foto sem rotulo. Pronto para --escrever-yaml.")
        return 0
    if not janela_disponivel(cv2):
        print("ERRO: --rotular precisa de tela. Rode numa maquina com area de")
        print("trabalho - o PC serve, com `pip install opencv-python`.")
        return 1

    print(f"{len(pendentes)} foto(s) sem rotulo.")
    print("Para cada uma: ARRASTE a caixa e ENTER. C cancela, e depois:")
    print("  S pula esta | X apaga a foto | Q sai | outra tecla tenta de novo\n")

    feitas = 0
    for i, img_path in enumerate(pendentes, start=1):
        indice = indice_pelo_nome(img_path)
        if indice is None:
            print(f"  {img_path.name}: prefixo nao e classe conhecida, pulando")
            continue
        frame = cv2.imread(str(img_path))
        if frame is None:
            print(f"  {img_path.name}: nao abriu, pulando")
            continue
        lbl_path = img_path.parent.parent.parent / "labels" / \
            img_path.parent.name / f"{img_path.stem}.txt"

        while True:
            caixa = pedir_caixa(cv2, frame, f"[{i}/{len(pendentes)}] "
                                f"{img_path.name} - {CLASSES[indice]}")
            if caixa is not None:
                altura, largura = frame.shape[:2]
                lbl_path.write_text(
                    caixa_para_yolo(indice, *caixa, largura, altura),
                    encoding="utf-8")
                feitas += 1
                print(f"  {img_path.name}  rotulada")
                break
            vis = frame.copy()
            texto(cv2, vis, ["S pula | X apaga a foto | Q sai | outra tecla: "
                             "desenhar de novo"], (0, 255, 255))
            cv2.imshow(JANELA, vis)
            tecla = cv2.waitKey(0) & 0xFF
            if tecla in (ord("s"), ord("S")):
                print(f"  {img_path.name}  pulada (continua sem rotulo)")
                break
            if tecla in (ord("x"), ord("X")):
                img_path.unlink(missing_ok=True)
                print(f"  {img_path.name}  APAGADA")
                break
            if tecla in TECLAS_SAIR:
                cv2.destroyAllWindows()
                print(f"\n{feitas} rotulada(s). Faltam "
                      f"{len(sem_rotulo('train')) + len(sem_rotulo('val'))}.")
                return 0

    cv2.destroyAllWindows()
    restam = len(sem_rotulo("train")) + len(sem_rotulo("val"))
    print(f"\n{feitas} rotulada(s)." + (f" Restam {restam}." if restam else
                                       " Nenhuma pendente."))
    return 0


# ------------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(
        description="Captura e rotula imagens de treino pela camera do sistema.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument("--classe", default=None,
                    help=f"classe desta sessao: {', '.join(CLASSES)}")
    ap.add_argument("--conjunto", default="train",
                    help="train ou val (padrao %(default)s). Posicoes "
                         "DIFERENTES em cada um")
    ap.add_argument("--autorotular", action="store_true",
                    help="o modelo base marca a caixa sozinho. So para classe "
                         "que o COCO ja tem, como mouse")
    ap.add_argument("--confianca", type=float, default=0.5,
                    help="confianca minima da autorrotulagem (padrao "
                         "%(default)s)")
    ap.add_argument("--terminal", action="store_true",
                    help="forca o modo sem janela: ENTER no terminal, camera "
                         "no painel")
    ap.add_argument("--rotular", action="store_true",
                    help="desenha a caixa nas fotos que ficaram sem rotulo e "
                         "sai. Roda em qualquer maquina com tela")
    ap.add_argument("--escrever-yaml", action="store_true",
                    help="gera o dataset/data.yaml e sai")
    args = ap.parse_args()

    if args.escrever_yaml:
        escrever_yaml()
        return 0

    try:
        import cv2
    except ImportError:
        print("ERRO: falta o OpenCV. No PC: pip install opencv-python")
        return 1

    if args.rotular:
        return rotular_pendentes(cv2)

    if args.classe not in CLASSES:
        print(f"ERRO: --classe tem de ser uma de {CLASSES}.")
        return 1
    conjunto = CONJUNTOS.get(args.conjunto.lower())
    if conjunto is None:
        print(f"ERRO: --conjunto {args.conjunto!r} invalido. Use train ou val.")
        return 1
    indice = CLASSES.index(args.classe)

    modelo, coco = None, None
    if args.autorotular:
        from ultralytics import YOLO
        from src.vision.deteccao import ClasseDesconhecida, resolver_classe
        modelo = YOLO(str(CAMINHO_MODELO_BASE))
        try:
            coco = resolver_classe(args.classe, modelo.names)
        except ClasseDesconhecida as e:
            print(f"ERRO: o modelo base nao conhece {args.classe!r}: {e}")
            print("Rode sem --autorotular e desenhe a caixa.")
            return 1

    from coletar import abrir_camera
    capturar, fechar = abrir_camera()

    print(f"Classe {args.classe} (indice {indice}), conjunto {conjunto}"
          + (", autorrotulando" if modelo else "") + ".")
    print("Mova o objeto ENTRE as capturas: centro, bordas, cantos, em pe,")
    print("deitado, tombado.\n")

    try:
        if not args.terminal and janela_disponivel(cv2):
            n = capturar_com_janela(cv2, capturar, args.classe, conjunto,
                                    indice, modelo, coco, args.confianca)
        else:
            if not args.terminal:
                print("Sem area de trabalho para abrir janela - modo terminal.\n")
            n = capturar_no_terminal(cv2, capturar, args.classe, conjunto,
                                     indice, modelo, coco, args.confianca)
    finally:
        fechar()

    imgs, _ = pastas(conjunto)
    total = len(list(imgs.glob(f"{args.classe}_*.jpg")))
    print(f"\n{n} nesta sessao; {total} de {args.classe} em {conjunto}.")
    faltam = sem_rotulo(conjunto, args.classe)
    if faltam:
        print(f"{len(faltam)} sem rotulo — rode `python capturar_dataset.py "
              f"--rotular` numa maquina com tela.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
