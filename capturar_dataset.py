#!/usr/bin/env python3
"""
Captura as imagens de treino pela MESMA camera que vai operar o sistema.

POR QUE pela camera do Pi, e nao com celular: o detector so vai ver esta
camera, nesta altura, com esta lente e esta iluminacao. Foto de celular em
perspectiva arbitraria e um dominio que nao existe na operacao - nao e
variacao util, e distribuicao errada.

O QUE varia, entao: a POSICAO da peca na placa (centro, bordas, cantos, perto
do limite de alcance) e a ORIENTACAO (em pe, deitada, tombada). E isso que o
modelo precisa generalizar. Mover a peca entre quadros e o trabalho; o resto
e so apertar Enter.

DIVISAO TREINO/VALIDACAO POR POSICAO, e nao aleatoria. Quadros da mesma cena
sao quase identicos, e divisao aleatoria joga copias do mesmo quadro nos dois
lados. A validacao passa a medir memorizacao e devolve mAP alto que nao
significa nada. Por isso o conjunto e escolhido na linha de comando: posicoes
de validacao sao posicoes que o treino nunca viu.

USO

    # umas 20-30 posicoes por classe no treino
    python capturar_dataset.py --classe peca
    python capturar_dataset.py --classe mouse --autorotular

    # 5-8 posicoes DIFERENTES para validacao
    python capturar_dataset.py --classe peca  --conjunto val
    python capturar_dataset.py --classe mouse --conjunto val --autorotular

    # depois, uma vez:
    python capturar_dataset.py --escrever-yaml

Enter captura. 'd' apaga o ultimo. Ctrl-C ou 'q' encerra.

AUTORROTULAGEM: com --autorotular, o modelo base marca a caixa sozinho. Serve
para o mouse, que e classe do COCO e sai com confianca de 0,95 a 0,99. NAO
serve para a peca do projeto, que o modelo base nunca viu - essa precisa de
anotacao a mao. Confira os rotulos automaticos antes de treinar: rotulo errado
e pior que rotulo faltando, porque o treino aprende o erro.

FORMATO: YOLO. Uma linha por objeto, `classe cx cy w h`, tudo normalizado de
0 a 1 pela largura e altura da imagem.

A imagem e gravada com a MESMA convencao de canais que a inferencia consome
(ver `src/vision/deteccao.py`), para o modelo treinar no que vai ver.
"""

import argparse
import sys
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(BASE_DIR))

RAIZ = BASE_DIR / "dataset"
CAMINHO_MODELO = BASE_DIR / "models" / "yolov8n.pt"

# Ordem das classes no data.yaml. O indice e o numero que vai no rotulo, e
# mudar a ordem depois de anotar invalida todos os rotulos ja escritos.
CLASSES = ["peca", "mouse"]

CONJUNTOS = {"train": "train", "treino": "train", "val": "val",
             "validacao": "val", "validação": "val"}


def pastas(conjunto):
    imgs = RAIZ / "images" / conjunto
    lbls = RAIZ / "labels" / conjunto
    imgs.mkdir(parents=True, exist_ok=True)
    lbls.mkdir(parents=True, exist_ok=True)
    return imgs, lbls


def escrever_yaml():
    """O data.yaml que o treino consome. Caminhos absolutos: o Ultralytics
    resolve relativo ao diretorio de datasets dele, nao ao do projeto."""
    RAIZ.mkdir(parents=True, exist_ok=True)
    nomes = "\n".join(f"  {i}: {n}" for i, n in enumerate(CLASSES))
    texto = (
        f"# Gerado por capturar_dataset.py\n"
        f"path: {RAIZ.as_posix()}\n"
        f"train: images/train\n"
        f"val: images/val\n"
        f"\n"
        f"names:\n{nomes}\n"
    )
    destino = RAIZ / "data.yaml"
    destino.write_text(texto, encoding="utf-8")
    print(f"Escrito {destino}\n")
    print(texto)
    print("ATENCAO: estes nomes de classe passam a valer no lugar dos do COCO.")
    print("Atualize 'coleta.depositos_por_classe' no braco.json com eles,")
    print("senao a separacao manda tudo para o deposito padrao SEM AVISO.")


def contar(imgs, classe):
    return len(list(imgs.glob(f"{classe}_*.jpg")))


def main():
    ap = argparse.ArgumentParser(
        description="Captura imagens de treino pela camera do sistema.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument("--classe", default=None,
                    help=f"classe capturada nesta sessao: {', '.join(CLASSES)}")
    ap.add_argument("--conjunto", default="train",
                    help="train ou val (padrao %(default)s). Use posicoes "
                         "DIFERENTES em cada um: divisao por posicao e o que "
                         "evita quadro quase identico nos dois lados")
    ap.add_argument("--autorotular", action="store_true",
                    help="o modelo base marca a caixa sozinho. So funciona "
                         "para classe que o COCO ja tem, como mouse")
    ap.add_argument("--confianca", type=float, default=0.5,
                    help="confianca minima da autorrotulagem (padrao "
                         "%(default)s)")
    ap.add_argument("--escrever-yaml", action="store_true",
                    help="gera o dataset/data.yaml e sai")
    args = ap.parse_args()

    if args.escrever_yaml:
        escrever_yaml()
        return 0

    if args.classe is None:
        print(f"ERRO: --classe e obrigatoria. Opcoes: {', '.join(CLASSES)}")
        return 1
    if args.classe not in CLASSES:
        print(f"ERRO: classe {args.classe!r} nao esta em CLASSES = {CLASSES}.")
        print("Acrescente ao fim da lista em capturar_dataset.py; nao mude a")
        print("ordem das existentes, que e o numero gravado nos rotulos.")
        return 1

    conjunto = CONJUNTOS.get(args.conjunto.lower())
    if conjunto is None:
        print(f"ERRO: --conjunto {args.conjunto!r} invalido. Use train ou val.")
        return 1

    indice = CLASSES.index(args.classe)
    imgs, lbls = pastas(conjunto)

    import cv2  # noqa: E402
    from coletar import abrir_camera  # noqa: E402

    modelo = None
    if args.autorotular:
        from ultralytics import YOLO  # noqa: E402
        from src.vision.deteccao import melhor_deteccao, resolver_classe  # noqa: E402

        modelo = YOLO(str(CAMINHO_MODELO))
        try:
            coco = resolver_classe(args.classe, modelo.names)
        except Exception as e:
            print(f"ERRO: --autorotular pede uma classe que o modelo base "
                  f"conheca, e {args.classe!r} nao serve: {e}")
            print("Capture sem --autorotular e anote a mao.")
            return 1
        print(f"Autorrotulando como {modelo.names[coco]} (classe COCO {coco}),")
        print(f"gravado no dataset como indice {indice} ({args.classe}).")

    capturar, fechar = abrir_camera()
    ja = contar(imgs, args.classe)

    print(f"\nClasse {args.classe} (indice {indice}), conjunto {conjunto}.")
    print(f"Ja existem {ja} imagem(ns) dessa classe nesse conjunto.")
    print("\nMova a peca ENTRE cada captura: posicao e orientacao diferentes.")
    print("Centro, bordas, cantos, em pe, deitada, tombada.")
    print("Enter captura | d apaga a ultima | q encerra\n")

    gravadas = []
    try:
        while True:
            try:
                tecla = input(f"[{len(gravadas) + ja}] > ").strip().lower()
            except EOFError:
                break

            if tecla == "q":
                break

            if tecla == "d":
                if not gravadas:
                    print("  nada para apagar nesta sessao")
                    continue
                img, lbl = gravadas.pop()
                img.unlink(missing_ok=True)
                lbl.unlink(missing_ok=True)
                print(f"  apagado {img.name}")
                continue

            if tecla:
                print("  Enter captura, d apaga, q encerra")
                continue

            frame = capturar()
            n = ja + len(gravadas) + 1
            nome = f"{args.classe}_{conjunto}_{n:03d}"
            destino_img = imgs / f"{nome}.jpg"
            destino_lbl = lbls / f"{nome}.txt"

            if not cv2.imwrite(str(destino_img), frame):
                print(f"  ERRO: nao consegui gravar {destino_img}")
                continue

            aviso = ""
            if modelo is not None:
                altura, largura = frame.shape[:2]
                resultado = modelo(frame, classes=[coco], imgsz=640,
                                   conf=args.confianca, verbose=False)[0]
                det = melhor_deteccao(resultado, modelo.names, args.confianca)
                if det is None:
                    # Sem rotulo e sem imagem: imagem sem rotulo entra no
                    # treino como "aqui nao tem nada", que ensina o oposto.
                    destino_img.unlink(missing_ok=True)
                    print("  NAO detectou — descartada. Reposicione e repita.")
                    continue
                cx = ((det.x1 + det.x2) / 2) / largura
                cy = ((det.y1 + det.y2) / 2) / altura
                w = abs(det.x2 - det.x1) / largura
                h = abs(det.y2 - det.y1) / altura
                destino_lbl.write_text(
                    f"{indice} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}\n",
                    encoding="utf-8",
                )
                aviso = f"  rotulo automatico, confianca {det.confianca:.2f}"
            else:
                # Arquivo vazio e rotulo valido dizendo "sem objeto". Nao criar
                # e o certo: a anotacao a mao vai escrever este arquivo.
                aviso = "  SEM rotulo — anotar a mao"

            gravadas.append((destino_img, destino_lbl))
            print(f"  {destino_img.name}{aviso}")

    except KeyboardInterrupt:
        print()
    finally:
        fechar()

    total = contar(imgs, args.classe)
    print(f"\n{len(gravadas)} nesta sessao; {total} de {args.classe} em "
          f"{conjunto}.")

    sem_rotulo = [p for p in imgs.glob(f"{args.classe}_*.jpg")
                  if not (lbls / f"{p.stem}.txt").exists()]
    if sem_rotulo:
        print(f"\n{len(sem_rotulo)} imagem(ns) de {args.classe} sem rotulo em "
              f"{conjunto}.")
        print(f"Anote antes de treinar: {imgs}")
        print("Imagem sem rotulo nao e ignorada pelo treino — ela entra como")
        print("exemplo de quadro VAZIO, e ensina o modelo a nao detectar nada.")

    print("\nQuando os dois conjuntos estiverem prontos:")
    print("  python capturar_dataset.py --escrever-yaml")
    print(f"  yolo detect train data={(RAIZ / 'data.yaml').as_posix()} "
          f"model=yolov8n.pt epochs=60 imgsz=640 batch=8")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
