#!/usr/bin/env python3
"""
Pipeline de coleta: câmera -> YOLO -> espera assentar -> mm -> serial -> braço.

É a ligação de ponta a ponta que os módulos já permitiam mas ninguém chamava.
O laço é:

    captura frame
      -> YOLO detecta o objeto de maior confiança
      -> portão de estabilidade acumula detecções até o objeto parar
      -> homografia converte a caixa em milímetros na mesa
      -> cinemática inversa converte em ângulos
      -> sequência de coleta manda os SET pela serial
      -> rearma e espera o próximo objeto

Roda sem interface: numa produção o braço não deveria depender de um navegador
aberto. A dashboard (app.py) continua sendo só visualização.

USO

    python coletar.py --dry-run      mostra os comandos, não abre serial
    python coletar.py                coleta de verdade

O `--dry-run` é o passo 5 do roteiro de montagem. Rode ele antes de energizar
os servos, conferindo se as coordenadas e os ângulos fazem sentido.

PRÉ-REQUISITOS

    config/braco.json       com "calibrado": true (passos 1 e 2)
    config/homografia.json  gerado por calibrar_camera.py (passo 4)

Sem os dois, o script recusa a rodar — apontar o braço com números de
placeholder é como ele quebra engrenagem.
"""

import argparse
import sys
import time
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(BASE_DIR))

from src.arm.coleta import ControladorColeta, ParametrosColeta  # noqa: E402
from src.arm.kinematics import PoseInalcancavel, carregar_config  # noqa: E402
from src.vision.estabilidade import (  # noqa: E402
    ParametrosEstabilidade,
    PortaoEstabilidade,
)
from src.vision.deteccao import (  # noqa: E402
    CONFIANCA_MINIMA,
    desenhar,
    melhor_bruta,
    melhor_deteccao,
)
from src.vision.fonte import aplicar_rotacao  # noqa: E402
from src.vision.quadro_publicado import limpar, publicar  # noqa: E402
from src.vision.homografia import CalibracaoInvalida, Homografia  # noqa: E402

CAMINHO_CONFIG = BASE_DIR / "config" / "braco.json"
CAMINHO_HOMOGRAFIA = BASE_DIR / "config" / "homografia.json"
CAMINHO_MODELO = BASE_DIR / "models" / "yolov8n.pt"

# Depois de coletar, ignora detecções por este tempo. Sem isso o braço volta
# do depósito, vê o objeto seguinte (ou a sombra do que acabou de sair) e sai
# de novo antes de a cena assentar.
PAUSA_APOS_COLETA_S = 2.0


class SerialDeMentira:
    """Registra os comandos em vez de enviar. É o que o --dry-run usa."""

    def __init__(self):
        self.linhas = []
        self.braco_real = False
        self.port = "DRY-RUN"

    def write(self, dados: bytes) -> int:
        linha = dados.decode("utf-8", errors="ignore").strip()
        if linha:
            self.linhas.append(linha)
            print(f"    -> {linha}")
        return len(dados)


def abrir_camera(largura=640, altura=480):
    """Picamera2 no Pi; cai para webcam USB quando ela não existe."""
    try:
        from picamera2 import Picamera2

        cam = Picamera2()
        cam.configure(
            cam.create_preview_configuration(
                main={"format": "RGB888", "size": (largura, altura)},
                controls={"AwbEnable": True, "AwbMode": 0},
            )
        )
        cam.start()
        time.sleep(2)  # deixa o auto-exposure assentar antes do primeiro frame
        return lambda: aplicar_rotacao(cam.capture_array()), cam.stop

    except ImportError:
        import cv2

        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            raise RuntimeError("nenhuma câmera disponível (nem Picamera2, nem webcam)")

        def captura():
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError("falha ao capturar frame")
            return aplicar_rotacao(frame)

        return captura, cap.release


def resolver_classe(pedido, nomes):
    """
    Aceita "cell phone", "CELL PHONE" ou "67" e devolve o número da classe.

    Nome em vez de número porque trocar o objeto de teste é frequente, e
    decorar que celular é 67 e mouse é 64 não ajuda ninguém. Erro de digitação
    devolve None depois de listar o que se parece com o pedido — melhor que
    rodar meia hora detectando a classe errada.
    """
    texto = str(pedido).strip()

    if texto.lstrip("-").isdigit():
        numero = int(texto)
        if numero in nomes:
            return numero
        print(f"ERRO: não existe a classe {numero}. O modelo tem "
              f"{len(nomes)} classes, de 0 a {max(nomes)}.")
        return None

    procurado = texto.lower()
    for numero, nome in nomes.items():
        if nome.lower() == procurado:
            return numero

    parecidos = [f"{n} ({i})" for i, n in nomes.items() if procurado in n.lower()]
    print(f"ERRO: nenhuma classe chamada {texto!r}.")
    if parecidos:
        print("      Parecidas: " + ", ".join(parecidos[:8]))
    else:
        print("      Pequenas e boas para teste: mouse (64), remote (65), "
              "cell phone (67), scissors (76), book (73), cup (41).")
    return None


def main() -> int:
    ap = argparse.ArgumentParser(description="Coleta automática ponta a ponta")
    ap.add_argument("--dry-run", action="store_true",
                    help="mostra os comandos sem abrir a serial")
    ap.add_argument("--forcar", action="store_true",
                    help="roda mesmo com a config marcada como não calibrada")
    ap.add_argument("--confianca", type=float, default=CONFIANCA_MINIMA,
                    help="limiar de confiança do YOLO (padrão %(default)s). "
                         "Abaixar aceita detecções mais fracas; o objeto "
                         "parado na mesa costuma pontuar menos que o mesmo "
                         "objeto na mão")
    # Uma classe, nao a lista inteira. O CLASSES_PERMITIDAS existe para o
    # painel, onde ver uma pessoa no quadro e informacao util. Na coleta ele
    # e defeito: `melhor_deteccao` devolve a caixa de MAIOR confianca entre as
    # permitidas, e a mao que ajusta a peca pontua como "person" acima do
    # proprio celular. A caixa muda de objeto no meio da janela, as posicoes
    # viram de outro lugar, e no fim da sequencia o braco iria buscar a mao.
    ap.add_argument("--classe", default="cell phone",
                    help="classe COCO a coletar, por nome ou número "
                         "(padrão %(default)r). Só ela entra na janela de "
                         "estabilidade; o painel continua mostrando todas. "
                         "Objetos pequenos e achatados dão coordenada melhor: "
                         "mouse, remote, scissors, book, cup")
    ap.add_argument("--imgsz", type=int, default=640,
                    help="resolução de entrada do YOLO (padrão %(default)s). "
                         "O custo é quadrático: 320 roda ~4x mais rápido que "
                         "640, com menos acerto em objeto pequeno no quadro")
    args = ap.parse_args()

    # ---- configuração ------------------------------------------------
    braco, calib, params_dict, calibrado = carregar_config(CAMINHO_CONFIG)

    if not calibrado and not args.forcar:
        print("ERRO: config/braco.json está com \"calibrado\": false.")
        print("      Os elos e limites ainda são placeholders — o braço apontaria")
        print("      para o lugar errado. Faça os passos 1 e 2 do README, ou use")
        print("      --forcar se souber o que está fazendo.")
        return 1

    try:
        homografia = Homografia.carregar(CAMINHO_HOMOGRAFIA)
    except CalibracaoInvalida as e:
        print(f"ERRO: {e}")
        return 1

    residual = homografia.meta.get("erro_residual_mm")
    if residual is not None and residual > 5.0:
        print(f"AVISO: a calibração da câmera tem erro residual de {residual:.1f} mm.")
        print("       Acima de ~5 mm costuma ser ponto mal medido. Considere refazer.")

    params_est = ParametrosEstabilidade.de_json(CAMINHO_CONFIG)

    import json
    acs = json.loads(CAMINHO_CONFIG.read_text(encoding="utf-8")).get("acs712", {})

    # ---- serial ------------------------------------------------------
    if args.dry_run:
        serial = SerialDeMentira()
        print("MODO DRY-RUN: nada será enviado pela serial.\n")
    else:
        from src.arm.mock_arm_serial import MockArmSerial, detectar_porta_arduino

        porta = detectar_porta_arduino()
        serial = MockArmSerial(porta_arduino=porta)
        if serial.braco_real:
            print(f"Arduino encontrado em {serial.port}.")

            # Empurra os limites do JSON para o firmware. Sem isto eles vivem
            # em dois lugares e dessincronizam calados: alguém recalibra e
            # corrige o JSON, esquece de recompilar o sketch, e o firmware
            # segue permitindo um ângulo que já se sabe que bate.
            aceitas = serial.enviar_limites(calib)
            if aceitas == 4:
                print("Limites de junta enviados ao firmware.")
            else:
                print(f"AVISO: o firmware aceitou {aceitas}/4 limites. Ele pode ser")
                print("       uma versão antiga, sem o comando LIM. Os limites do")
                print("       sketch é que valem — confira se batem com o JSON.")

            if serial.configurar_sensor(acs):
                lim = float(acs.get("limite_stall_amp", 0.0))
                if lim > 0:
                    print(f"Sensor de corrente: alívio de travamento ligado em {lim:.2f} A.\n")
                else:
                    print("Sensor de corrente configurado; alívio DESLIGADO (limite 0).")
                    print("       Meça a corrente de pega com AMP e ajuste limite_stall_amp.\n")
            else:
                print("AVISO: firmware não aceitou SENS/LIMAMP. Sensor de corrente")
                print("       indisponível ou firmware antigo.\n")
        else:
            print("AVISO: Arduino não encontrado. Rodando só na simulação.\n")

    controlador = ControladorColeta(
        serial, braco=braco, calib=calib, params=ParametrosColeta(**params_dict)
    )
    portao = PortaoEstabilidade(homografia, params_est)

    # ---- modelo e câmera ---------------------------------------------
    from ultralytics import YOLO

    modelo = YOLO(str(CAMINHO_MODELO))
    classe_alvo = resolver_classe(args.classe, modelo.names)
    if classe_alvo is None:
        return 1
    capturar, fechar_camera = abrir_camera()

    print(f"Coletando a classe {classe_alvo} "
          f"({modelo.names[classe_alvo]}), confiança >= {args.confianca:.2f}.")
    print(f"Portão: {params_est.janela} detecções dentro de "
          f"{params_est.tolerancia_mm:.1f} mm, com no máximo "
          f"{params_est.intervalo_maximo_s:.1f} s entre elas. Ctrl-C para sair.\n")

    ultima_coleta = 0.0
    coletados = 0

    # Contadores de diagnóstico. A pergunta "por que não coleta?" tem três
    # respostas possíveis — não vê, vê fraco, ou vê e não assenta — e sem
    # medir as três ficam todas parecidas com "não funciona".
    t_inicio_taxa = time.monotonic()
    inferencias = 0
    taxa_relatada = False
    t_ultimo_aviso = 0.0

    try:
        while True:
            frame = capturar()

            if time.monotonic() - ultima_coleta < PAUSA_APOS_COLETA_S:
                continue

            resultado = modelo(frame, classes=[classe_alvo],
                               imgsz=args.imgsz, conf=0.10, verbose=False)[0]

            # O `conf` da chamada fica BAIXO de propósito: o limiar que decide
            # é o `args.confianca`, aplicado abaixo. Deixar o modelo devolver
            # as caixas fracas é o que permite dizer "vi com 0,38" em vez de
            # "não vi nada", que pedem providências opostas.
            bruta = melhor_bruta(resultado, modelo.names)
            deteccao = melhor_deteccao(resultado, modelo.names, args.confianca)

            inferencias += 1
            decorrido = time.monotonic() - t_inicio_taxa
            if not taxa_relatada and inferencias >= 10:
                taxa = inferencias / decorrido
                espera = (params_est.janela - 1) / taxa
                print(f"Taxa de inferência: {taxa:.1f}/s a imgsz {args.imgsz}. "
                      f"Nessa taxa o portão precisa de ~{espera:.1f} s de objeto "
                      f"parado.")
                if 1.0 / taxa > params_est.intervalo_maximo_s:
                    print(f"AVISO: o intervalo entre inferências ({1/taxa:.2f} s) "
                          f"passa do intervalo_maximo_s ({params_est.intervalo_maximo_s:.2f} s): "
                          f"cada detecção zera a janela e o portão nunca fecha. "
                          f"Suba o intervalo_maximo_s no braco.json ou baixe o --imgsz.")
                print()
                taxa_relatada = True

            # Publica o quadro para o painel. Quem roda a coleta e dono da
            # camera; o app.py le daqui em vez de tentar abrir a dele, que
            # falharia com "Pipeline handler in use by another process".
            publicar(
                desenhar(frame, deteccao) if deteccao else frame,
                {"rotulo": deteccao.rotulo if deteccao else None,
                 "coletados": coletados},
            )

            if deteccao is None:
                portao.perdeu_deteccao()
                # Uma linha a cada 2 s, não a cada frame: a informação é o
                # estado, e repetir a 3 FPS só empurra o resto para fora da
                # tela.
                agora = time.monotonic()
                if agora - t_ultimo_aviso >= 2.0:
                    t_ultimo_aviso = agora
                    if bruta is None:
                        print("  nada das classes permitidas no quadro")
                    else:
                        print(f"  {bruta.nome} com {bruta.confianca:.2f}, abaixo "
                              f"do limiar de {args.confianca:.2f} — "
                              f"--confianca {max(0.1, bruta.confianca - 0.05):.2f} "
                              f"aceitaria")
                continue

            leitura = portao.observar(*deteccao.caixa)

            if not leitura.estavel:
                print(f"  {deteccao.rotulo} — {leitura.motivo}")
                # Primeira amostra da janela: confere alcance já, mesmo com a
                # coordenada ainda crua. Esperar o portão fechar para só então
                # recusar gasta ~1,5 s de objeto parado e não diz nada que não
                # desse para saber antes — e o que o operador precisa ouvir é
                # "arraste a peça", não "falhou".
                if leitura.amostras == 1:
                    x_cru, y_cru = homografia.caixa_para_mm(*deteccao.caixa)
                    pode, _ = controlador.alcancavel(x_cru, y_cru)
                    if not pode:
                        print(f"    AVISO: ({x_cru:.0f}, {y_cru:.0f}) mm não é "
                              f"coletável. A sequência completa só fecha entre "
                              f"~121 e ~188 mm do eixo.")
                continue

            x_mm, y_mm = leitura.x_mm, leitura.y_mm
            print(f"\n{deteccao.rotulo} assentado em "
                  f"({x_mm:.0f}, {y_mm:.0f}) mm — {leitura.motivo}")

            pode, motivo = controlador.alcancavel(x_mm, y_mm)
            if not pode:
                # Recusar e seguir observando: o objeto pode ser empurrado para
                # dentro do alcance, e travar o laço aqui não ajudaria ninguém.
                print(f"  fora de alcance: {motivo}\n")
                portao.rearmar()
                ultima_coleta = time.monotonic()
                continue

            try:
                for etapa in controlador.executar(x_mm, y_mm):
                    linha = f"  {etapa.descricao}"
                    # Depois de fechar, a corrente da garra é o dado que diz se
                    # a pega por encaixe está funcionando sem forçar o servo.
                    # Vai direto para o capítulo de resultados.
                    if "fechando" in etapa.descricao.lower():
                        medir = getattr(serial, "corrente", None)
                        amp = medir() if medir else None
                        if amp is not None:
                            linha += f"  (garra: {amp:.2f} A)"
                    print(linha)
            except PoseInalcancavel as e:
                print(f"  coleta abortada: {e}")

            coletados += 1
            print(f"  concluída ({coletados} no total)\n")
            portao.rearmar()
            ultima_coleta = time.monotonic()

    except KeyboardInterrupt:
        print(f"\nEncerrando. {coletados} coleta(s) nesta sessão.")

    finally:
        fechar_camera()
        # Apaga a publicação: imagem antiga parada no painel parece ao vivo,
        # que é pior do que painel sem imagem.
        limpar()
        if hasattr(serial, "close"):
            serial.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
