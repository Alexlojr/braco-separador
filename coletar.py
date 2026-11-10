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
import json
import math
import statistics
import sys
import time
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(BASE_DIR))

from src.arm.coleta import (  # noqa: E402
    ControladorColeta,
    ParametrosColeta,
    SincroniaPerdida,
)
from src.arm.kinematics import PoseInalcancavel, carregar_config  # noqa: E402
from src.vision.estabilidade import (  # noqa: E402
    ParametrosEstabilidade,
    PortaoEstabilidade,
)
from src.vision.deteccao import (  # noqa: E402
    ClasseDesconhecida,
    carregar_preferencias,
    desenhar,
    melhor_bruta,
    melhor_deteccao,
    resolver_classe,
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


class SerialInstrumentada:
    """
    Repassa tudo para a serial de verdade, marcando o instante do primeiro
    comando de posicao e, se pedido, imprimindo o que passa.

    O eco existe porque em modo real so as descricoes das etapas apareciam, e
    as descricoes sao texto que EU escrevi: elas dizem o que o codigo
    pretendia, nao o que foi enviado. Quando o braco faz algo que a descricao
    nao explica, a primeira duvida vira "sera que o codigo certo esta
    rodando?" — e sem ver os comandos nao da para responder.

    A marca de tempo fica AQUI, dentro do write, e nao em volta da chamada de
    `executar`: a latencia definida na secao 5.1 do relatorio termina na
    emissao do primeiro comando de posicao na serial, e medir na borda de
    `executar` incluiria o planejamento da sequencia inteira, que acontece
    antes de qualquer byte sair.

    So `SET` conta. `SPEED`, `DETACH` e `ATTACH` nao sao comandos de posicao,
    e o primeiro SET da sequencia e a abertura da garra.
    """

    def __init__(self, alvo, eco: bool = False):
        self._alvo = alvo
        self._eco = eco
        self.t_primeiro_comando = None

    def armar_marco(self):
        """Esquece a marca da sequencia anterior."""
        self.t_primeiro_comando = None

    def write(self, dados: bytes) -> int:
        linha = dados.decode("utf-8", errors="ignore").strip()
        # Antes do write, nao depois: interessa quando o comando foi emitido,
        # nao quando o driver devolveu o controle.
        if self.t_primeiro_comando is None and linha.startswith("SET"):
            self.t_primeiro_comando = time.monotonic()
        if self._eco and linha:
            print(f"    -> {linha}")
        return self._alvo.write(dados)

    def __getattr__(self, nome):
        return getattr(self._alvo, nome)


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


def main() -> int:
    ap = argparse.ArgumentParser(description="Coleta automática ponta a ponta")
    ap.add_argument("--dry-run", action="store_true",
                    help="mostra os comandos sem abrir a serial")
    ap.add_argument("--forcar", action="store_true",
                    help="roda mesmo com a config marcada como não calibrada")
    ap.add_argument("--deslocamento", type=float, default=None,
                    help="correção radial em mm, sobrepondo o "
                         "deslocamento_radial_mm do braco.json. É meia "
                         "profundidade do apoio do OBJETO: 25 para a peça do "
                         "projeto (prato de 50 mm), ~12 para o mouse em pé. "
                         "Trocar o objeto de teste sem trocar isto faz a "
                         "garra fechar atrás dele")
    ap.add_argument("--eco", action="store_true",
                    help="imprime cada comando enviado e cada resposta do "
                         "firmware. Use quando o braco fizer algo que a "
                         "descricao da etapa nao explica")
    ap.add_argument("--deposito", default=None, metavar="X,Y",
                    help="posicao do deposito em mm, sobrepondo o "
                         "deposito_xy do braco.json. Mesma convencao da area "
                         "de coleta: origem no eixo da base, +X direita, "
                         "+Y frente. Sobrepoe o destino PADRAO; classes "
                         "listadas em coleta.depositos_por_classe continuam "
                         "indo para o deposito delas. "
                         "Ex: --deposito -150,0 poe o deposito a "
                         "esquerda, com a base girando ate 180 graus")
    ap.add_argument("--sem-soltar", action="store_true",
                    help="forca soltar_na_pega, soltar_no_giro e "
                         "soltar_garra_no_giro em false, sem editar o "
                         "braco.json. Util para comparar com e sem DETACH na "
                         "mesma maquina")
    ap.add_argument("--pausa", type=float, default=None,
                    help="pausa em segundos depois de cada etapa, sobrepondo "
                         "o pausa_s do braco.json. Sobe quando uma etapa "
                         "comeca com a anterior ainda andando")
    ap.add_argument("--velocidade", type=float, default=None,
                    help="graus por segundo, enviado por SPEED ao conectar. "
                         "Sem isto vale a velocidade compilada no firmware "
                         "(90 °/s). Use 40 na primeira coleta de verdade; "
                         "abaixo de ~23 o movimento entra na zona morta do "
                         "servo e vira solavanco")
    ap.add_argument("--confianca", type=float, default=None,
                    help="limiar de confiança do YOLO. Padrão: o "
                         "visao.confianca_minima do braco.json. "
                         "Abaixar aceita detecções mais fracas; o objeto "
                         "parado na mesa costuma pontuar menos que o mesmo "
                         "objeto na mão")
    # Uma classe, nao a lista inteira. O CLASSES_PERMITIDAS existe para o
    # painel, onde ver uma pessoa no quadro e informacao util. Na coleta ele
    # e defeito: `melhor_deteccao` devolve a caixa de MAIOR confianca entre as
    # permitidas, e a mao que ajusta a peca pontua como "person" acima do
    # proprio celular. A caixa muda de objeto no meio da janela, as posicoes
    # viram de outro lugar, e no fim da sequencia o braco iria buscar a mao.
    ap.add_argument("--classe", default=None,
                    help="classes a coletar, por nome ou número, separadas "
                         "por vírgula. Padrão: o visao.classe_alvo do "
                         "braco.json, que o painel também lê. Mais de uma "
                         "ativa a separação: cada classe vai para o depósito "
                         "de coleta.depositos_por_classe. Ex: --classe mouse,cell phone")
    ap.add_argument("--imgsz", type=int, default=640,
                    help="resolução de entrada do YOLO (padrão %(default)s). "
                         "O custo é quadrático: 320 roda ~4x mais rápido que "
                         "640, com menos acerto em objeto pequeno no quadro")
    args = ap.parse_args()

    # O firmware recusa fora de (0, 150] e responde ERR, que daqui chegaria
    # como um timeout mudo. Conferir aqui dá a mensagem certa, e antes de
    # abrir a serial.
    if args.velocidade is not None and not (0 < args.velocidade <= 150):
        print(f"ERRO: --velocidade {args.velocidade:.0f} fora da faixa aceita "
              "pelo firmware, que vai de 1 a 150 graus por segundo.")
        print("      O teto de 150 é de MÉDIA: a curva suavizada tem pico de")
        print("      1,875x, então 150 vira 281 °/s no meio do movimento.")
        return 1

    # A config manda; a linha de comando sobrepõe. Assim trocar o objeto no
    # braco.json troca para a coleta E para o painel de uma vez, e o
    # --classe continua servindo para um teste rápido sem editar arquivo.
    classe_cfg, confianca_cfg = carregar_preferencias(CAMINHO_CONFIG)
    if args.classe is None:
        args.classe = classe_cfg
    if args.confianca is None:
        args.confianca = confianca_cfg

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

    # O número que vale é o dos pontos de CONFERÊNCIA, que não entraram no
    # cálculo da matriz. O "erro_residual_mm" é o residual sobre os próprios
    # pontos de calibração e, com exatamente 4, ele é ~0 por construção — 8
    # incógnitas para 8 equações. Avisar com base nele é uma validação que
    # parece validar e não valida: fica calada numa calibração girada ou com
    # um canto clicado errado, que são os dois jeitos de errar isto.
    conferencia = homografia.meta.get("erro_conferencia_mm")
    if conferencia is None:
        print("AVISO: esta calibração nunca foi verificada contra pontos")
        print("       independentes. O residual dos próprios cantos é ~0 por")
        print("       construção e não prova nada. Rode:")
        print("         python calibrar_camera.py --conferir ...")
    elif conferencia.get("maximo", 0.0) > 5.0:
        print(f"AVISO: a calibração erra até {conferencia['maximo']:.1f} mm nos")
        print(f"       pontos de conferência (médio {conferencia['medio']:.1f} mm),")
        print("       acima do critério de 5 mm. Considere refazer.")
    else:
        print(f"Calibração da câmera: erro de {conferencia['medio']:.1f} mm médio, "
              f"{conferencia['maximo']:.1f} mm máximo.")

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
            # A velocidade é do firmware, e o padrão compilado é 90 °/s. Para
            # a primeira coleta de verdade isso é rápido demais para alguém
            # conseguir reagir com a mão no interruptor, e o roteiro de
            # montagem manda usar 40. Mandar pela serial era impossível com o
            # coletar.py rodando, porque ele é o dono da porta — a única saída
            # era fechar tudo, abrir o monitor, mandar SPEED e começar de novo.
            if args.velocidade is not None:
                if serial.definir_velocidade(args.velocidade):
                    print(f"Velocidade ajustada para {args.velocidade:.0f} °/s.")
                else:
                    print(f"AVISO: o firmware não aceitou SPEED {args.velocidade:.0f}. "
                          "Ele segue na velocidade compilada.")

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

    # A correção radial é propriedade do OBJETO, não do braço: o objeto de
    # teste muda mais do que o braco.json, e errar isto manda a garra fechar
    # atrás da peça sem nada reclamar.
    if args.deslocamento is not None:
        params_dict["deslocamento_radial"] = args.deslocamento
    if args.pausa is not None:
        params_dict["pausa_s"] = args.pausa
    if args.deposito is not None:
        try:
            dx_cli, dy_cli = (float(v) for v in args.deposito.split(","))
        except ValueError:
            print(f"ERRO: --deposito {args.deposito!r} invalido. Use X,Y em "
                  "mm, por exemplo -150,0")
            return 1
        params_dict["deposito"] = (dx_cli, dy_cli)
    if args.sem_soltar:
        for chave in ("soltar_na_pega", "soltar_no_giro", "soltar_garra_no_giro"):
            params_dict[chave] = False

    # O que vale AGORA, e nao o que esta escrito em algum arquivo. Editar o
    # braco.json na maquina errada e perder a tarde achando que testou outra
    # coisa e o tipo de engano que uma linha de saida evita.
    print("Sequencia: "
          + f"recuo {params_dict['recuo_entrada']:.0f} mm, "
          + f"correcao radial {params_dict['deslocamento_radial']:.0f} mm, "
          + (f"giro a {params_dict['raio_giro']:.0f} mm"
             if params_dict.get("raio_giro") else "giro sem recolher")
          + (f" em passos de {params_dict['passo_giro']:.0f} graus"
             if params_dict.get("passo_giro") else " de uma vez"))
    import math as _m
    _dx, _dy = params_dict["deposito"]
    print(f"Deposito padrao: ({_dx:.0f}, {_dy:.0f}) mm, que e base "
          f"{_m.degrees(_m.atan2(_dy, _dx)):.0f} graus e raio "
          f"{_m.hypot(_dx, _dy):.0f} mm.")

    soltas = [nome for chave, nome in (
        ("soltar_na_pega", "na pega"),
        ("soltar_no_giro", "ombro+cotovelo no giro"),
        ("soltar_garra_no_giro", "garra no giro"),
    ) if params_dict.get(chave)]
    print("DETACH: " + (", ".join(soltas) if soltas else "nenhum"))
    if params_dict.get("deslocamento_radial"):
        print(f"Correção radial: {params_dict['deslocamento_radial']:.0f} mm "
              f"(meia profundidade do apoio do objeto).")

    if args.eco and hasattr(serial, "ao_registrar"):
        # BUSY aparece dezenas de vezes por etapa e afogaria o resto; conta e
        # resume. O que interessa sao os OK/ERR e quanto cada etapa esperou.
        #
        # Pelo GANCHO, e nao lendo o historico: ele e um deque limitado a 200
        # entradas, e quem acompanhava por indice parava de receber quando
        # enchia. Pior: parecia que a serial tinha emudecido no meio da
        # sequencia, e foi o que me fez diagnosticar errado em 09/11.
        espera = {"n": 0}

        def mostrar(direcao, linha):
            if linha.startswith("BUSY"):
                if linha.endswith("1"):
                    espera["n"] += 1
                elif espera["n"]:
                    print(f"       <- BUSY 1 x{espera['n']}, depois 0")
                    espera["n"] = 0
                else:
                    print("       <- BUSY 0")
                return
            if direcao == "RX":
                print(f"       <- {linha}")
            elif direcao == "ERRO":
                print(f"       !! {linha}")

        serial.ao_registrar = mostrar

    # Sempre embrulha: a marca de tempo do primeiro comando e medicao, nao
    # depuracao, e precisa valer tambem na corrida que vai para o relatorio.
    serial = SerialInstrumentada(serial, eco=args.eco)

    controlador = ControladorColeta(
        serial, braco=braco, calib=calib, params=ParametrosColeta(**params_dict)
    )
    portao = PortaoEstabilidade(homografia, params_est)

    # ---- modelo e câmera ---------------------------------------------
    from ultralytics import YOLO

    modelo = YOLO(str(CAMINHO_MODELO))
    try:
        pedidos = [c.strip() for c in str(args.classe).split(",") if c.strip()]
        if not pedidos:
            print("ERRO: nenhuma classe pedida. Use --classe ou preencha "
                  "visao.classe_alvo no braco.json.")
            return 1
        # dict.fromkeys e nao set: preserva a ordem pedida, que e a ordem em
        # que o banner lista os destinos.
        classes_alvo = list(dict.fromkeys(
            resolver_classe(c, modelo.names) for c in pedidos))
    except ClasseDesconhecida as e:
        print(f"ERRO: {e}")
        return 1

    # Depósito por classe. Fica fora de ParametrosColeta de propósito:
    # o controlador é um só, calibrado uma vez, e o destino é escolhido
    # por ciclo. Classe sem entrada no mapa cai no deposito_xy.
    try:
        bruto = json.loads(CAMINHO_CONFIG.read_text(encoding="utf-8"))
        por_classe = bruto.get("coleta", {}).get(
            "depositos_por_classe", {}) or {}
    except (OSError, ValueError) as e:
        print(f"ERRO lendo depositos_por_classe: {e}")
        return 1

    destinos = {}
    for nome, xy in por_classe.items():
        try:
            destinos[nome.lower()] = (float(xy[0]), float(xy[1]))
        except (TypeError, ValueError, IndexError):
            print(f"ERRO: depositos_por_classe[{nome!r}] = {xy!r} não é "
                  f"[X, Y] em mm.")
            return 1

    capturar, fechar_camera = abrir_camera()

    nomes_alvo = ", ".join(f"{c} ({modelo.names[c]})"
                            for c in classes_alvo)
    print(f"Coletando: {nomes_alvo}. Confiança >= {args.confianca:.2f}.")
    for c in classes_alvo:
        nome = modelo.names[c]
        destino = destinos.get(nome.lower())
        if destino is None:
            dx_d, dy_d = params_dict["deposito"]
            origem = "deposito_xy (padrão)"
        else:
            dx_d, dy_d = destino
            origem = "depositos_por_classe"
        ang_d = math.degrees(math.atan2(dy_d, dx_d)) % 360.0
        print(f"  {nome} -> ({dx_d:.0f}, {dy_d:.0f}) mm, "
              f"base {ang_d:.0f}°, de {origem}")
    if len(classes_alvo) > 1 and not destinos:
        print("  AVISO: mais de uma classe e nenhum depósito por classe "
              "configurado — tudo vai para o mesmo lugar.")
    print(f"Portão: {params_est.janela} detecções dentro de "
          f"{params_est.tolerancia_mm:.1f} mm, com no máximo "
          f"{params_est.intervalo_maximo_s:.1f} s entre elas. Ctrl-C para sair.\n")

    ultima_coleta = 0.0
    coletados = 0
    abortadas = 0

    # Contadores de diagnóstico. A pergunta "por que não coleta?" tem três
    # respostas possíveis — não vê, vê fraco, ou vê e não assenta — e sem
    # medir as três ficam todas parecidas com "não funciona".
    t_inicio_taxa = time.monotonic()
    inferencias = 0
    taxa_relatada = False
    t_ultimo_aviso = 0.0

    # Latencia, na definicao da secao 5.1 do relatorio: do quadro que fecha
    # o portao ate o primeiro SET. Guardada por ciclo para sair mediana no
    # fim. O tempo de resposta, que inclui a espera do portao, vai separado:
    # ele depende da janela e da taxa de inferencia, nao do processamento.
    latencias_ms = []
    inferencia_ms = []
    respostas_s = []
    t_inferencia = None
    t_primeira_deteccao = None

    try:
        while True:
            frame = capturar()
            # Imediatamente apos a leitura do quadro, como a definicao de
            # latencia manda: o tempo da inferencia e percepcao e tem de
            # ficar DENTRO do intervalo medido.
            t_quadro = time.monotonic()

            if t_quadro - ultima_coleta < PAUSA_APOS_COLETA_S:
                continue

            resultado = modelo(frame, classes=classes_alvo,
                               imgsz=args.imgsz, conf=0.10, verbose=False)[0]
            t_inferencia = time.monotonic()

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

            if leitura.amostras == 1:
                # Primeiro quadro da janela que vai produzir a coordenada.
                # A diferenca entre este instante e o do quadro que fecha
                # o portao e a espera do portao.
                t_primeira_deteccao = t_quadro

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

            # O ponto da visão é a borda do objeto virada para a câmera. Se
            # houver correção configurada, mostrar o ponto corrigido evita a
            # confusão de comparar a linha de cima com uma régua posta no
            # centro da peça e achar que a calibração está errada.
            pega = controlador.ponto_de_pega(x_mm, y_mm)
            if (pega[0], pega[1]) != (x_mm, y_mm):
                print(f"  centro do apoio, com a correção radial de "
                      f"{controlador.params.deslocamento_radial:.0f} mm: "
                      f"({pega[0]:.0f}, {pega[1]:.0f}) mm")

            pode, motivo = controlador.alcancavel(x_mm, y_mm)
            if not pode:
                # Recusar e seguir observando: o objeto pode ser empurrado para
                # dentro do alcance, e travar o laço aqui não ajudaria ninguém.
                print(f"  fora de alcance: {motivo}\n")
                portao.rearmar()
                ultima_coleta = time.monotonic()
                continue

            # O destino sai da CLASSE detectada, e nao de um parametro
            # fixo: e isto que faz a separacao ser decidida pela
            # inferencia. Classe fora do mapa cai no deposito padrao.
            destino = destinos.get(deteccao.nome.lower())
            if destino is not None:
                print(f"  destino: {deteccao.nome} -> "
                      f"({destino[0]:.0f}, {destino[1]:.0f}) mm")

            try:
                serial.armar_marco()
                for etapa in controlador.executar(x_mm, y_mm,
                                                  deposito=destino):
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
                # Chegou ao fim sem exceção: foi uma coleta de verdade.
                coletados += 1
                t_cmd = serial.t_primeiro_comando
                if t_cmd is not None and t_inferencia is not None:
                    lat = (t_cmd - t_quadro) * 1000.0
                    inf = (t_inferencia - t_quadro) * 1000.0
                    latencias_ms.append(lat)
                    inferencia_ms.append(inf)
                    if t_primeira_deteccao is not None:
                        respostas_s.append(t_cmd - t_primeira_deteccao)
                    print(f"  latência da percepção: {lat:.0f} ms — "
                          f"{inf:.0f} ms de inferência e {lat - inf:.1f} ms "
                          f"de conversão, cinemática e serialização")
                print(f"  concluída ({coletados} no total)\n")
            except SincroniaPerdida as e:
                print(f"\n  COLETA INTERROMPIDA: {e}")
                print("  O braco pode ter ficado numa pose intermediaria.")
                print("  Duas causas, e as duas deixam a serial muda:")
                print("    - o Arduino reinicia se a tensao cair no pico de")
                print("      corrente, e volta para a pose de repouso sozinho;")
                print("    - porta USB que cai derruba a serial sem aviso.")
                abortadas += 1
                break
            except PoseInalcancavel as e:
                # Nada se moveu: `executar` valida o plano inteiro antes do
                # primeiro SET. Mas contar como concluída, que era o que
                # acontecia aqui, estraga o numerador da taxa de acerto — e é
                # justamente essa medida que vai para o capítulo de resultados.
                print(f"  coleta abortada, nada foi enviado: {e}\n")
                abortadas += 1

            portao.rearmar()
            ultima_coleta = time.monotonic()

    except KeyboardInterrupt:
        resumo = f"\nEncerrando. {coletados} coleta(s) nesta sessão"
        if abortadas:
            resumo += f", {abortadas} abortada(s) por pose inalcançável"
        print(resumo + ".")

    finally:
        if latencias_ms:
            print(f"\nLatência da percepção, em {len(latencias_ms)} "
                  f"ciclo(s): mediana "
                  f"{statistics.median(latencias_ms):.0f} ms "
                  f"(mínimo {min(latencias_ms):.0f}, "
                  f"máximo {max(latencias_ms):.0f}).")
            print(f"  Inferência do YOLO: mediana "
                  f"{statistics.median(inferencia_ms):.0f} ms. O resto da "
                  f"cadeia — homografia, cinemática e serialização — "
                  f"responde pela diferença.")
            if respostas_s:
                print(f"  Tempo de resposta, do primeiro quadro com "
                      f"detecção ao primeiro comando: mediana "
                      f"{statistics.median(respostas_s):.1f} s. Inclui a "
                      f"espera do portão, que depende da janela e da taxa "
                      f"de inferência.")
        fechar_camera()
        # Apaga a publicação: imagem antiga parada no painel parece ao vivo,
        # que é pior do que painel sem imagem.
        limpar()
        if hasattr(serial, "close"):
            serial.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
