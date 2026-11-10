"""
Dashboard de detecção — Raspberry Pi 5 + Camera Module 3 + YOLOv8n.

Mostra a câmera ao vivo com as detecções desenhadas e conta os objetos que
passam. É a camada de visão do braço separador: quando a homografia estiver
calibrada, é daqui que sai a bounding box que vira coordenada em mm.

O braço não é acionado por esta dashboard. Para movimentar servos durante a
montagem, use:

    python src/arm/mock_arm_serial.py    terminal interativo (SET/GET/HOME)
    python mapa_alcance.py               o que o braço alcança, sem hardware
"""

import time
from collections import defaultdict
from pathlib import Path

import streamlit as st
from picamera2 import Picamera2
from ultralytics import YOLO

from src.vision.fonte import aplicar_rotacao
from src.vision.quadro_publicado import ler as ler_publicado
from src.telemetria import ler as ler_telemetria
from src.vision.deteccao import (
    ClasseDesconhecida,
    carregar_preferencias,
    resolver_classe,
    INTERVALO_DETECCAO,
    caminho_modelo,
    desenhar,
    melhor_deteccao,
    para_rgb,
)

BASE_DIR = Path(__file__).resolve().parent
# Os pesos saem do braco.json (visao.modelo), o mesmo que a coleta le:
# painel e coleta tem de estar olhando pelo MESMO modelo.
CAMINHO_MODELO = caminho_modelo(BASE_DIR / "config" / "braco.json", BASE_DIR)

st.set_page_config(page_title="Detecção de Objetos", layout="wide")
st.title("Detecção de Objetos (YOLO)")


@st.cache_resource
def carregar_modelo():
    return YOLO(str(CAMINHO_MODELO))


@st.cache_resource
def iniciar_camera():
    """
    Abre a camera. So e chamada quando NAO ha coleta publicando quadros.

    Um processo por vez consegue a camera. Se o coletar.py estiver rodando,
    ele e o dono e o painel le a publicacao dele; abrir aqui so produziria
    "Pipeline handler in use by another process".
    """
    picam2 = Picamera2()
    picam2.configure(
        picam2.create_preview_configuration(
            main={"format": "RGB888", "size": (640, 480)},
            controls={"AwbEnable": True, "AwbMode": 0}  # Auto white balance
        )
    )
    picam2.start()
    return picam2


model = carregar_modelo()
picam2 = None   # aberta sob demanda, se nao houver coleta publicando

# O que procurar vem do braco.json, o MESMO arquivo que o coletar.py le. Ter
# a classe escrita aqui tambem foi o que fez o mouse nao aparecer no painel:
# a lista fixa era [pessoa, celular], e mouse e a classe 64. Nao havia caixa
# para desenhar, e nada na tela dizia por que.
CAMINHO_CONFIG = BASE_DIR / "config" / "braco.json"
_pedido, CONFIANCA = carregar_preferencias(CAMINHO_CONFIG)
try:
    CLASSE_ALVO = resolver_classe(_pedido, model.names)
except ClasseDesconhecida as e:
    st.error(f"classe_alvo invalida no braco.json: {e}")
    st.stop()

st.sidebar.markdown(
    f"**Procurando:** {model.names[CLASSE_ALVO]} ({CLASSE_ALVO})\n\n"
    f"**Confiança mínima:** {CONFIANCA:.2f}\n\n"
    f"Vem da seção `visao` do `braco.json`, a mesma que o `coletar.py` lê."
)

if "rodando" not in st.session_state:
    st.session_state.rodando = False

if "contagem" not in st.session_state:
    st.session_state.contagem = defaultdict(int)

if "ultimo_objeto" not in st.session_state:
    st.session_state.ultimo_objeto = None

if st.sidebar.button("Iniciar câmera"):
    st.session_state.rodando = True

if st.sidebar.button("Parar"):
    st.session_state.rodando = False

col1, col2 = st.columns([2, 1])

camera_placeholder = col1.empty()
log_placeholder = col1.empty()
status_placeholder = col2.empty()
servos_placeholder = col2.empty()
tabela_placeholder = col2.empty()

# Um servo cujo angulo comandado mudou ha menos que isto aparece como "em
# movimento". E aproximado: o painel so sabe o que foi COMANDADO (malha
# aberta), e os movimentos do ciclo duram de 0,1 a 1,7 s.
JANELA_MOVIMENTO_S = 1.5


def barra(fracao, largura=10):
    cheio = max(0, min(largura, round(fracao * largura)))
    return "\u2588" * cheio + "\u2591" * (largura - cheio)


def mostrar_telemetria(tele):
    """Angulos comandados e log da coleta, publicados pelo coletar.py."""
    if tele is None:
        servos_placeholder.empty()
        log_placeholder.empty()
        return
    agora = time.time()
    linhas = ["#### Servos — último ângulo comandado", "",
              "| Junta | Ângulo | Estado |", "|---|---|---|"]
    for sv in tele.get("servos", []):
        ang = float(sv.get("ang", 0.0))
        if sv.get("solto"):
            estado = "solto (sem sinal)"
        elif agora - float(sv.get("t", 0.0)) < JANELA_MOVIMENTO_S:
            estado = "**em movimento**"
        else:
            estado = "parado"
        linhas.append(f"| {sv.get('nome')} ({sv.get('canal')}) "
                      f"| **{ang:.1f}°**<br>{barra(ang / 180.0)} | {estado} |")
    linhas.append("")
    linhas.append("<sub>Servos sem realimentação: o valor é o último ângulo "
                  "enviado ao Arduino; durante um movimento o servo ainda está "
                  "a caminho dele.</sub>")
    servos_placeholder.markdown("\n".join(linhas), unsafe_allow_html=True)

    # Mais recente no TOPO: com a mais nova embaixo, ela cai fora da area
    # visivel do bloco e o log parece parado justamente durante o ciclo.
    log = tele.get("log", [])[-15:][::-1]
    with log_placeholder.container():
        st.markdown("**Log da coleta** — mais recente no topo")
        st.code("\n".join(log) if log else "(sem mensagens ainda)", language=None)

frame_count = 0
ultima_detec = None

while st.session_state.rodando:
    # Preferencia pelo quadro publicado pela coleta: ela ja fez a inferencia,
    # e repetir aqui seria pagar o YOLO duas vezes no mesmo Pi.
    publicado = ler_publicado()
    if publicado is not None:
        frame, meta = publicado
        camera_placeholder.image(para_rgb(frame), channels="RGB",
                                 caption="recebendo do coletar.py")
        rotulo = meta.get("rotulo")
        status_placeholder.markdown(
            f"**Coleta em andamento** — {meta.get('coletados', 0)} peça(s)  \n"
            f"{('vendo: ' + rotulo) if rotulo else 'nenhum objeto no campo'}"
        )
        mostrar_telemetria(ler_telemetria())
        time.sleep(0.2)
        continue

    # Sem coleta rodando, nao ha servos nem log para mostrar.
    mostrar_telemetria(None)

    if picam2 is None:
        picam2 = iniciar_camera()

    frame = aplicar_rotacao(picam2.capture_array())
    frame_count += 1

    if frame_count % INTERVALO_DETECCAO == 0:
        resultado = model(frame, classes=[CLASSE_ALVO], verbose=False)[0]
        ultima_detec = melhor_deteccao(resultado, model.names, CONFIANCA)

        if ultima_detec is not None:
            if st.session_state.ultimo_objeto != ultima_detec.nome:
                st.session_state.contagem[ultima_detec.nome] += 1
                st.session_state.ultimo_objeto = ultima_detec.nome
        else:
            st.session_state.ultimo_objeto = None

    if ultima_detec is not None:
        desenhar(frame, ultima_detec)
        status_placeholder.success(f"Detectando: {ultima_detec.rotulo}")
    else:
        status_placeholder.info(
            f"Nenhum {model.names[CLASSE_ALVO]} com confiança >= {CONFIANCA:.2f}")

    camera_placeholder.image(para_rgb(frame))

    tabela_placeholder.subheader("Contagem de objetos")

    if len(st.session_state.contagem) > 0:
        tabela_placeholder.table(dict(st.session_state.contagem))
    else:
        tabela_placeholder.write("Nenhum objeto contado ainda.")

    time.sleep(0.01)

if not st.session_state.rodando:
    st.info("Clique em **Iniciar câmera** na barra lateral.")
