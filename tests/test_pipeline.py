"""
Teste de integração da corrente visão -> braço.

Cobre o acoplamento que os testes unitários não pegam: uma bounding box em
pixels entra, e comandos SET saem pela serial, passando por homografia,
portão de estabilidade, cinemática inversa e sequência de coleta.

Não usa câmera nem YOLO — esses são os dois elos que precisam de hardware e
de peso de modelo. Tudo abaixo deles é exercitado de verdade, com uma
homografia sintética de escala e translação conhecidas.

    python tests/test_pipeline.py
"""

import sys
from pathlib import Path

import numpy as np

BASE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(BASE))

from src.arm.coleta import ControladorColeta, ParametrosColeta  # noqa: E402
from src.arm.kinematics import PoseInalcancavel, carregar_config  # noqa: E402
from src.vision.estabilidade import (  # noqa: E402
    ParametrosEstabilidade,
    PortaoEstabilidade,
)
from src.vision.homografia import Homografia  # noqa: E402

falhas = []


def checa(condicao, descricao):
    if not condicao:
        falhas.append(descricao)


class SerialEspiã:
    """
    Registra os comandos e modela o tempo que o firmware levaria para
    executá-los, para que o teste consiga detectar etapa cortada pela metade.

    `aguardar_parada` é o que o ControladorColeta chama entre etapas; aqui ela
    só adianta o relógio simulado, sem dormir de verdade. Se o controlador
    deixar de chamá-la, `parado` fica False no fim da etapa e o teste acusa.
    """

    VELOCIDADE = 60.0        # °/s, o padrão do firmware
    DURACAO_MIN = 0.08

    def __init__(self):
        self.linhas = []
        self.pos = {0: 90.0, 1: 90.0, 2: 90.0, 3: 90.0}
        self.t = 0.0                  # relógio simulado, em segundos
        self.t_fim_movimento = 0.0    # quando o movimento em curso termina

    @property
    def parado(self) -> bool:
        return self.t >= self.t_fim_movimento - 1e-9

    @property
    def pendente_s(self) -> float:
        return max(0.0, self.t_fim_movimento - self.t)

    def write(self, dados: bytes) -> int:
        linha = dados.decode().strip()
        if not linha:
            return len(dados)
        self.linhas.append(linha)

        if linha.startswith("SET "):
            _, canal, angulo = linha.split()
            canal, angulo = int(canal), float(angulo)
            dur = max(abs(angulo - self.pos[canal]) / self.VELOCIDADE, self.DURACAO_MIN)
            self.pos[canal] = angulo
            # Comandos do mesmo lote compartilham a duração: fica valendo o fim
            # mais distante, como o lote do firmware faz.
            self.t_fim_movimento = max(self.t_fim_movimento, self.t + dur)
        elif linha == "HOME":
            dur = max(
                (max(abs(90.0 - a) for a in self.pos.values()) / self.VELOCIDADE),
                self.DURACAO_MIN,
            )
            self.pos = {c: 90.0 for c in self.pos}
            self.t_fim_movimento = max(self.t_fim_movimento, self.t + dur)

        return len(dados)

    def aguardar_parada(self, timeout_s: float = 15.0) -> bool:
        self.t = max(self.t, self.t_fim_movimento)
        return True


class RelogioFalso:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def avanca(self, s):
        self.t += s


# --- homografia sintética: 4 pontos de uma mesa vista de cima -------------
# Escolhidos para dar uma correspondência simples de conferir: a área de
# 640x480 pixels vira uma janela de 200x150 mm à frente da base do braço.
PIXEIS = [(0, 0), (640, 0), (640, 480), (0, 480)]
MILIMETROS = [(-100, 150), (100, 150), (100, 0), (-100, 0)]

homografia = Homografia.de_pontos(PIXEIS, MILIMETROS)
residual = homografia.meta["erro_residual_mm"]
checa(residual < 0.01, f"homografia sintetica deveria ser exata, residual {residual}")


# --- a conversão de caixa usa a base, não o centro ------------------------
# Uma caixa alta e uma baixa com a MESMA base têm que dar a mesma coordenada.
alta = homografia.caixa_para_mm(300, 100, 340, 400)
baixa = homografia.caixa_para_mm(300, 380, 340, 400)
checa(abs(alta[0] - baixa[0]) < 0.01 and abs(alta[1] - baixa[1]) < 0.01,
      f"caixas com a mesma base deveriam dar a mesma coordenada: {alta} vs {baixa}")

# E uma caixa mais embaixo na imagem tem que estar mais perto da base do braço.
perto = homografia.caixa_para_mm(300, 400, 340, 460)
longe = homografia.caixa_para_mm(300, 40, 340, 100)
checa(perto[1] < longe[1],
      f"caixa mais baixa na imagem deveria ter y menor: {perto[1]} vs {longe[1]}")


# --- corrente completa ----------------------------------------------------
braco, calib, params_dict, CALIBRADO = carregar_config(BASE / "config" / "braco.json")
serial = SerialEspiã()
controlador = ControladorColeta(
    serial, braco=braco, calib=calib, params=ParametrosColeta(**params_dict)
)

relogio = RelogioFalso()
portao = PortaoEstabilidade(
    homografia,
    ParametrosEstabilidade(janela=5, tolerancia_mm=3.0),
    relogio=relogio,
)

# O alvo é procurado, não fixado: o braco.json muda quando vocês calibrarem,
# e um alvo escrito na mão hoje viraria falha falsa amanhã. Procura o ponto
# alcançável mais central dentro do campo que a homografia cobre.
def procura_alvo():
    bons = [
        (float(x), float(y))
        for x in range(-100, 101, 5)
        for y in range(0, 151, 5)
        if controlador.alcancavel(float(x), float(y))[0]
    ]
    if not bons:
        return None
    cx = sum(b[0] for b in bons) / len(bons)
    cy = sum(b[1] for b in bons) / len(bons)
    return min(bons, key=lambda b: (b[0] - cx) ** 2 + (b[1] - cy) ** 2)


alvo_mm = procura_alvo()

if alvo_mm is None:
    # Nenhum ponto alcançável. O que isso significa depende de a config já
    # estar calibrada ou não, e a diferença importa:
    #
    #   calibrado: false -> esperado. Os elos ainda são placeholder e o
    #     offset/sentido de cada junta ainda não foram medidos, então as
    #     faixas de ângulo matemático que os limites permitem não têm como
    #     coincidir com o que a cinemática pede. Não é defeito de código.
    #
    #   calibrado: true  -> defeito real. A área de coleta e o envelope do
    #     braço não se sobrepõem, e o sistema seria inútil assim.
    if CALIBRADO:
        falhas.append(
            "NENHUM ponto do campo da camera e alcancavel com a config marcada "
            "como calibrada. A area de coleta e o alcance do braco nao se "
            "sobrepoem. Confira elos_mm, offset/sentido e os limites com "
            "'python mapa_alcance.py'."
        )
        for f in falhas:
            print(f"  FALHA: {f}")
        raise SystemExit(1)

    for f in falhas:
        print(f"  FALHA: {f}")
    print("PENDENTE — nenhum alvo alcancavel, mas a config ainda esta marcada")
    print("  como nao calibrada. Faltam medir os elos e o offset/sentido de")
    print("  cada junta. Os testes acima (homografia e conversao de caixa)")
    print("  passaram; a corrente visao -> braco so pode ser exercitada depois.")
    raise SystemExit(1 if falhas else 0)

# mm -> pixel pela inversa da homografia, para o teste falar em mm.
inversa = np.linalg.inv(homografia.matriz)
ponto = inversa @ np.array([alvo_mm[0], alvo_mm[1], 1.0])
u, v = ponto[0] / ponto[2], ponto[1] / ponto[2]
checa(0 <= u <= 640 and 0 <= v <= 480,
      f"o alvo {alvo_mm} caiu fora da imagem: pixel ({u:.0f}, {v:.0f})")

# Cinco detecções com tremor de sub-milímetro: tem que estabilizar.
leitura = None
for i in range(5):
    tremor = [0.0, 0.4, -0.3, 0.2, -0.1][i]
    leitura = portao.observar(u - 20 + tremor, v - 40, u + 20 + tremor, v)
    relogio.avanca(0.16)

checa(leitura.estavel, f"deveria ter estabilizado: {leitura.motivo}")
checa(abs(leitura.x_mm - alvo_mm[0]) < 1.0 and abs(leitura.y_mm - alvo_mm[1]) < 1.0,
      f"coordenada deveria bater com o alvo {alvo_mm}, veio {leitura.coordenada}")

# A coordenada estabilizada vira sequência de comandos.
etapas = []
cortadas = []
for etapa in controlador.executar(leitura.x_mm, leitura.y_mm):
    etapas.append(etapa)
    # No ponto em que uma etapa termina, o braço tem que estar parado. Se não
    # estiver, a etapa seguinte comeca por cima do movimento em curso.
    if not serial.parado:
        cortadas.append((etapa.descricao, serial.pendente_s))

checa(len(etapas) == 11, f"a coleta deveria ter 11 etapas, tem {len(etapas)}")
checa(len(serial.linhas) == 25, f"deveriam sair 25 comandos, sairam {len(serial.linhas)}")
checa(all(l.startswith("SET ") or l == "HOME" for l in serial.linhas),
      "todo comando deveria ser SET ou HOME")
checa(serial.linhas[-1] == "HOME", "a sequencia deveria terminar em HOME")

# Nenhuma etapa pode comecar com a anterior ainda em movimento.
#
# Este e o teste que teria pego o bug que a suavizacao do firmware criou: a
# pausa fixa de 0.6s foi calibrada quando o SET era instantaneo, e virou curta
# demais quando o movimento passou a durar de 0.08s a 1.7s conforme a
# distancia. Cinco das oito etapas comecavam pela metade, e "fechar a garra"
# era interrompida aos 36% — o braco subia com a garra quase aberta.
if cortadas:
    pior = max(cortadas, key=lambda x: x[1])
    checa(False,
          f"{len(cortadas)} etapa(s) terminaram com o braco ainda andando; "
          f"pior: '{pior[0]}' faltando {pior[1]:.2f}s")

# Todo ângulo enviado tem que respeitar os limites da junta do braco.json.
for linha in serial.linhas:
    if not linha.startswith("SET "):
        continue
    _, canal, angulo = linha.split()
    canal, angulo = int(canal), float(angulo)
    servo = next(s for s in (calib.base, calib.ombro, calib.cotovelo, calib.garra)
                 if s.canal == canal)
    checa(servo.minimo <= angulo <= servo.maximo,
          f"'{linha}' viola o limite de {servo.nome} "
          f"[{servo.minimo:.0f}, {servo.maximo:.0f}]")


# --- objeto se movendo não dispara nada ----------------------------------
serial2 = SerialEspiã()
controlador2 = ControladorColeta(
    serial2, braco=braco, calib=calib, params=ParametrosColeta(**params_dict)
)
relogio2 = RelogioFalso()
portao2 = PortaoEstabilidade(
    homografia, ParametrosEstabilidade(janela=5, tolerancia_mm=3.0), relogio=relogio2
)

for i in range(15):
    desloca = i * 12          # ~12 px por deteccao, bem acima da tolerancia
    leit = portao2.observar(u - 20 + desloca, v - 40, u + 20 + desloca, v)
    if leit.estavel:
        controlador2.executar(leit.x_mm, leit.y_mm)
    relogio2.avanca(0.16)

checa(not serial2.linhas,
      f"objeto em movimento nao deveria gerar comando, gerou {len(serial2.linhas)}")


# --- angulos da garra: explicitos mandam, e sao validados ----------------
from src.arm.coleta import ParametrosColeta as PC  # noqa: E402

# Sem os angulos de garra: assim o primeiro caso exercita mesmo o fallback
# para o minimo/maximo da junta, independente do que a config traga hoje.
base_params = {k: v for k, v in params_dict.items()
               if k not in ("garra_aberta", "garra_fechada")}

# sem configurar, cai no minimo/maximo da junta (comportamento antigo)
s_pad = SerialEspiã()
c_pad = ControladorColeta(s_pad, braco=braco, calib=calib, params=PC(**base_params))
cmds = [e.comandos[0] for e in c_pad.planejar(*alvo_mm) if "garra" in e.descricao.lower()]
checa(cmds[0] == f"SET {calib.garra.canal} {calib.garra.minimo}",
      f"garra aberta padrao deveria ser o minimo da junta, veio {cmds[0]}")
checa(cmds[1] == f"SET {calib.garra.canal} {calib.garra.maximo}",
      f"garra fechada padrao deveria ser o maximo da junta, veio {cmds[1]}")

# configurados, os valores explicitos prevalecem
p_expl = dict(base_params, garra_aberta=125.0, garra_fechada=133.0)
c_expl = ControladorColeta(SerialEspiã(), braco=braco, calib=calib, params=PC(**p_expl))
cmds = [e.comandos[0] for e in c_expl.planejar(*alvo_mm) if "garra" in e.descricao.lower()]
checa(cmds[0] == f"SET {calib.garra.canal} 125.0", f"garra aberta explicita ignorada: {cmds[0]}")
checa(cmds[1] == f"SET {calib.garra.canal} 133.0", f"garra fechada explicita ignorada: {cmds[1]}")

# ordem contraria a da montagem atual (aberta > fechada) tambem funciona:
# nada no caminho assume qual extremo fecha. Na garra real e 100 aberta /
# 140 fechada, ou seja aberta < fechada, que ja e o contrario do intuitivo.
p_inv = dict(base_params, garra_aberta=135.0, garra_fechada=122.0)
c_inv = ControladorColeta(SerialEspiã(), braco=braco, calib=calib, params=PC(**p_inv))
cmds = [e.comandos[0] for e in c_inv.planejar(*alvo_mm) if "garra" in e.descricao.lower()]
checa(cmds[0] == f"SET {calib.garra.canal} 135.0" and cmds[1] == f"SET {calib.garra.canal} 122.0",
      f"garra montada ao contrario deveria funcionar, veio {cmds}")

# fora dos limites da junta: recusa antes de mover, e diz onde consertar
p_ruim = dict(base_params, garra_fechada=175.0)   # acima do maximo (130)
c_ruim = ControladorColeta(SerialEspiã(), braco=braco, calib=calib, params=PC(**p_ruim))
try:
    c_ruim.planejar(*alvo_mm)
    checa(False, "garra fora do limite deveria levantar PoseInalcancavel")
except PoseInalcancavel as e:
    checa("config" in str(e).lower(),
          f"a mensagem deveria apontar para a config, veio: {e}")


# --- entrada lateral: a garra nao pode descer sobre o chapeu da peca ------
#
# A peca tem um chapeu de 50 mm sobre um tronco de 20 mm, entao a sequencia
# desce AO LADO e entra de fora para dentro. Tres propriedades fazem isso
# funcionar, e as tres sao faceis de quebrar sem perceber ao mexer no
# planejar():
#
#   1. a descida acontece num raio MENOR que o do alvo, com folga de pelo
#      menos recuo_minimo (metade do chapeu);
#   2. a base nao se move na etapa de entrada. Se ela girar, a garra chega
#      de lado em vez de de frente, e os dedos batem na peca com a face
#      externa em vez de passarem por fora dela;
#   3. a entrada SOBE. Com a visao errando ate 2,2 mm, errar para baixo
#      empurra a peca contra a mesa; errar para cima so fecha a garra um
#      pouco acima do meio do tronco.
import math as _math

c_lat = ControladorColeta(SerialEspiã(), braco=braco, calib=calib, params=PC(**base_params))
etapas_lat = c_lat.planejar(0.0, 160.0)

def _angulos(etapa):
    """{canal: angulo} dos comandos SET de uma etapa."""
    saida = {}
    for cmd in etapa.comandos:
        partes = cmd.split()
        if partes[0] == "SET":
            saida[int(partes[1])] = float(partes[2])
    return saida

descida = next(e for e in etapas_lat if "ao lado" in e.descricao)
entrada = next(e for e in etapas_lat if "tronco" in e.descricao)

a_descida, a_entrada = _angulos(descida), _angulos(entrada)
checa(a_descida[calib.base.canal] == a_entrada[calib.base.canal],
      f"a base nao pode girar na entrada: {a_descida[calib.base.canal]} -> "
      f"{a_entrada[calib.base.canal]}")

# a descida fica mais perto do eixo, e por pelo menos o recuo minimo.
# Volta de angulo de servo para angulo matematico e roda a cinematica
# direta: e a unica forma de testar o PLANO, e nao a aritmetica que o gerou.
def _ponto(angulos):
    def mat(servo):
        return (angulos[servo.canal] - servo.offset) / servo.sentido
    ombro = mat(calib.ombro)
    cotovelo = mat(calib.cotovelo)
    # Num braco de paralelogramo o servo do cotovelo manda no angulo ABSOLUTO
    # do antebraco, enquanto a cinematica direta quer o relativo ao braco.
    # Esquecer de descontar o ombro aqui faz o teste calcular um ponto que
    # existe, mas nao e o planejado - e o erro cresce com a altura do ombro,
    # entao parece aleatorio.
    if calib.acoplamento_cotovelo == "absoluto":
        cotovelo -= ombro
    return braco.cinematica_direta(mat(calib.base), ombro, cotovelo)

pose_descida = _ponto(a_descida)
pose_entrada = _ponto(a_entrada)
r_descida = _math.hypot(pose_descida[0], pose_descida[1])
r_entrada = _math.hypot(pose_entrada[0], pose_entrada[1])
checa(r_entrada - r_descida >= base_params["recuo_minimo"] - 1.0,
      f"a descida deveria ficar ao menos {base_params['recuo_minimo']:.0f} mm "
      f"antes do alvo, ficou {r_entrada - r_descida:.1f} mm")
checa(pose_entrada[2] > pose_descida[2],
      f"a entrada tem que SUBIR: desceu a {pose_descida[2]:.1f} mm e entrou a "
      f"{pose_entrada[2]:.1f} mm")

# --- correcao radial do ponto da visao -----------------------------------
#
# A homografia entrega a base da bounding box, que numa vista obliqua e a
# borda do objeto virada para a camera. O centro do apoio, que e onde a garra
# fecha, fica mais longe no raio. Medido em 09/11/2026 com um mouse: relatado
# (-33, 137), borda a regua (-32,5 , 130), centro (-30, 150) - erro de 7,0 mm
# da borda e 13,3 mm do centro, quase todo em Y.
#
# Duas propriedades aqui: a correcao e RADIAL (nao muda o angulo da base) e
# AFASTA da base. Trocar o sinal por engano puxaria a garra para dentro da
# peca, que e o oposto do que se quer.
p_desloc = dict(base_params, deslocamento_radial=12.0)
c_desloc = ControladorColeta(SerialEspiã(), braco=braco, calib=calib, params=PC(**p_desloc))
bruto = (-33.0, 137.0)
corrigido = c_desloc.ponto_de_pega(*bruto)

r_bruto = _math.hypot(*bruto)
r_corr = _math.hypot(*corrigido)
checa(abs((r_corr - r_bruto) - 12.0) < 0.01,
      f"a correcao deveria somar 12 mm no raio, somou {r_corr - r_bruto:.2f}")
checa(abs(_math.atan2(*bruto[::-1]) - _math.atan2(*corrigido[::-1])) < 1e-9,
      "a correcao nao pode mudar o angulo da base: ela e radial")

# com deslocamento 0 nada muda, nem por arredondamento. O zero e EXPLICITO:
# base_params vem do braco.json de verdade, onde o valor e 25 mm desde que a
# peca foi medida, e herdar dali faria este teste nao testar nada.
p_zero = dict(base_params, deslocamento_radial=0.0)
c_zero = ControladorColeta(SerialEspiã(), braco=braco, calib=calib, params=PC(**p_zero))
checa(c_zero.ponto_de_pega(*bruto) == bruto,
      "sem deslocamento configurado o ponto tem que passar intacto")

# e a sequencia inteira tem de sair do ponto CORRIGIDO
etapas_d = c_desloc.planejar(*bruto)
entrada_d = next(e for e in etapas_d if "tronco" in e.descricao)
ponto_entrada = _ponto(_angulos(entrada_d))
checa(abs(_math.hypot(ponto_entrada[0], ponto_entrada[1]) - r_corr) < 1.0,
      f"a entrada deveria mirar o raio corrigido {r_corr:.1f}, mirou "
      f"{_math.hypot(ponto_entrada[0], ponto_entrada[1]):.1f}")


# --- soltar as juntas enquanto a garra age -------------------------------
#
# Com soltar_na_pega, o PWM da base, ombro e cotovelo e cortado enquanto a
# garra abre ou fecha, para ela ter a fonte inteira. Medido em 09/11 que o
# braco nao cai solto: quem segura e o atrito do redutor.
#
# Duas invariantes, e quebrar qualquer uma derruba o braco no meio da
# sequencia com a peca na garra:
#   1. nenhuma junta do braco pode receber SET enquanto esta solta;
#   2. a sequencia nao pode TERMINAR com alguma junta solta.
p_solta = dict(base_params, soltar_na_pega=True)
c_solta = ControladorColeta(SerialEspiã(), braco=braco, calib=calib, params=PC(**p_solta))

canais_braco = {calib.base.canal, calib.ombro.canal, calib.cotovelo.canal}
ligado = {c: True for c in canais_braco}
viu_detach = False

for etapa in c_solta.planejar(*alvo_mm):
    for cmd in etapa.comandos:
        partes = cmd.split()
        if partes[0] == "DETACH":
            ligado[int(partes[1])] = False
            viu_detach = True
        elif partes[0] == "ATTACH":
            ligado[int(partes[1])] = True
        elif partes[0] == "SET":
            canal = int(partes[1])
            if canal in canais_braco and not ligado[canal]:
                checa(False, f"SET no canal {canal} com ele solto, na etapa "
                             f"{etapa.descricao!r}")
        elif partes[0] == "HOME":
            # HOME move TODAS as juntas: nenhuma pode estar solta aqui
            soltas = [c for c, lig in ligado.items() if not lig]
            checa(not soltas, f"HOME com os canais {soltas} soltos")

checa(viu_detach, "soltar_na_pega ligado nao gerou DETACH nenhum")
checa(all(ligado.values()),
      f"a sequencia terminou com juntas soltas: "
      f"{[c for c, lig in ligado.items() if not lig]}")

# desligado (o padrao) nao gera nenhum dos dois
c_presa = ControladorColeta(SerialEspiã(), braco=braco, calib=calib,
                            params=PC(**dict(base_params, soltar_na_pega=False)))
todos = [cmd for e in c_presa.planejar(*alvo_mm) for cmd in e.comandos]
checa(not [c for c in todos if c.startswith(("DETACH", "ATTACH"))],
      "soltar_na_pega desligado nao deveria gerar DETACH/ATTACH")


# --- girar a base recolhido, e sozinha ------------------------------------
#
# Girar esticado e o pior caso do sistema: ombro no maior torque, base com a
# maior inercia, tres juntas andando juntas e a garra segurando. No primeiro
# ciclo real, em 09/11, a base travava no meio e a sequencia seguia soltando
# a peca ali - o firmware conta o TEMPO do movimento, nao a posicao.
#
# Duas propriedades que fazem a correcao valer:
#   1. na etapa do giro, SO a base se move;
#   2. ela gira num raio menor que o do alvo (recolhida).
etapas_t = controlador.planejar(*alvo_mm)
giro = next(e for e in etapas_t if "irando a base" in e.descricao)
recolhe = next(e for e in etapas_t if "ecolhendo" in e.descricao)

ang_giro = _angulos(giro)
ang_recolhe = _angulos(recolhe)

moveu = [c for c in ang_giro
         if c in ang_recolhe and abs(ang_giro[c] - ang_recolhe[c]) > 0.5]
checa(moveu == [calib.base.canal],
      f"no giro so a base (canal {calib.base.canal}) pode se mover, moveram {moveu}")

r_recolhe = _math.hypot(*_ponto(ang_recolhe)[:2])
r_alvo = _math.hypot(*_ponto(_angulos(
    next(e for e in etapas_t if "tronco" in e.descricao)))[:2])
checa(r_recolhe < r_alvo,
      f"deveria recolher antes de girar: raio {r_recolhe:.0f} contra "
      f"{r_alvo:.0f} do alvo")

# com raio_giro None volta ao movimento unico, para poder comparar
c_direto = ControladorColeta(SerialEspiã(), braco=braco, calib=calib,
                             params=PC(**dict(base_params, raio_giro=None)))
checa(not [e for e in c_direto.planejar(*alvo_mm) if "irando a base" in e.descricao],
      "raio_giro None deveria voltar ao movimento unico ate o deposito")


# alvo que nao aceita o recuo minimo e recusado, e a mensagem diz por que
p_colado = dict(base_params, recuo_entrada=90.0, recuo_minimo=90.0)
c_colado = ControladorColeta(SerialEspiã(), braco=braco, calib=calib, params=PC(**p_colado))
try:
    c_colado.planejar(0.0, 160.0)
    checa(False, "recuo minimo inalcancavel deveria levantar PoseInalcancavel")
except PoseInalcancavel as e:
    checa(e.motivo == PoseInalcancavel.ENVELOPE,
          f"motivo deveria ser ENVELOPE, veio {e.motivo}")
    checa("chapéu" in str(e) or "chapeu" in str(e),
          f"a mensagem deveria explicar por que nao recua menos, veio: {e}")

for f in falhas:
    print(f"  FALHA: {f}")
print("OK — a corrente visao -> braco esta ligada" if not falhas
      else f"\n{len(falhas)} falha(s)")
raise SystemExit(1 if falhas else 0)
