"""
Testes do firmware sem Arduino na mesa.

O firmware é C e roda no Nano, então não dá para importá-lo aqui. O que este
arquivo faz é reimplementar em Python as duas partes que mais quebram em
silêncio, e exercitá-las:

1. a cadeia de if/else que despacha os comandos, junto com a tokenização e as
   conversões atoi/atof que ela usa — é aí que um comando engole o outro
   (SET/SPEED/SWEEP/STOP começam todos com S, e GET colide com GET ALL);

2. a curva de suavização do movimento, para confirmar que a velocidade
   realmente nasce e morre em zero. Se ela não fizer isso, o servo dá tranco,
   que é o defeito que a curva existe para resolver.

Isto NÃO substitui compilar e carregar. Erro de sintaxe, biblioteca faltando e
problema de I2C não aparecem aqui — e, mais importante, DIFERENÇAS DA
BIBLIOTECA DO ALVO também não. A primeira versão deste arquivo modelava o
sscanf de C padrão e passava com 100%, enquanto o firmware respondia
ERR UNKNOWN no hardware, porque o avr-libc linka um scanf sem ponto flutuante.
É teste de lógica, não de integração, e a lógica pode estar certa contra a
plataforma errada.

    python tests/test_firmware.py
"""

import re
import sys
from pathlib import Path

# Precisam bater com os #define do firmware.
NUM_SERVOS = 4
INTERVALO_PASSO_MS = 20
DURACAO_MIN_MS = 80
JANELA_LOTE_MS = 50

# Resolução do PCA9685 na faixa 102-492: 390 contagens para 180 graus.
GRAUS_POR_CONTAGEM = 180.0 / 390.0
SPEED_MAX = 150
VEL_GARRA_MAX = 1000


# ---------------------------------------------------------------------------
# 1. Dispatch de comandos
# ---------------------------------------------------------------------------

def divide_em_tokens(texto, maximo=4):
    """
    Espelha divideEmTokens() do firmware: separa por espaço ou tabulação,
    no máximo `maximo` tokens.
    """
    tok, i, n = [], 0, len(texto)
    while i < n and len(tok) < maximo:
        while i < n and texto[i] in " 	":
            i += 1
        if i >= n:
            break
        j = i
        while j < n and texto[j] not in " 	":
            j += 1
        tok.append(texto[i:j])
        i = j
    return tok


def atoi(t):
    """atoi() do C: lê o inteiro do começo, 0 se não houver nenhum."""
    m = re.match(r"\s*[+-]?\d+", t)
    return int(m.group(0)) if m else 0


def atof(t):
    """atof() do C: lê o float do começo, 0.0 se não houver nenhum."""
    m = re.match(r"\s*[+-]?(\d+\.?\d*|\.\d+)([eE][+-]?\d+)?", t)
    return float(m.group(0)) if m else 0.0


def dispatch(texto):
    """
    Espelha processaComando(), devolvendo o nome do ramo que casou.

    NOTA HISTÓRICA: a primeira versão do firmware usava sscanf com "%d %f", e
    esta função a reproduzia com a semântica do sscanf de C padrão. No AVR isso
    não vale: a biblioteca do Arduino linka um scanf sem suporte a ponto
    flutuante, então TODOS os comandos com argumento decimal falhavam em
    silêncio e respondiam ERR UNKNOWN. O teste passava e o hardware não
    funcionava. O parser foi reescrito com tokenização mais atoi/atof, e este
    modelo acompanha.
    """
    tok = divide_em_tokens(texto.upper())
    n = len(tok)
    if n == 0:
        return None

    cmd = tok[0]

    def id_ok(i):
        return 0 <= i < NUM_SERVOS

    if cmd == "SET" and n == 3:
        return "SET" if id_ok(atoi(tok[1])) else "ERR BAD_ARGS"
    elif cmd == "GET" and n == 2 and tok[1] == "ALL":
        return "GET ALL"
    elif cmd == "GET" and n == 2:
        return "GET" if id_ok(atoi(tok[1])) else "ERR BAD_ARGS"
    elif cmd == "HOME" and n == 1:
        return "HOME"
    elif cmd == "STOP" and n == 1:
        return "STOP"
    elif cmd == "BUSY" and n == 1:
        return "BUSY"
    elif cmd == "SWEEP" and n == 4:
        return "SWEEP" if id_ok(atoi(tok[1])) else "ERR BAD_ARGS"
    elif cmd == "LIM" and n == 4:
        i, a, b = atoi(tok[1]), atof(tok[2]), atof(tok[3])
        return "ERR BAD_ARGS" if not id_ok(i) or a > b else "LIM set"
    elif cmd == "LIM" and n == 1:
        return "LIM show"
    elif cmd == "SPEED" and n == 1:
        return "SPEED show"
    elif cmd == "SPEED" and n == 2:
        a = atof(tok[1])
        return "ERR BAD_ARGS" if a <= 0 or a > SPEED_MAX else "SPEED"
    elif cmd == "SPEEDG" and n == 1:
        return "SPEEDG show"
    elif cmd == "SPEEDG" and n == 2:
        a = atof(tok[1])
        return "ERR BAD_ARGS" if a < 0 or a > VEL_GARRA_MAX else "SPEEDG"
    elif cmd == "DETACH" and n == 2 and tok[1] == "ALL":
        return "DETACH ALL"
    elif cmd == "DETACH" and n == 2:
        return "DETACH" if id_ok(atoi(tok[1])) else "ERR BAD_ARGS"
    elif cmd == "ATTACH" and n == 2 and tok[1] == "ALL":
        return "ATTACH ALL"
    elif cmd == "ATTACH" and n == 2:
        return "ATTACH" if id_ok(atoi(tok[1])) else "ERR BAD_ARGS"
    elif cmd == "RAW" and n == 3:
        i, pulso = atoi(tok[1]), atoi(tok[2])
        return "ERR BAD_ARGS" if not id_ok(i) or not (0 <= pulso <= 4095) else "RAW"
    elif cmd == "PULSO" and n == 4:
        i, pi, pj = atoi(tok[1]), atoi(tok[2]), atoi(tok[3])
        return "ERR BAD_ARGS" if not id_ok(i) or pi < 0 or pj > 4095 or pi >= pj else "PULSO"
    elif cmd == "AMP" and n == 1:
        return "AMP"
    elif cmd == "ZERO" and n == 1:
        return "ZERO"
    elif cmd == "SENS" and n == 2:
        s_ = atof(tok[1])
        return "ERR BAD_ARGS" if s_ < 1.0 or s_ > 1000.0 else "SENS"
    elif cmd == "LIMAMP" and n == 2:
        a = atof(tok[1])
        return "ERR BAD_ARGS" if a < 0.0 or a > 5.0 else "LIMAMP set"
    elif cmd == "LIMAMP" and n == 1:
        return "LIMAMP show"
    elif cmd == "HELP" and n == 1:
        return "HELP"
    else:
        return "ERR UNKNOWN"


CASOS_DISPATCH = [
    # operação
    ("SET 1 45", "SET"), ("SET 0 90.5", "SET"), ("set 2 100", "SET"),
    # Os quatro abaixo usam argumento DECIMAL. Eram exatamente os que o parser
    # com sscanf quebrava no AVR, respondendo ERR UNKNOWN.
    ("SET 1 45.7", "SET"), ("SWEEP 1 80.5 100.5", "SWEEP"),
    ("LIM 1 20.5 160.5", "LIM set"), ("SPEED 12.5", "SPEED"),
    ("SET 3 -10", "SET"),                       # negativo é limitado, não recusado
    ("GET 0", "GET"), ("GET ALL", "GET ALL"),
    ("HOME", "HOME"), ("STOP", "STOP"), ("BUSY", "BUSY"),

    # bancada
    ("SWEEP 1 80 100", "SWEEP"),
    ("LIM", "LIM show"), ("LIM 1 20 160", "LIM set"),
    ("SPEED 30", "SPEED"), ("SPEED 45.5", "SPEED"), ("SPEED 150", "SPEED"),
    ("DETACH ALL", "DETACH ALL"), ("DETACH 2", "DETACH"),
    ("ATTACH ALL", "ATTACH ALL"), ("ATTACH 2", "ATTACH"),
    ("RAW 0 300", "RAW"), ("PULSO 0 140 610", "PULSO"), ("HELP", "HELP"),

    # prefixos que poderiam se engolir: SET/SPEED/SWEEP/STOP, GET/GET ALL
    ("SPEED 60", "SPEED"), ("SWEEP 0 10 30", "SWEEP"),
    # SPEEDG comeca com SPEED: strcmp e exato, mas e exatamente o tipo de
    # colisao que ja mordeu este parser antes.
    ("SPEED", "SPEED show"), ("SPEEDG", "SPEEDG show"),
    ("SPEEDG 500", "SPEEDG"), ("SPEEDG 1000", "SPEEDG"),
    ("SPEEDG 0", "SPEEDG"),            # 0 = sem suavizacao, e o padrao
    ("SPEEDG -1", "ERR BAD_ARGS"), ("SPEEDG 1001", "ERR BAD_ARGS"),
    ("SPEEDG 200 3", "ERR UNKNOWN"), ("SPEEDGG 500", "ERR UNKNOWN"),
    # o teto da garra e maior que o do braco: 200 recusa num e aceita no outro
    ("SPEED 200", "ERR BAD_ARGS"), ("SPEEDG 200", "SPEEDG"),
    ("GET 3", "GET"), ("LIM 0 0 180", "LIM set"),
    ("DETACH 0", "DETACH"), ("ATTACH 0", "ATTACH"), ("PULSO 1 150 600", "PULSO"),

    # argumentos inválidos
    ("SET 9 45", "ERR BAD_ARGS"), ("SET 4 45", "ERR BAD_ARGS"),
    ("GET 9", "ERR BAD_ARGS"), ("SWEEP 7 10 20", "ERR BAD_ARGS"),
    ("LIM 0 170 10", "ERR BAD_ARGS"),           # min > max
    ("SPEED 0", "ERR BAD_ARGS"), ("SPEED 200", "ERR BAD_ARGS"),
    ("RAW 0 9999", "ERR BAD_ARGS"), ("PULSO 0 600 150", "ERR BAD_ARGS"),

    # lixo e argumentos faltando
    ("SET", "ERR UNKNOWN"), ("SET 1", "ERR UNKNOWN"), ("XYZ", "ERR UNKNOWN"),
    ("SETUP 1 2", "ERR UNKNOWN"), ("GETALL", "ERR UNKNOWN"),
    ("", None), ("   ", None),          # linha vazia: o firmware nao responde

    # o firmware come espaço das pontas antes de comparar
    ("  HOME", "HOME"), ("SET 1 45  ", "SET"), ("\tGET ALL ", "GET ALL"),

    # sensor de corrente
    ("AMP", "AMP"), ("ZERO", "ZERO"),
    ("SENS 185", "SENS"), ("SENS 100", "SENS"), ("SENS 66", "SENS"),
    ("LIMAMP 0.8", "LIMAMP set"), ("LIMAMP 0", "LIMAMP set"),   # 0 desliga
    ("LIMAMP", "LIMAMP show"),
    ("SENS 0", "ERR BAD_ARGS"), ("SENS 5000", "ERR BAD_ARGS"),
    ("LIMAMP -1", "ERR BAD_ARGS"), ("LIMAMP 9", "ERR BAD_ARGS"),
    ("AMP 3", "ERR UNKNOWN"),          # AMP nao aceita argumento
    ("SET 3 45", "SET"),               # SET continua funcionando na garra
]


# ---------------------------------------------------------------------------
# 2. Suavização do movimento
# ---------------------------------------------------------------------------

def suaviza(p):
    """smootherstep: 6p^5 - 15p^4 + 10p^3. Espelha suaviza() do firmware."""
    if p <= 0.0:
        return 0.0
    if p >= 1.0:
        return 1.0
    return p * p * p * (p * (p * 6.0 - 15.0) + 10.0)


class BracoSimulado:
    """Modelo de defineAlvo() + atualizaMovimento()."""

    def __init__(self, velocidade=60.0):
        self.atual = [90.0] * NUM_SERVOS
        self.alvo = [90.0] * NUM_SERVOS
        self.origem = [90.0] * NUM_SERVOS
        self.t_inicio = [0] * NUM_SERVOS
        self.duracao = [0] * NUM_SERVOS
        self.velocidade = velocidade
        self.t = 0                       # ms
        self.t_lote = -10 ** 9
        self.duracao_lote = 0
        self.no_lote = [False] * NUM_SERVOS

    def define_alvo(self, i, angulo):
        if self.t - self.t_lote > JANELA_LOTE_MS:
            self.t_lote = self.t
            self.duracao_lote = 0
            self.no_lote = [False] * NUM_SERVOS

        self.origem[i] = self.atual[i]
        self.alvo[i] = angulo
        self.no_lote[i] = True

        natural = int(abs(angulo - self.origem[i]) / self.velocidade * 1000)
        natural = max(natural, DURACAO_MIN_MS)
        self.duracao_lote = max(self.duracao_lote, natural)

        for k in range(NUM_SERVOS):
            if self.no_lote[k]:
                self.t_inicio[k] = self.t_lote
                self.duracao[k] = self.duracao_lote

    def passo(self):
        self.t += INTERVALO_PASSO_MS
        for i in range(NUM_SERVOS):
            if self.duracao[i] == 0:
                pos = self.alvo[i]
            else:
                decorrido = self.t - self.t_inicio[i]
                if decorrido >= self.duracao[i]:
                    pos = self.alvo[i]
                    self.duracao[i] = 0
                else:
                    p = decorrido / self.duracao[i]
                    pos = self.origem[i] + (self.alvo[i] - self.origem[i]) * suaviza(p)
                    # Gruda no alvo abaixo de meia contagem, sem encerrar o
                    # movimento: encerrar desincronizaria o lote.
                    if abs(self.alvo[i] - pos) < GRAUS_POR_CONTAGEM * 0.5:
                        pos = self.alvo[i]

            self.atual[i] = pos

    def perfil(self, i):
        """Velocidades instantâneas (°/s) até a junta i parar."""
        vels, anterior = [], self.atual[i]
        while self.duracao[i] != 0:
            self.passo()
            vels.append((self.atual[i] - anterior) / (INTERVALO_PASSO_MS / 1000.0))
            anterior = self.atual[i]
        return vels


def testa_movimento():
    """Devolve a lista de falhas encontradas."""
    falhas = []

    def checa(condicao, descricao):
        if not condicao:
            falhas.append(descricao)

    # --- a velocidade nasce e morre em zero -----------------------------
    b = BracoSimulado(60.0)
    b.define_alvo(1, 180.0)
    vels = b.perfil(1)

    checa(abs(vels[0]) < 1.0, f"velocidade inicial deveria ser ~0, veio {vels[0]:.2f}")
    checa(abs(vels[-1]) < 1.0, f"velocidade final deveria ser ~0, veio {vels[-1]:.2f}")

    media = sum(vels) / len(vels)
    checa(abs(media - 60.0) < 1.0, f"velocidade media deveria ser ~60, veio {media:.2f}")

    # A smootherstep tem pico exatamente 1,875x a média. Se essa razão mudar,
    # a curva foi trocada e o teto do SPEED precisa ser revisto junto.
    razao = max(vels) / media
    checa(abs(razao - 1.875) < 0.05, f"pico/media deveria ser ~1.875, veio {razao:.3f}")

    duracao_s = len(vels) * INTERVALO_PASSO_MS / 1000.0
    checa(abs(duracao_s - 1.5) < 0.05, f"90 graus a 60/s deveria durar 1.5 s, deu {duracao_s:.2f}")

    # --- a cauda da curva não vira ticks isolados -----------------------
    #
    # A smootherstep chega com velocidade zero, então os últimos graus saem
    # em passos cada vez menores. Abaixo de meia contagem de pulso eles
    # deixam de virar movimento: o braço "acerta aos poucos", em ticks
    # espaçados. O corte encerra ali, no mesmo destino.
    b = BracoSimulado(60.0)
    b.define_alvo(0, 180.0)
    while b.duracao[0] != 0:
        b.passo()
    checa(abs(b.atual[0] - 180.0) < 1e-9,
          f"deveria terminar exatamente no alvo, veio {b.atual[0]}")

    # O que importa não é quantas vezes o ângulo muda, e sim quantas
    # CONTAGENS DE PULSO distintas saem: é só isso que o servomotor vê.
    def conta(angulo):
        return round(102 + angulo / 180.0 * 390)

    b = BracoSimulado(10.0)          # velocidade baixa: é onde dói
    b.define_alvo(0, 135.0)
    contagens = []
    while b.duracao[0] != 0:
        b.passo()
        if abs(135.0 - b.atual[0]) < GRAUS_POR_CONTAGEM:
            c = conta(b.atual[0])
            if not contagens or c != contagens[-1]:
                contagens.append(c)
    checa(len(contagens) <= 2,
          f"na última contagem de curso deveriam sair no máximo 2 valores "
          f"de pulso, saíram {len(contagens)}: {contagens}")
    checa(contagens and contagens[-1] == conta(135.0),
          f"o último pulso escrito deveria ser o do alvo ({conta(135.0)}), "
          f"veio {contagens[-1] if contagens else None}")

    # --- movimento minúsculo ainda é suavizado --------------------------
    b = BracoSimulado(60.0)
    b.define_alvo(0, 91.0)          # 1 grau: natural daria 16 ms, o piso e 80
    checa(b.duracao[0] == DURACAO_MIN_MS,
          f"movimento de 1 grau deveria durar {DURACAO_MIN_MS} ms, deu {b.duracao[0]}")

    # --- juntas disparadas juntas chegam juntas -------------------------
    b = BracoSimulado(60.0)
    b.define_alvo(0, 120.0)         # 30 graus
    b.define_alvo(1, 150.0)         # 60 graus
    b.define_alvo(2, 95.0)          #  5 graus
    checa(b.duracao[0] == b.duracao[1] == b.duracao[2],
          f"lote deveria ter duracao unica, veio {b.duracao[:3]}")

    chegada = [None] * 3
    while any(b.duracao[i] != 0 for i in range(3)):
        b.passo()
        for i in range(3):
            if b.duracao[i] == 0 and chegada[i] is None:
                chegada[i] = b.t
    checa(chegada[0] == chegada[1] == chegada[2],
          f"juntas do mesmo lote deveriam chegar juntas, chegaram em {chegada}")

    # --- SET no meio do movimento não salta de posição ------------------
    b = BracoSimulado(60.0)
    b.define_alvo(1, 180.0)
    for _ in range(20):             # 400 ms de movimento
        b.passo()
    meio = b.atual[1]
    b.define_alvo(1, 100.0)         # muda de ideia no meio do caminho
    checa(abs(b.origem[1] - meio) < 0.01,
          "um SET novo deveria partir da posicao atual, nao do alvo antigo")
    b.passo()
    checa(abs(b.atual[1] - meio) < 2.0,
          f"a posicao saltou {abs(b.atual[1]-meio):.1f} graus ao mudar de alvo")

    # --- sweep inverte com o servo parado -------------------------------
    b = BracoSimulado(30.0)
    b.define_alvo(1, 80.0)
    de, ate = 80.0, 100.0
    vels_inversao = []
    for _ in range(int(8000 / INTERVALO_PASSO_MS)):
        anterior = b.atual[1]
        b.passo()
        if b.duracao[1] == 0:
            vels_inversao.append(abs((b.atual[1] - anterior) / (INTERVALO_PASSO_MS / 1000.0)))
            b.define_alvo(1, ate if abs(b.alvo[1] - de) < 0.05 else de)

    checa(len(vels_inversao) >= 4, "o sweep deveria ter invertido varias vezes")
    checa(max(vels_inversao) < 2.0,
          f"o sweep deveria inverter parado, veio {max(vels_inversao):.2f} graus/s")

    return falhas


# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# 3. Sensor de corrente e alívio de travamento
# ---------------------------------------------------------------------------

# Precisam bater com o firmware.
GRACA_STALL_MS = 300
INTERVALO_ALIVIO_MS = 50
PASSO_ALIVIO = 1.0
ALIVIO_MAX = 15.0
LIM_MIN_GARRA, LIM_MAX_GARRA = 30.0, 130.0


def contagens_por_amp(mv_por_amp=185.0):
    """Espelha contagensPorAmp(): ratiométrico, VCC cancela."""
    return mv_por_amp * 1023.0 / 5000.0


class GarraComSensor:
    """
    Modelo de verificaStall() para a garra, com corrente injetada pelo teste.

    Não modela o servo físico: o teste dita a corrente a cada passo e observa
    o que o firmware faz com o ângulo comandado e com as mensagens.
    """

    def __init__(self, limite_amp=0.8, atual=90.0, origem=90.0):
        self.limite = limite_amp
        self.atual = atual
        self.alvo = atual
        self.origem = origem
        self.t = 0
        self.t_inicio = 0
        self.ligado = True
        self.em_alivio = False
        self.acumulado = 0.0
        self.direcao = 0.0
        self.t_ultimo_alivio = 0
        self.desistiu = False
        self.msgs = []

    def limita(self, a):
        return max(LIM_MIN_GARRA, min(LIM_MAX_GARRA, a))

    def passo(self, corrente, dt_ms=5):
        self.t += dt_ms
        if self.limite <= 0 or not self.ligado:
            return
        corrente = abs(corrente)

        if self.em_alivio:
            if corrente < self.limite * 0.8:
                self.em_alivio = False
                self.msgs.append(("OK ALIVIO", round(self.atual, 1)))
                return
            if self.acumulado >= ALIVIO_MAX:
                self.em_alivio = False
                self.desistiu = True
                self.msgs.append(("ERR STALL_PERSISTE", round(corrente, 2)))
                return
            if self.t - self.t_ultimo_alivio >= INTERVALO_ALIVIO_MS:
                self.t_ultimo_alivio = self.t
                novo = self.limita(self.atual + self.direcao * PASSO_ALIVIO)
                self.atual = self.alvo = self.origem = novo
                self.acumulado += PASSO_ALIVIO
            return

        if corrente <= self.limite:
            self.desistiu = False
            return
        if self.desistiu:
            return
        if self.t - self.t_inicio < GRACA_STALL_MS:
            return

        self.em_alivio = True
        self.acumulado = 0.0
        self.alvo = self.atual
        de_onde = self.origem - self.atual
        if abs(de_onde) > 0.5:
            self.direcao = 1.0 if de_onde > 0 else -1.0
        else:
            meio = (LIM_MIN_GARRA + LIM_MAX_GARRA) / 2.0
            self.direcao = 1.0 if meio > self.atual else -1.0
        self.origem = self.atual
        self.t_ultimo_alivio = self.t
        self.msgs.append(("STALL", round(corrente, 2)))


def testa_sensor():
    falhas = []

    def checa(cond, desc):
        if not cond:
            falhas.append(desc)

    # --- conversão ratiométrica ----------------------------------------
    cpa = contagens_por_amp(185.0)
    checa(abs(cpa - 37.85) < 0.05, f"185 mV/A deveria dar ~37.85 cont/A, deu {cpa:.2f}")
    corrente = (549.5 - 511.5) / cpa          # +38 contagens acima do zero
    checa(abs(corrente - 1.0) < 0.02, f"+38 contagens deveria ser ~1.0 A, deu {corrente:.3f}")

    # --- desligado (limite 0) nao faz nada, mesmo com corrente alta -----
    g = GarraComSensor(limite_amp=0.0)
    g.t_inicio = -10_000
    for _ in range(200):
        g.passo(3.0)
    checa(not g.msgs and g.atual == 90.0, "com LIMAMP 0 nada deveria acontecer")

    # --- abaixo do limite nao dispara --------------------------------------
    g = GarraComSensor(limite_amp=0.8)
    g.t_inicio = -10_000
    for _ in range(200):
        g.passo(0.5)
    checa(not g.msgs, "corrente abaixo do limite nao deveria disparar")

    # --- pico de partida dentro da graca e ignorado ------------------------
    g = GarraComSensor(limite_amp=0.8)
    g.t_inicio = 0
    for _ in range(50):                        # 250 ms, dentro dos 300 de graca
        g.passo(2.0)
    checa(not g.msgs, "pico dentro da janela de graca nao deveria disparar")

    # --- travou fechando: recua no sentido de onde veio (abrindo) ----------
    # fechando = indo de 60 para 120; travou em 110 -> deve recuar para menos
    g = GarraComSensor(limite_amp=0.8, atual=110.0, origem=60.0)
    g.t_inicio = -10_000
    g.passo(1.5)
    checa(bool(g.msgs) and g.msgs[0][0] == "STALL", "deveria ter reportado STALL")
    checa(g.direcao == -1.0, f"fechando de 60->120, alivio deveria recuar (-1), deu {g.direcao}")
    # corrente continua alta por 3 passos de alivio, depois cai
    for _ in range(int(3 * INTERVALO_ALIVIO_MS / 5)):
        g.passo(1.5)
    checa(abs(g.atual - 107.0) < 0.01, f"apos 3 passos deveria estar em 107, esta em {g.atual}")
    g.passo(0.3)
    checa(g.msgs[-1][0] == "OK ALIVIO", f"corrente caiu, deveria ter concluido; msgs={g.msgs}")
    checa(not g.em_alivio, "deveria ter saido do modo alivio")
    checa(g.alvo == g.atual, "apos alivio, alvo deve ser igual a atual (segura onde parou)")

    # --- histerese ---------------------------------------------------------
    g = GarraComSensor(limite_amp=1.0, atual=100.0, origem=60.0)
    g.t_inicio = -10_000
    g.passo(1.5)
    g.passo(0.9)                                # acima de 0.8: continua aliviando
    checa(g.em_alivio, "0.9 A com limite 1.0 nao deveria concluir (histerese 0.8)")
    g.passo(0.7)
    checa(not g.em_alivio, "0.7 A com limite 1.0 deveria concluir")

    # --- travou parada (segurando): recua para o meio do curso -------------
    g = GarraComSensor(limite_amp=0.8, atual=120.0, origem=120.0)
    g.t_inicio = -10_000
    g.passo(1.5)
    checa(g.direcao == -1.0, f"parada em 120 (meio=80), deveria recuar para baixo, deu {g.direcao}")
    g2 = GarraComSensor(limite_amp=0.8, atual=40.0, origem=40.0)
    g2.t_inicio = -10_000
    g2.passo(1.5)
    checa(g2.direcao == 1.0, f"parada em 40 (meio=80), deveria recuar para cima, deu {g2.direcao}")

    # --- nunca cai: desiste em 15 graus com erro ---------------------------
    g = GarraComSensor(limite_amp=0.8, atual=110.0, origem=60.0)
    g.t_inicio = -10_000
    for _ in range(int(20 * INTERVALO_ALIVIO_MS / 5) + 10):
        g.passo(2.0)
    checa(g.msgs[-1][0] == "ERR STALL_PERSISTE",
          f"corrente que nunca cai deveria terminar em STALL_PERSISTE, msgs={g.msgs[-2:]}")
    checa(abs(g.atual - 95.0) < 0.01, f"deveria ter recuado exatamente 15 graus (110->95), esta em {g.atual}")
    checa(not g.em_alivio, "apos desistir, nao deveria continuar em alivio")

    # --- depois de desistir, nao reentra enquanto a corrente nao cair ------
    g = GarraComSensor(limite_amp=0.8, atual=110.0, origem=60.0)
    g.t_inicio = -10_000
    for _ in range(int(20 * INTERVALO_ALIVIO_MS / 5) + 10):
        g.passo(2.0)
    n_msgs = len(g.msgs)
    for _ in range(400):                       # mais 2 s de corrente alta
        g.passo(2.0)
    checa(len(g.msgs) == n_msgs, f"apos desistir nao deveria reentrar; msgs novas: {g.msgs[n_msgs:]}")
    checa(abs(g.atual - 95.0) < 0.01, f"apos desistir deveria ficar parada em 95, foi a {g.atual}")
    g.passo(0.3)                               # corrente normaliza
    g.passo(2.0)                               # e sobe de novo: agora pode vigiar
    checa(g.msgs[-1][0] == "STALL", "apos a corrente cair, uma nova subida deveria disparar de novo")

    # --- alivio respeita o limite da junta ---------------------------------
    g = GarraComSensor(limite_amp=0.8, atual=35.0, origem=60.0)   # perto do minimo 30
    g.t_inicio = -10_000
    for _ in range(int(20 * INTERVALO_ALIVIO_MS / 5) + 10):
        g.passo(2.0)
    checa(g.atual >= LIM_MIN_GARRA, f"alivio nao pode passar do limite da junta, foi a {g.atual}")

    return falhas


# ---------------------------------------------------------------------------
# Tabela da sequencia DEMO
# ---------------------------------------------------------------------------
# Esta nao e uma reimplementacao: le a tabela do .ino de verdade e confere as
# invariantes dela. Serve porque a tabela e editada a mao, e as duas maneiras
# de errar nao doem na compilacao:
#
#   - garra junto com outra junta no mesmo passo: as duas caem no mesmo lote
#     e dividem a duracao do movimento mais longo. A garra fica abaixo da zona
#     morta e trava, parecendo que o comando nao chegou.
#   - angulo fora de lim_min/lim_max: o firmware satura em silencio e o passo
#     simplesmente nao vai aonde o roteiro diz.

RAIZ = Path(__file__).resolve().parent.parent
INO = RAIZ / "firmware" / "braco_firmware" / "braco_firmware.ino"


def _extrai_vetor(fonte, nome):
    corpo = re.search(rf"{nome}\[NUM_SERVOS\]\s*=\s*\{{([^}}]*)\}}", fonte).group(1)
    return [float(x) for x in re.findall(r"-?\d+\.?\d*", corpo)]


def _extrai_demo(fonte):
    corpo = re.search(r"demo_passos\[\]\[NUM_SERVOS\]\s*PROGMEM\s*=\s*\{(.*?)\n\};",
                      fonte, re.S).group(1)
    passos = []
    for linha in corpo.splitlines():
        linha = linha.split("//")[0]
        m = re.search(r"\{([^}]*)\}", linha)
        if m:
            passos.append([int(x) for x in re.findall(r"-?\d+", m.group(1))])
    return passos


def testa_demo():
    falhas = []
    fonte = INO.read_text(encoding="utf-8")
    lo = _extrai_vetor(fonte, "lim_min")
    hi = _extrai_vetor(fonte, "lim_max")
    passos = _extrai_demo(fonte)
    idx_garra = int(re.search(r"IDX_GARRA\s*=\s*(\d+)", fonte).group(1))

    if not passos:
        return ["nao consegui ler demo_passos do .ino"]

    for k, passo in enumerate(passos, start=1):
        if len(passo) != len(lo):
            falhas.append(f"passo {k}: {len(passo)} angulos para {len(lo)} juntas")
            continue
        moveis = [i for i, a in enumerate(passo) if a >= 0]
        if not moveis:
            falhas.append(f"passo {k}: nao move nada")
        if idx_garra in moveis and len(moveis) > 1:
            falhas.append(f"passo {k}: garra junto com outra junta (dividem a duracao do lote)")
        for i in moveis:
            if not lo[i] <= passo[i] <= hi[i]:
                falhas.append(f"passo {k}: junta {i} em {passo[i]}, fora de [{lo[i]:.0f}, {hi[i]:.0f}]")

    # Divergencia com o pegada_manual.py nao e falha: os dois existem para
    # serem ajustados, e o fluxo e ajustar no Python (rapido) e so depois
    # passar para o firmware. Mas vale avisar, senao o video mostra um
    # roteiro e o script mostra outro.
    try:
        sys.path.insert(0, str(RAIZ))
        import pegada_manual
        py = [{c: a for c, a in alvos.items()} for _, alvos in pegada_manual.SEQUENCIA]
        ino = [{i: a for i, a in enumerate(p) if a >= 0} for p in passos]
        if py != ino:
            print("  AVISO: a sequencia do pegada_manual.py e a tabela DEMO do .ino")
            print("         estao diferentes. Ajuste no Python e passe para o firmware.")
    except Exception as e:
        print(f"  AVISO: nao deu para comparar com pegada_manual.py ({e})")

    return falhas


def testa_limites_sincronizados():
    """
    Os limites de junta vivem em tres arquivos. Aqui so se compara.

    A FONTE e o .ino, decidido em 05/11: foi com aqueles valores que o braco
    foi operado na bancada. O braco.json e o pegada_manual.py copiam de la.

    Isto e FALHA e nao aviso porque o modo de errar e silencioso: o
    coletar.py empurra os limites do JSON por LIM ao conectar, entao um JSON
    desatualizado AFROUXA na conexao um limite que a bancada mostrou ser
    necessario, e so se descobre quando o braco bate. Foi assim em 31/10.
    """
    import json

    fonte = INO.read_text(encoding="utf-8")
    ino_lo = _extrai_vetor(fonte, "lim_min")
    ino_hi = _extrai_vetor(fonte, "lim_max")

    cfg = json.loads((RAIZ / "config" / "braco.json").read_text(encoding="utf-8"))
    por_canal = {j["canal"]: (nome, j) for nome, j in cfg["juntas"].items()}

    try:
        sys.path.insert(0, str(RAIZ))
        import pegada_manual
        py_lim = pegada_manual.LIMITES
    except Exception:
        py_lim = {}

    divergencias = []
    for canal in range(len(ino_lo)):
        nome, junta = por_canal.get(canal, (f"canal {canal}", {}))
        fontes = {
            "braco.json": (junta.get("minimo"), junta.get("maximo")),
            ".ino": (ino_lo[canal], ino_hi[canal]),
        }
        if canal in py_lim:
            fontes["pegada_manual.py"] = py_lim[canal]

        valores = {k: (float(a), float(b)) for k, (a, b) in fontes.items()
                   if a is not None and b is not None}
        if len(set(valores.values())) > 1:
            divergencias.append((nome, valores))

    return [
        f"{nome}: " + "  ".join(f"{k} {a:.0f}-{b:.0f}" for k, (a, b) in valores.items())
        + "  (a fonte e o .ino)"
        for nome, valores in divergencias
    ]


def testa_acoplamento_sincronizado():
    """
    A reta do limite acoplado esta em DOIS lugares, e os dois tem de concordar.

    No firmware (ACOP_A / ACOP_B / ACOP_MARGEM) ela satura o comando; no
    kinematics.py ela RECUSA a pose, para o planejador tentar o outro ramo da
    IK. Se divergirem, o Python planeja uma pose que o firmware vai saturar em
    silencio: o braco para num angulo que ninguem pediu, no meio de uma
    sequencia, possivelmente com a peca na garra.

    E o mesmo modo de falha dos limites de junta, que ja apareceu duas vezes
    neste projeto - por isso vira falha, e nao aviso.
    """
    fonte = INO.read_text(encoding="utf-8")

    sys.path.insert(0, str(RAIZ))
    from src.arm.kinematics import CalibracaoBraco

    falhas = []
    for nome in ("ACOP_A", "ACOP_B", "ACOP_MARGEM"):
        achado = re.search(rf"const\s+float\s+{nome}\s*=\s*(-?[\d.]+)\s*;", fonte)
        if achado is None:
            falhas.append(f"{nome} nao encontrado no .ino")
            continue
        do_ino = float(achado.group(1))
        do_py = float(getattr(CalibracaoBraco, nome))
        if abs(do_ino - do_py) > 1e-9:
            falhas.append(f"{nome}: .ino {do_ino} x kinematics.py {do_py}")

    # A pose de repouso tambem e copiada: o pegada_manual.py tem um passo
    # "Volta ao repouso" que deve bater com o pose_repouso compilado, senao
    # o roteiro manual e o HOME terminam em lugares diferentes.
    repouso = _extrai_vetor(fonte, "pose_repouso")
    try:
        import pegada_manual
        passo = next((ang for desc, ang in pegada_manual.SEQUENCIA
                      if "repouso" in desc.lower()), None)
    except Exception:
        passo = None

    if passo:
        for canal, angulo in passo.items():
            if canal < len(repouso) and abs(float(angulo) - repouso[canal]) > 1e-9:
                falhas.append(
                    f"pose de repouso, canal {canal}: .ino {repouso[canal]:.0f} x "
                    f"pegada_manual.py {float(angulo):.0f}"
                )

    return falhas


def main():
    falhas = 0

    for entrada, esperado in CASOS_DISPATCH:
        obtido = dispatch(entrada)
        if obtido != esperado:
            falhas += 1
            print(f"  FALHA dispatch {entrada!r:20} esperado={esperado!r:16} obtido={obtido!r}")
    print(f"{len(CASOS_DISPATCH) - falhas}/{len(CASOS_DISPATCH)} casos de dispatch")

    falhas_mov = testa_movimento()
    for f in falhas_mov:
        print(f"  FALHA movimento: {f}")

    falhas_demo = testa_demo()
    for f in falhas_demo:
        print(f"  FALHA tabela DEMO: {f}")
    print(f"{'tabela DEMO do firmware ok' if not falhas_demo else f'tabela DEMO -> {len(falhas_demo)} falha(s)'}")

    falhas_limites = testa_limites_sincronizados()
    for f in falhas_limites:
        print(f"  FALHA limites: {f}")
    if not falhas_limites:
        print("limites de junta iguais nos tres arquivos")

    falhas_acop = testa_acoplamento_sincronizado()
    for f in falhas_acop:
        print(f"  FALHA acoplamento: {f}")
    if not falhas_acop:
        print("limite acoplado e pose de repouso iguais no .ino e no Python")

    falhas_sensor = testa_sensor()
    for f in falhas_sensor:
        print(f"  FALHA sensor: {f}")
    print(('todos os ' if not falhas_sensor else '') + 'testes do sensor/alivio ' + ('passaram' if not falhas_sensor else f'-> {len(falhas_sensor)} falha(s)'))
    print(f"{'todos os' if not falhas_mov else ''} testes de movimento "
          f"{'passaram' if not falhas_mov else f'-> {len(falhas_mov)} falha(s)'}")

    total = (falhas + len(falhas_mov) + len(falhas_sensor)
             + len(falhas_demo) + len(falhas_limites) + len(falhas_acop))
    print("\nOK" if total == 0 else f"\n{total} falha(s)")
    return 1 if total else 0


if __name__ == "__main__":
    raise SystemExit(main())
