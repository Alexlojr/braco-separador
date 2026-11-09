"""
Cinemática do braço: converte uma posição (x, y, z) em milímetros
nos ângulos de servo que o firmware entende.

São duas camadas separadas de propósito:

1. `resolver_ik()` — matemática pura. Trabalha na convenção matemática
   (ângulos absolutos, 0° = horizontal, cotovelo relativo ao ombro).
   Não sabe nada sobre servos.

2. `CalibracaoServo` — traduz ângulo matemático -> ângulo de servo (0-180).
   É aqui que entram offset, inversão de sentido e limites físicos, que
   variam conforme a montagem mecânica do braço.

Essa separação importa: se o braço for remontado ou trocado, só a camada 2
muda. A matemática da camada 1 continua válida.

Convenção da camada 1 (mantenha igual no firmware, se algum dia migrar pra lá):
    theta_base     -> rotação no plano XY, 0° = eixo +X
    theta_ombro    -> ângulo do elo L2 em relação à horizontal
    theta_cotovelo -> ângulo do elo L3 RELATIVO ao L2 (0° = braço esticado)
"""

from dataclasses import dataclass, field
import math


# ---------------------------------------------------------------------------
# Camada 1: matemática pura
# ---------------------------------------------------------------------------

@dataclass
class Braco:
    """Dimensões físicas do braço, em milímetros. MEÇA no braço real."""
    # Altura VERTICAL do topo da placa até o eixo do ombro. A placa do braço
    # tem a mesma espessura das placas da área de coleta, então esse topo é o
    # mesmo plano onde o objeto se apoia e onde a homografia é calibrada:
    # z = 0 vale para os dois, sem offset a corrigir.
    l1: float = 50.0
    l2: float = 80.0   # eixo do ombro -> eixo do cotovelo
    l3: float = 80.0   # eixo do cotovelo -> ponto de preensão da garra

    # Deslocamento horizontal entre o eixo da base e o eixo do ombro.
    # Em vários braços o ombro não fica exatamente acima do centro de giro
    # da base; ignorar isso produz um erro radial constante, do mesmo
    # tamanho do deslocamento, em todos os alvos.
    offset_radial: float = 0.0

    @property
    def alcance_max(self) -> float:
        return self.l2 + self.l3

    @property
    def alcance_min(self) -> float:
        return abs(self.l2 - self.l3)

    def resolver_ik_ramos(self, x: float, y: float, z: float):
        """
        Todas as soluções de cinemática inversa para o alvo, em GRAUS.

        Uma cadeia planar de dois elos alcança cada ponto por DUAS poses,
        espelhadas em torno da reta ombro->alvo: cotovelo pra cima e cotovelo
        pra baixo. A lei dos cossenos devolve o módulo do ângulo; o sinal é
        quem escolhe o ramo.

        Antes aqui só existia o ramo positivo, e isso bloqueava o braço
        inteiro. Com offset de ombro 200 o elo do ombro nunca desce abaixo de
        20° da horizontal, então quem leva a garra até a mesa é o antebraço
        apontando pra baixo — exatamente o ramo negativo. Só o ramo positivo
        pedia ombro em -74°, ou seja servo 274, e todo alvo na altura da mesa
        era recusado por limite de junta.

        Devolve [] se o alvo estiver fora do envelope. A ordem é
        [positivo, negativo]; quem chama escolhe pelos limites das juntas.
        """
        theta_base = math.degrees(math.atan2(y, x))

        r = math.hypot(x, y) - self.offset_radial
        h = z - self.l1
        dist2 = r * r + h * h
        dist = math.sqrt(dist2)

        if dist > self.alcance_max or dist < self.alcance_min:
            return []

        cos_cot = (dist2 - self.l2 ** 2 - self.l3 ** 2) / (2 * self.l2 * self.l3)
        cos_cot = max(-1.0, min(1.0, cos_cot))
        magnitude = math.acos(cos_cot)

        solucoes = []
        for theta_cotovelo in (magnitude, -magnitude):
            theta_ombro = math.atan2(h, r) - math.atan2(
                self.l3 * math.sin(theta_cotovelo),
                self.l2 + self.l3 * math.cos(theta_cotovelo),
            )
            solucoes.append({
                "base": theta_base,
                "ombro": math.degrees(theta_ombro),
                "cotovelo": math.degrees(theta_cotovelo),
            })
            if magnitude == 0.0:     # braço reto: os dois ramos coincidem
                break
        return solucoes

    def resolver_ik(self, x: float, y: float, z: float):
        """
        Entrada: alvo (x, y, z) em mm, relativo à base do braço.
        Saída: dict com ângulos em GRAUS (convenção matemática), ou
               None se o alvo estiver fora do envelope alcançável.

        Devolve só o primeiro ramo, de cotovelo positivo. Quem precisa
        respeitar os limites das juntas deve usar `resolver_ik_ramos()`,
        porque neste braço é o segundo ramo que serve na altura da mesa.
        """
        ramos = self.resolver_ik_ramos(x, y, z)
        return ramos[0] if ramos else None

    def cinematica_direta(self, base: float, ombro: float, cotovelo: float):
        """Inverso da IK. Serve para validar que os ângulos reproduzem o alvo."""
        tb, to, tc = map(math.radians, (base, ombro, cotovelo))
        r = self.l2 * math.cos(to) + self.l3 * math.cos(to + tc) + self.offset_radial
        h = self.l2 * math.sin(to) + self.l3 * math.sin(to + tc)
        return r * math.cos(tb), r * math.sin(tb), h + self.l1

    def erro_reconstrucao(self, x, y, z) -> float | None:
        """
        Resolve a IK e reconstrói via cinemática direta.
        Retorna a distância entre o alvo pedido e o reconstruído (mm),
        ou None se o alvo for inalcançável. Use isso como teste de sanidade.
        """
        ang = self.resolver_ik(x, y, z)
        if ang is None:
            return None
        xr, yr, zr = self.cinematica_direta(ang["base"], ang["ombro"], ang["cotovelo"])
        return math.dist((x, y, z), (xr, yr, zr))


# ---------------------------------------------------------------------------
# Camada 2: tradução para ângulos de servo
# ---------------------------------------------------------------------------

@dataclass
class CalibracaoServo:
    """
    Traduz um ângulo matemático em ângulo de servo (0-180).

        angulo_servo = offset + sentido * angulo_matematico

    - `offset`: quanto vale, no servo, o 0° matemático da junta.
    - `sentido`: +1 se o servo cresce no mesmo sentido da convenção
      matemática, -1 se cresce ao contrário (descobre-se testando).
    - `minimo`/`maximo`: limites FÍSICOS reais da junta, achados com o
      sketch de teste individual. Nunca deixe 0/180 sem ter verificado.
    """
    canal: int
    nome: str
    offset: float = 90.0
    sentido: int = 1
    minimo: float = 10.0
    maximo: float = 170.0

    def para_servo(self, angulo_matematico: float) -> float:
        return self.offset + self.sentido * angulo_matematico

    def dentro_dos_limites(self, angulo_servo: float) -> bool:
        return self.minimo <= angulo_servo <= self.maximo

    def clamp(self, angulo_servo: float) -> float:
        return max(self.minimo, min(self.maximo, angulo_servo))


@dataclass
class CalibracaoBraco:
    """
    Conjunto de calibrações, uma por junta, na ordem dos canais do PCA9685.

    ATENÇÃO: os valores abaixo são PLACEHOLDERS auto-consistentes, escolhidos
    só para o pipeline rodar ponta a ponta antes da calibração física. Eles
    NÃO correspondem ao braço real. Antes de mover o braço de verdade:

      1. rode o sketch de teste individual e ache o mín/máx real de cada junta;
      2. ache o `offset` mandando a junta para o 0° matemático e vendo que
         ângulo de servo deixa o elo na posição de referência;
      3. ache o `sentido` mandando um ângulo maior e vendo se o elo se move
         no sentido esperado (se for ao contrário, use -1).
    """
    base: CalibracaoServo = field(
        default_factory=lambda: CalibracaoServo(0, "base", offset=0, sentido=1, minimo=10, maximo=170)
    )
    ombro: CalibracaoServo = field(
        default_factory=lambda: CalibracaoServo(1, "ombro", offset=90, sentido=1, minimo=20, maximo=160)
    )
    cotovelo: CalibracaoServo = field(
        default_factory=lambda: CalibracaoServo(2, "cotovelo", offset=0, sentido=1, minimo=10, maximo=170)
    )
    garra: CalibracaoServo = field(
        default_factory=lambda: CalibracaoServo(3, "garra", offset=0, sentido=1, minimo=30, maximo=130)
    )

    # Como o servomotor do cotovelo se relaciona com o antebraco.
    #
    #   "relativo"  cadeia serial comum: o servo fica NO cotovelo e manda no
    #               angulo entre antebraco e braco superior. E o que a
    #               convencao matematica daqui ja chama de theta_cotovelo.
    #
    #   "absoluto"  mecanismo de paralelogramo (MeArm e parentes): os dois
    #               servomotores ficam na BASE e o cotovelo e uma junta
    #               passiva. O servo manda no angulo do antebraco em relacao
    #               ao CHAO, nao ao braco superior, e a garra se mantem
    #               nivelada sozinha em qualquer pose.
    #
    # Na convencao daqui o angulo absoluto do antebraco e theta_ombro +
    # theta_cotovelo, entao a diferenca entre os dois casos e uma soma.
    #
    # Errar isto nao quebra nada de forma visivel: o cotovelo so erra por um
    # valor que muda conforme a altura do ombro, e o erro parece aleatorio.
    # O teste esta em tests/test_kinematics_acoplamento.py.
    acoplamento_cotovelo: str = "relativo"

    def angulos_de_servo(self, angulos: dict[str, float]) -> dict[str, float]:
        """
        Converte os angulos da cinematica no que cada SERVO precisa receber,
        ainda em graus matematicos (offset e sentido vem depois).

        So o cotovelo muda, e so no caso paralelogramo.
        """
        if self.acoplamento_cotovelo == "absoluto":
            saida = dict(angulos)
            saida["cotovelo"] = angulos["ombro"] + angulos["cotovelo"]
            return saida
        if self.acoplamento_cotovelo != "relativo":
            raise ValueError(
                f"acoplamento_cotovelo invalido: {self.acoplamento_cotovelo!r}. "
                "Use 'relativo' ou 'absoluto'."
            )
        return angulos

    def juntas_ik(self):
        """Só as três juntas que participam da cinemática (garra fica de fora)."""
        return {"base": self.base, "ombro": self.ombro, "cotovelo": self.cotovelo}

    # Limite acoplado cotovelo x ombro. Precisa bater com ACOP_A / ACOP_B /
    # ACOP_MARGEM do braco_firmware.ino, onde está a medição que os originou.
    ACOP_A = 285.0
    ACOP_B = 1.5
    ACOP_MARGEM = 15.0

    def cotovelo_minimo_para(self, servo_ombro: float) -> float:
        """Menor ângulo de servo do cotovelo que não encosta, dado o ombro."""
        m = self.ACOP_A - self.ACOP_B * servo_ombro + self.ACOP_MARGEM
        return max(m, self.cotovelo.minimo)

    def viola_acoplamento(self, servo_ombro: float, servo_cotovelo: float):
        """
        None se a dupla é segura; a mensagem do problema se não é.

        O firmware também aplica esta regra, e lá ela satura. Aqui ela RECUSA,
        de propósito: o planejador tem para onde ir — a cinemática inversa
        resolve o mesmo alvo por dois ramos, e recusar um deixa o outro ser
        tentado. Saturar escolheria em silêncio uma pose que ninguém pediu.
        """
        minimo = self.cotovelo_minimo_para(servo_ombro)
        if servo_cotovelo < minimo:
            return (
                f"ombro em {servo_ombro:.1f}° exige cotovelo >= {minimo:.1f}°, "
                f"veio {servo_cotovelo:.1f}° (limite acoplado: o antebraço "
                f"encosta no braço antes do limite de caixa)"
            )
        return None


class PoseInalcancavel(Exception):
    """
    Alvo fora do envelope físico, ou ângulo fora dos limites da junta.

    O `motivo` diz qual dos dois casos ocorreu, e as duas coisas se resolvem
    de maneiras opostas: ENVELOPE quer dizer que o braço é curto (ou longo)
    demais para o alvo, e só se resolve mudando a geometria ou aproximando a
    área de coleta; LIMITE_JUNTA quer dizer que o braço alcançaria, mas a
    calibração proíbe o ângulo — normalmente é limite apertado demais no
    `braco.json`, não impossibilidade física.

    Leia `e.motivo`, nunca o texto da mensagem: o texto é para humanos e pode
    mudar sem aviso.
    """

    ENVELOPE = "envelope"
    LIMITE_JUNTA = "limite_junta"

    def __init__(self, mensagem: str, motivo: str):
        super().__init__(mensagem)
        self.motivo = motivo


def pose_para_servos(
    braco: Braco,
    calib: CalibracaoBraco,
    x: float,
    y: float,
    z: float,
    permitir_clamp: bool = False,
) -> dict[int, float]:
    """
    Caminho completo: alvo em mm -> {canal_do_servo: ângulo}.

    Levanta PoseInalcancavel se o alvo estiver fora do envelope, ou se
    algum ângulo cair fora dos limites da junta (a menos que
    `permitir_clamp=True`, que satura em vez de recusar).

    Recusar por padrão é intencional: um alvo ligeiramente fora do alcance
    que vira um clamp silencioso faz o braço ir para uma pose errada sem
    ninguém perceber. Melhor falhar alto e tratar em cima.
    """
    ramos = braco.resolver_ik_ramos(x, y, z)
    if not ramos:
        raise PoseInalcancavel(
            f"Alvo ({x:.0f}, {y:.0f}, {z:.0f}) fora do envelope "
            f"[{braco.alcance_min:.0f}..{braco.alcance_max:.0f}] mm",
            motivo=PoseInalcancavel.ENVELOPE,
        )

    # Dos dois ramos, vale o primeiro que couber nos limites. Testar os dois
    # nao e refinamento: com a calibracao deste braco o ramo de cotovelo
    # positivo pede ombro em -74 graus para qualquer alvo na altura da mesa,
    # e sozinho ele recusaria a area de coleta inteira.
    ultimo_erro = None
    for angulos in ramos:
        # O que a cinematica calcula nao e necessariamente o que o servo
        # recebe: num braco de paralelogramo o servo do cotovelo manda no
        # angulo absoluto do antebraco. Ver CalibracaoBraco.acoplamento_cotovelo.
        angulos_servo = calib.angulos_de_servo(angulos)

        comandos: dict[int, float] = {}
        estourou = None
        for nome, servo in calib.juntas_ik().items():
            bruto = servo.para_servo(angulos_servo[nome])
            if not servo.dentro_dos_limites(bruto):
                estourou = (
                    f"Junta '{nome}' exigiria {bruto:.1f}°, "
                    f"fora do limite [{servo.minimo:.0f}, {servo.maximo:.0f}]"
                )
                break
            comandos[servo.canal] = round(bruto, 1)

        if estourou is None:
            acoplado = calib.viola_acoplamento(
                comandos[calib.ombro.canal], comandos[calib.cotovelo.canal]
            )
            if acoplado is None:
                return comandos
            estourou = acoplado
        ultimo_erro = estourou

    if permitir_clamp:
        angulos_servo = calib.angulos_de_servo(ramos[0])
        return {
            servo.canal: round(servo.clamp(servo.para_servo(angulos_servo[nome])), 1)
            for nome, servo in calib.juntas_ik().items()
        }

    raise PoseInalcancavel(
        f"Nenhum dos {len(ramos)} ramos cabe nos limites. {ultimo_erro}",
        motivo=PoseInalcancavel.LIMITE_JUNTA,
    )


def classificar_alvo(braco: Braco, calib: CalibracaoBraco, x: float, y: float, z: float) -> str:
    """
    Diz se o braço pega em (x, y, z), sem levantar exceção.

    Devolve "ok", PoseInalcancavel.ENVELOPE ou PoseInalcancavel.LIMITE_JUNTA.
    Serve para desenhar mapas de alcance, onde é preciso testar milhares de
    pontos e um try/except por ponto polui quem chama.
    """
    try:
        pose_para_servos(braco, calib, x, y, z)
        return "ok"
    except PoseInalcancavel as e:
        return e.motivo


# ---------------------------------------------------------------------------
# Carregamento a partir de config/braco.json
# ---------------------------------------------------------------------------

def carregar_config(caminho="config/braco.json"):
    """
    Lê o JSON de configuração e devolve (braco, calib, params_dict, calibrado).

    `params_dict` sai como dicionário e não como ParametrosColeta para
    evitar import circular — quem chama monta o objeto.

    `calibrado` é a flag do arquivo: enquanto for False, os números ainda
    são placeholders e o braço não deve ser movido de verdade.
    """
    import json
    from pathlib import Path

    caminho = Path(caminho)
    if not caminho.exists():
        raise FileNotFoundError(
            f"config não encontrada em {caminho}. "
            "Copie config/braco.json do repositório e preencha com as medidas reais."
        )

    dados = json.loads(caminho.read_text(encoding="utf-8"))

    elos = dados["elos_mm"]
    braco = Braco(
        l1=float(elos["l1"]),
        l2=float(elos["l2"]),
        l3=float(elos["l3"]),
        offset_radial=float(dados.get("offset_radial_mm", 0.0)),
    )

    def _servo(nome: str) -> CalibracaoServo:
        j = dados["juntas"][nome]
        return CalibracaoServo(
            canal=int(j["canal"]),
            nome=nome,
            offset=float(j["offset"]),
            sentido=int(j["sentido"]),
            minimo=float(j["minimo"]),
            maximo=float(j["maximo"]),
        )

    calib = CalibracaoBraco(
        base=_servo("base"),
        ombro=_servo("ombro"),
        cotovelo=_servo("cotovelo"),
        garra=_servo("garra"),
        acoplamento_cotovelo=str(dados.get("acoplamento_cotovelo", "relativo")),
    )

    coleta = dados.get("coleta", {})
    params = {
        "z_mesa": float(coleta.get("z_mesa", 0.0)),
        "z_pega": float(coleta.get("z_pega", 15.0)),
        "z_aproximacao": float(coleta.get("z_aproximacao", 70.0)),
        "z_transporte": float(coleta.get("z_transporte", 80.0)),
        "recuo_entrada": float(coleta.get("recuo_entrada_mm", 30.0)),
        "recuo_minimo": float(coleta.get("recuo_minimo_mm", 25.0)),
        "subida_entrada": float(coleta.get("subida_entrada_mm", 5.0)),
        "deslocamento_radial": float(coleta.get("deslocamento_radial_mm", 0.0)),
        "deposito": tuple(coleta.get("deposito_xy", (-80.0, 100.0))),
        "pausa_s": float(coleta.get("pausa_s", 0.25)),
        # None deixa a coleta cair no mínimo/máximo da junta. Explícito é
        # melhor: qual extremo fecha a garra depende da montagem mecânica.
        "garra_aberta": (
            float(coleta["garra_aberta_graus"])
            if coleta.get("garra_aberta_graus") is not None else None
        ),
        "garra_fechada": (
            float(coleta["garra_fechada_graus"])
            if coleta.get("garra_fechada_graus") is not None else None
        ),
    }

    return braco, calib, params, bool(dados.get("calibrado", False))
