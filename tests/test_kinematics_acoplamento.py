"""
Testes do acoplamento do cotovelo.

Este braço é de paralelogramo: os dois servomotores ficam na base, o cotovelo
é uma junta passiva, e o servo 2 manda no ângulo do antebraço em relação ao
CHÃO — não ao braço superior. A cinemática calcula o ângulo relativo, então
alguém precisa somar o ombro antes de mandar pelo serial.

Confirmado nas fotos de perfil de outubro/2026, com o ombro a 120 e a 180
graus e o cotovelo intocado: o antebraço manteve a inclinação com o chão.

O que torna esse erro traiçoeiro é que ele NÃO é um desvio constante. Um
desvio constante seria absorvido pelo `offset` da junta na primeira
calibração e ninguém notaria. Aqui o erro vale exatamente theta_ombro, que
muda a cada alvo, então o braço erra mais em uns pontos da mesa que em
outros e o padrão parece aleatório.

    python tests/test_kinematics_acoplamento.py
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from src.arm.kinematics import (  # noqa: E402
    Braco,
    CalibracaoBraco,
    CalibracaoServo,
    pose_para_servos,
)

falhas = []


def checa(cond, msg):
    if not cond:
        falhas.append(msg)


def nova_calib(acoplamento):
    return CalibracaoBraco(
        base=CalibracaoServo(0, "base", offset=0, sentido=1, minimo=0, maximo=180),
        ombro=CalibracaoServo(1, "ombro", offset=90, sentido=1, minimo=0, maximo=180),
        cotovelo=CalibracaoServo(2, "cotovelo", offset=90, sentido=1, minimo=0, maximo=180),
        garra=CalibracaoServo(3, "garra", offset=0, sentido=1, minimo=80, maximo=130),
        acoplamento_cotovelo=acoplamento,
    )


# --- 'relativo' não mexe em nada ----------------------------------------
angulos = {"base": 30.0, "ombro": -40.0, "cotovelo": 55.0}
saida = nova_calib("relativo").angulos_de_servo(angulos)
checa(saida == angulos, f"'relativo' deveria ser identidade, veio {saida}")


# --- 'absoluto' soma o ombro no cotovelo, e só nele ----------------------
saida = nova_calib("absoluto").angulos_de_servo(angulos)
checa(abs(saida["cotovelo"] - 15.0) < 1e-9,
      f"cotovelo absoluto deveria ser -40+55 = 15, veio {saida['cotovelo']}")
checa(saida["base"] == 30.0 and saida["ombro"] == -40.0,
      "base e ombro não deveriam mudar com o acoplamento")
checa(angulos["cotovelo"] == 55.0,
      "angulos_de_servo não pode alterar o dicionário que recebeu")


# --- valor inválido falha alto, não escolhe um padrão --------------------
try:
    nova_calib("paralelogramo").angulos_de_servo(angulos)
    falhas.append("acoplamento inválido deveria levantar ValueError")
except ValueError:
    pass


# --- o erro varia com a pose: não dá para absorver num offset ------------
#
# É este o teste que importa. Se a diferença entre os dois acoplamentos
# fosse constante, trocar um pelo outro seria inofensivo depois de
# recalibrar o offset. Ela não é.
braco = Braco(l1=50.0, l2=120.0, l3=130.0, offset_radial=20.0)
rel, absol = nova_calib("relativo"), nova_calib("absoluto")

diferencas = []
for alvo in [(0.0, 120.0, 15.0), (0.0, 180.0, 15.0),
             (0.0, 150.0, 90.0), (60.0, 160.0, 40.0)]:
    a = pose_para_servos(braco, rel, *alvo, permitir_clamp=True)
    b = pose_para_servos(braco, absol, *alvo, permitir_clamp=True)
    diferencas.append(b[2] - a[2])

espalhamento = max(diferencas) - min(diferencas)
checa(espalhamento > 20.0,
      f"a diferença entre os acoplamentos deveria variar muito com a pose; "
      f"variou só {espalhamento:.1f}° ({[round(d, 1) for d in diferencas]})")


# --- o ângulo comandado é mesmo o absoluto do antebraço ------------------
alvo = (0.0, 150.0, 40.0)
ang = braco.resolver_ik(*alvo)
checa(ang is not None, f"alvo {alvo} deveria ser alcançável com estes elos")
if ang is not None:
    comandos = pose_para_servos(braco, absol, *alvo, permitir_clamp=True)
    esperado = 90 + (ang["ombro"] + ang["cotovelo"])
    checa(abs(comandos[2] - round(esperado, 1)) < 0.05,
          f"canal 2 deveria receber {esperado:.1f}, veio {comandos[2]}")



# ---------------------------------------------------------------------------
# Limite acoplado cotovelo x ombro, contra as medicoes de bancada
# ---------------------------------------------------------------------------
# Cada linha e uma pose testada no braco real em 03 e 07/11, com DETACH e a
# mao. "colide" significa que ali o braco encostou; "livre" que nao.
#
# A regra tem que RECUSAR tudo que colidiu e ACEITAR o que estava livre. Se
# um dia alguem mexer em ACOP_A, ACOP_B ou ACOP_MARGEM sem remedir, e aqui
# que isso para.
MEDICOES = [
    (100, 135, "colide"),
    (115, 115, "colide"),   # ponto de validacao: nao entrou no ajuste da reta
    (130,  90, "colide"),
    ( 90, 136, "colide"),   # aqui quem encosta e a garra, nao o antebraco
    (160,  70, "livre"),
]

cal = CalibracaoBraco()
for s1, s2, esperado in MEDICOES:
    problema = cal.viola_acoplamento(float(s1), float(s2))
    if esperado == "colide":
        checa(problema is not None,
              f"ombro {s1} / cotovelo {s2} colidiu na bancada e a regra deixou passar")
    else:
        checa(problema is None,
              f"ombro {s1} / cotovelo {s2} estava livre e a regra recusou: {problema}")

# A regra nunca pode afrouxar o limite de caixa.
checa(cal.cotovelo_minimo_para(180.0) >= cal.cotovelo.minimo,
      "com o ombro estendido a regra nao pode pedir menos que o lim_min do cotovelo")

# E tem que ser monotona: ombro mais recolhido exige cotovelo mais alto.
anterior = None
for s1 in range(90, 181, 10):
    m = cal.cotovelo_minimo_para(float(s1))
    if anterior is not None:
        checa(m <= anterior, f"cotovelo minimo deveria cair quando o ombro estende (s1={s1})")
    anterior = m

for f in falhas:
    print(f"  FALHA: {f}")
print("OK — todos os testes de acoplamento passaram" if not falhas
      else f"\n{len(falhas)} falha(s)")
raise SystemExit(1 if falhas else 0)
