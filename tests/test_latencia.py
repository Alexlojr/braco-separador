#!/usr/bin/env python3
"""
Confere a marca de tempo da SerialInstrumentada sem importar o coletar.py
inteiro, que arrasta ultralytics e cv2 (nao instalados no Windows).

Extrai o bloco da classe do fonte e executa so ele. Roda sem dependencia
nenhuma, de proposito: e a medida que vai para a secao 5.1 do relatorio.
"""
import pathlib
import re
import time

CAMINHO = pathlib.Path(__file__).resolve().parent.parent / "coletar.py"
fonte = CAMINHO.read_text(encoding="utf-8")
m = re.search(r"^class SerialInstrumentada:.*?(?=^\S)", fonte, re.S | re.M)
if not m:
    raise SystemExit("nao achei a classe SerialInstrumentada")

ns = {"time": time}
exec(m.group(0), ns)
SerialInstrumentada = ns["SerialInstrumentada"]


class Alvo:
    def __init__(self):
        self.linhas = []
        self.falhas_envio = 0

    def write(self, dados):
        self.linhas.append(dados.decode().strip())
        return len(dados)


falhas = []


def checa(ok, msg):
    if not ok:
        falhas.append(msg)


alvo = Alvo()
s = SerialInstrumentada(alvo)

# comandos que NAO sao de posicao nao devem marcar
for cmd in (b"SPEED 40\n", b"DETACH 1\n", b"ATTACH ALL\n", b"AMP\n"):
    s.write(cmd)
checa(s.t_primeiro_comando is None,
      "SPEED/DETACH/ATTACH/AMP nao deveriam marcar o inicio da latencia")

t0 = time.monotonic()
s.write(b"SET 3 105\n")
t1 = time.monotonic()
marca = s.t_primeiro_comando
checa(marca is not None, "o primeiro SET tinha que marcar")
checa(marca is not None and t0 <= marca <= t1,
      "a marca tem que cair entre antes e depois do write")

# o segundo SET nao pode mover a marca
anterior = s.t_primeiro_comando
time.sleep(0.01)
s.write(b"SET 1 120\n")
checa(s.t_primeiro_comando == anterior,
      "so o PRIMEIRO comando de posicao conta; o segundo nao pode sobrescrever")

# armar esquece a sequencia anterior
s.armar_marco()
checa(s.t_primeiro_comando is None, "armar_marco tinha que zerar a marca")
s.write(b"SET 0 90\n")
checa(s.t_primeiro_comando is not None, "depois de armar, o proximo SET marca")

# repasse: tudo chegou na serial de verdade, na ordem
checa(alvo.linhas == ["SPEED 40", "DETACH 1", "ATTACH ALL", "AMP",
                      "SET 3 105", "SET 1 120", "SET 0 90"],
      "o wrapper tem que repassar todas as linhas, na ordem: %r" % alvo.linhas)

# passthrough de atributos
checa(s.falhas_envio == 0, "__getattr__ tinha que achar falhas_envio no alvo")

# eco desligado por padrao, ligado quando pedido
checa(SerialInstrumentada(Alvo())._eco is False, "eco deveria vir desligado")
checa(SerialInstrumentada(Alvo(), eco=True)._eco is True, "eco=True ignorado")

for f in falhas:
    print("  FALHA: %s" % f)
print("OK — a marca do primeiro comando de posicao esta correta" if not falhas
      else "\n%d falha(s)" % len(falhas))
raise SystemExit(1 if falhas else 0)
