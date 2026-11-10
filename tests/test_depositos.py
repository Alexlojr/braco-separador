#!/usr/bin/env python3
"""
Separacao em dois depositos: o destino vem do argumento, nao do parametro.

Sem numpy e sem cv2 de proposito, para rodar tambem na maquina de edicao.
O que este teste protege e a propriedade que faz a separacao existir: UM
controlador, calibrado uma vez, com o destino escolhido por ciclo. Se alguem
voltar a ler o deposito direto de `self.params` dentro de `planejar`, a
separacao passa a mandar tudo para o mesmo lugar sem erro nenhum.
"""
import math
import pathlib
import sys

BASE = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(BASE))

from src.arm.coleta import ControladorColeta, ParametrosColeta  # noqa: E402
from src.arm.kinematics import carregar_config  # noqa: E402

falhas = []


def checa(ok, msg):
    if not ok:
        falhas.append(msg)


class SerialMuda:
    def __init__(self):
        self.linhas = []

    def write(self, dados):
        self.linhas.append(dados.decode().strip())
        return len(dados)


braco, calib, params_dict, _ = carregar_config(str(BASE / "config/braco.json"))
params = ParametrosColeta(**params_dict)
ctrl = ControladorColeta(SerialMuda(), braco=braco, calib=calib, params=params)

ESQ = (-180.0, 0.0)
DIR = (180.0, 0.0)
ALVO = (0.0, 150.0)

# o padrao continua valendo quando ninguem passa destino
padrao = ctrl.planejar(*ALVO)
explicito = ctrl.planejar(*ALVO, deposito=params.deposito)
checa([e.comandos for e in padrao] == [e.comandos for e in explicito],
      "passar o proprio deposito da config tinha que dar o mesmo plano")

# destinos opostos tem que gerar planos DIFERENTES
plano_esq = ctrl.planejar(*ALVO, deposito=ESQ)
plano_dir = ctrl.planejar(*ALVO, deposito=DIR)
checa([e.comandos for e in plano_esq] != [e.comandos for e in plano_dir],
      "depositos em lados opostos geraram o MESMO plano: o destino nao esta "
      "chegando ao planejamento e a separacao nao existe")


def angulo_final_da_base(etapas):
    """Ultimo angulo comandado para a base antes do HOME."""
    canal = calib.base.canal
    visto = None
    for etapa in etapas:
        for cmd in etapa.comandos:
            partes = cmd.split()
            if len(partes) == 3 and partes[0] == "SET" and partes[1] == str(canal):
                visto = float(partes[2])
    return visto


ang_esq = angulo_final_da_base(plano_esq)
ang_dir = angulo_final_da_base(plano_dir)

esperado_esq = math.degrees(math.atan2(ESQ[1], ESQ[0])) % 360.0
esperado_dir = math.degrees(math.atan2(DIR[1], DIR[0])) % 360.0

checa(ang_esq is not None and abs(ang_esq - esperado_esq) < 1.0,
      f"deposito esquerdo deveria levar a base a ~{esperado_esq:.0f}°, "
      f"veio {ang_esq}")
checa(ang_dir is not None and abs(ang_dir - esperado_dir) < 1.0,
      f"deposito direito deveria levar a base a ~{esperado_dir:.0f}°, "
      f"veio {ang_dir}")

# o plano e sempre completo: validacao do destino acontece antes de mover
for rotulo, plano in (("esquerdo", plano_esq), ("direito", plano_dir)):
    descricoes = " | ".join(e.descricao.lower() for e in plano)
    checa("deposito" in descricoes or "depósito" in descricoes,
          f"plano do deposito {rotulo} nao tem etapa de deposito")
    checa(plano[-1].comandos == ["HOME"],
          f"plano do deposito {rotulo} tinha que terminar em HOME")

# o controlador nao guarda estado do destino entre ciclos
depois = ctrl.planejar(*ALVO)
checa([e.comandos for e in depois] == [e.comandos for e in padrao],
      "o destino de um ciclo vazou para o seguinte: planejar sem deposito "
      "tem que voltar ao padrao da configuracao")

# o mapa da configuracao tem de apontar para pontos alcancaveis
import json  # noqa: E402

bruto = json.loads((BASE / "config/braco.json").read_text(encoding="utf-8"))
mapa = bruto.get("coleta", {}).get("depositos_por_classe", {}) or {}
checa(bool(mapa), "depositos_por_classe vazio: a separacao nao tem para onde ir")
for nome, xy in mapa.items():
    try:
        ctrl.planejar(*ALVO, deposito=(float(xy[0]), float(xy[1])))
    except Exception as e:
        falhas.append(f"deposito da classe {nome!r} em {xy} nao fecha: {e}")

for f in falhas:
    print(f"  FALHA: {f}")
print("OK — a separacao em dois depositos esta ligada" if not falhas
      else f"\n{len(falhas)} falha(s)")
raise SystemExit(1 if falhas else 0)
