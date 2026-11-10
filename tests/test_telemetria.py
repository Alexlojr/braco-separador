#!/usr/bin/env python3
"""
Telemetria do painel: os angulos saem dos comandos, o log sai do terminal.

Sem numpy e sem cv2, para rodar na maquina de edicao.
"""
import io
import pathlib
import sys

BASE = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(BASE))

from src import telemetria as tm  # noqa: E402

falhas = []


def checa(ok, msg):
    if not ok:
        falhas.append(msg)


def angulos(t):
    return {c: round(s["ang"], 1) for c, s in t.servos.items()}


tm.Telemetria.limpar()
t = tm.Telemetria(pose_repouso=[50, 120, 120, 105])
checa(angulos(t) == {0: 50, 1: 120, 2: 120, 3: 105},
      "antes de qualquer comando, os angulos tinham de ser os da pose de repouso")

t.comando("SET 1 161.0")
t.comando("SET 2 93.6")
checa(angulos(t)[1] == 161.0 and angulos(t)[2] == 93.6, "SET nao atualizou o angulo")

t.comando("SPEED 40")
t.comando("BUSY")
checa(angulos(t)[1] == 161.0, "comando que nao e de posicao mexeu no angulo")

t.comando("DETACH 0")
checa(t.servos[0]["solto"], "DETACH nao marcou a junta como solta")
t.comando("ATTACH ALL")
checa(not any(s["solto"] for s in t.servos.values()), "ATTACH ALL nao religou")

t.comando("HOME")
checa(angulos(t) == {0: 50, 1: 120, 2: 120, 3: 105}, "HOME nao voltou a pose de repouso")

t.comando("SET x lixo")  # linha malformada nao pode derrubar a coleta
checa(angulos(t)[0] == 50, "linha malformada alterou o estado")

# o espelho repassa ao terminal E alimenta o log, linha a linha
terminal = io.StringIO()
esp = t.espelhar(terminal)
print("Descendo ao lado da peça, a 20 mm", file=esp)
print("parte 1 ", end="", file=esp)
print("parte 2", file=esp)
checa(terminal.getvalue() == "Descendo ao lado da peça, a 20 mm\nparte 1 parte 2\n",
      "o terminal nao recebeu exatamente o que foi impresso")
ultimas = [linha.split("  ", 1)[1] for linha in list(t.log)[-2:]]
checa(ultimas == ["Descendo ao lado da peça, a 20 mm", "parte 1 parte 2"],
      f"log diferente do terminal: {ultimas}")

# o painel le o que a coleta publicou
lido = tm.ler()
checa(lido is not None, "nada publicado para o painel")
if lido:
    checa([s["nome"] for s in lido["servos"]] == ["Base", "Ombro", "Cotovelo", "Garra"],
          "nomes das juntas errados no que o painel le")
    checa(lido["log"][-1].endswith("parte 1 parte 2"), "log publicado desatualizado")

tm.Telemetria.limpar()
checa(tm.ler() is None, "limpar() deixou publicacao para tras")

for f in falhas:
    print(f"  FALHA: {f}")
print("OK — telemetria do painel" if not falhas else f"\n{len(falhas)} falha(s)")
raise SystemExit(1 if falhas else 0)
