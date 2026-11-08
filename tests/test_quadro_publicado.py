#!/usr/bin/env python3
"""
Publicação de quadro entre a coleta e o painel, sem câmera.

O que isto protege: `coletar.py` e `app.py` disputavam a câmera, e só um
processo consegue. Agora a coleta publica e o painel lê. As três coisas que
quebram esse arranjo em silêncio são: ler publicação velha achando que é ao
vivo, ler um JPEG escrito pela metade, e a publicação derrubar a coleta
quando o disco recusa.
"""

import sys
from pathlib import Path

import numpy as np

RAIZ = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(RAIZ))

from src.vision import quadro_publicado as qp  # noqa: E402

falhas = []


def checa(cond, msg):
    if not cond:
        falhas.append(msg)


quadro = np.zeros((48, 64, 3), dtype=np.uint8)
quadro[10:20, 10:30] = (0, 200, 0)

qp.limpar()

# --- 1. sem publicacao, ler() devolve None ---------------------------------
checa(qp.ler() is None, "sem publicacao, ler() deveria devolver None")

# --- 2. ida e volta --------------------------------------------------------
checa(qp.publicar(quadro, {"rotulo": "peca", "coletados": 3}),
      "publicar() deveria ter conseguido")

lido = qp.ler()
checa(lido is not None, "logo apos publicar, ler() nao deveria devolver None")
if lido is not None:
    frame, meta = lido
    checa(frame.shape == quadro.shape,
          f"forma deveria ser {quadro.shape}, veio {frame.shape}")
    checa(meta.get("rotulo") == "peca", f"rotulo nao voltou: {meta}")
    checa(meta.get("coletados") == 3, f"coletados nao voltou: {meta}")
    checa("t" in meta, "meta deveria carregar o instante da publicacao")

# --- 3. publicacao velha nao vale ------------------------------------------
# Imagem parada e antiga no painel parece ao vivo, que e pior do que painel
# sem imagem: da a entender que a coleta esta rodando quando ela morreu.
checa(qp.ler(idade_maxima_s=-1.0) is None,
      "publicacao fora da idade maxima deveria ser recusada")

# --- 4. escrita atomica ----------------------------------------------------
# Sem os.replace o leitor pega JPEG pela metade de vez em quando, e o
# sintoma e uma imagem corrompida intermitente, dificil de ligar a causa.
# Aqui se cobra que nao sobre nenhum arquivo temporario depois de publicar.
for _ in range(5):
    qp.publicar(quadro, {"n": 1})
temporarios = [p.name for p in qp._diretorio().iterdir() if p.name.endswith(".tmp")]
checa(not temporarios, f"sobraram temporarios depois de publicar: {temporarios}")

# --- 5. publicar nunca derruba quem chama ----------------------------------
# A coleta nao pode parar porque o painel ficou sem imagem.
checa(qp.publicar(None) is False, "publicar(None) deveria devolver False, nao levantar")

# --- 6. limpar() apaga mesmo -----------------------------------------------
qp.limpar()
checa(qp.ler() is None, "depois de limpar(), ler() deveria devolver None")
qp.limpar()   # duas vezes seguidas nao pode levantar

for f in falhas:
    print(f"  FALHA: {f}")
print("OK — publicacao de quadro" if not falhas else f"\n{len(falhas)} falha(s)")
raise SystemExit(1 if falhas else 0)
