"""
Um quadro publicado por quem tem a câmera, para quem não tem.

O problema: `coletar.py` e `app.py` abriam a câmera cada um por conta. Só um
processo consegue — o segundo morre com "Pipeline handler in use by another
process". E os dois juntos é exatamente o que se quer na apresentação: o
braço coletando e o painel mostrando o que a IA está vendo.

A saída é não disputar. Quem roda a coleta é dono da câmera e PUBLICA cada
quadro já anotado; o painel LÊ o que foi publicado em vez de abrir a câmera.
Se não houver publicação recente, o painel abre a câmera normalmente — assim
ele continua servindo sozinho, para enquadrar e conferir foco.

Onde: `/dev/shm`, que é RAM. Não é detalhe de desempenho — gravar um JPEG
por quadro no cartão SD por horas é desgaste à toa, e este projeto já perdeu
um cartão por corrupção.

A escrita é atômica (arquivo temporário e `os.replace`): sem isso o leitor
pega JPEG pela metade de vez em quando, e o que aparece é uma imagem
corrompida intermitente, que é um sintoma difícil de associar à causa.
"""

from __future__ import annotations

import json
import os
import tempfile
import time
from pathlib import Path

import cv2


def _diretorio() -> Path:
    """RAM quando existe (Linux), temporário do sistema fora dele."""
    shm = Path("/dev/shm")
    base = shm if shm.is_dir() else Path(tempfile.gettempdir())
    d = base / "braco-separador"
    d.mkdir(parents=True, exist_ok=True)
    return d


def _caminhos():
    d = _diretorio()
    return d / "quadro.jpg", d / "estado.json"


def publicar(frame, meta: dict | None = None, qualidade: int = 80) -> bool:
    """
    Publica o quadro e um punhado de metadados. True se conseguiu.

    Nunca levanta: publicar é acessório, e a coleta não pode parar porque o
    painel ficou sem imagem.
    """
    if frame is None:
        return False
    img, est = _caminhos()
    try:
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, qualidade])
        if not ok:
            return False

        tmp = img.with_suffix(".jpg.tmp")
        tmp.write_bytes(buf.tobytes())
        os.replace(tmp, img)

        dados = dict(meta or {})
        dados["t"] = time.time()
        tmp = est.with_suffix(".json.tmp")
        tmp.write_text(json.dumps(dados), encoding="utf-8")
        os.replace(tmp, est)
        return True
    except OSError:
        return False


def ler(idade_maxima_s: float = 3.0):
    """
    Devolve (frame, meta) do último quadro publicado, ou None.

    None quando não há publicação, quando ela está velha demais, ou quando o
    arquivo não pôde ser lido. A idade máxima existe para o painel não
    mostrar indefinidamente a última imagem de um `coletar.py` que já morreu:
    imagem parada e antiga é pior que imagem nenhuma, porque parece ao vivo.
    """
    img, est = _caminhos()
    try:
        meta = json.loads(est.read_text(encoding="utf-8"))
        idade = time.time() - float(meta.get("t", 0))
        # Idade negativa é publicação com carimbo no futuro, e acontece de
        # verdade: o Pi não tem relógio de bateria, começa numa data velha e
        # salta quando o NTP sincroniza. Sem este teste, um quadro carimbado
        # depois do salto ficaria "fresco" para sempre e o painel mostraria
        # uma imagem congelada como se fosse ao vivo.
        if idade < 0 or idade > idade_maxima_s:
            return None
        frame = cv2.imread(str(img))
        if frame is None:
            return None
        return frame, meta
    except (OSError, ValueError, TypeError):
        return None


def limpar():
    """Apaga a publicação. Use ao encerrar, para o painel não mostrar sobra."""
    for c in _caminhos():
        try:
            c.unlink()
        except OSError:
            pass
