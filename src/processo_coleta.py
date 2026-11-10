"""
Iniciar e parar o coletar.py a partir do painel.

A coleta continua sendo um processo próprio, exatamente o mesmo da linha de
comando: o painel só a lança e, para parar, manda o mesmo Ctrl+C (SIGINT) que
o terminal mandaria. Assim o encerramento passa pelo `finally` do coletar.py
— resumo de latência, câmera fechada, publicação apagada — em vez de matar o
processo no meio de um movimento.

O identificador do processo fica numa variável deste módulo. O Streamlit
reexecuta o script do painel a cada clique, mas módulos importados persistem
entre as reexecuções, então o processo não se perde.

A saída do terminal da coleta vai para um arquivo em RAM, ao lado da
publicação do quadro. O log do painel já mostra as mensagens normais; este
arquivo serve para o caso em que a coleta morre ANTES de começar a publicar
(Arduino desconectado, calibração ausente), quando não haveria nada na tela.
"""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

_PROCESSO: subprocess.Popen | None = None
_ARQ_SAIDA = None


def _diretorio() -> Path:
    shm = Path("/dev/shm")
    base = shm if shm.is_dir() else Path(tempfile.gettempdir())
    d = base / "braco-separador"
    d.mkdir(parents=True, exist_ok=True)
    return d


def caminho_saida() -> Path:
    return _diretorio() / "coleta_terminal.log"


def rodando() -> bool:
    return _PROCESSO is not None and _PROCESSO.poll() is None


def codigo_saida():
    """None se nunca rodou ou ainda roda; o código de saída se terminou."""
    if _PROCESSO is None:
        return None
    return _PROCESSO.poll()


def iniciar(base_dir: Path, argumentos: list[str]) -> bool:
    """Lança `python coletar.py <argumentos>`. False se já havia uma rodando."""
    global _PROCESSO, _ARQ_SAIDA
    if rodando():
        return False
    if _ARQ_SAIDA is not None:
        _ARQ_SAIDA.close()
    _ARQ_SAIDA = open(caminho_saida(), "w", encoding="utf-8", buffering=1)
    ambiente = dict(os.environ, PYTHONUNBUFFERED="1", PYTHONIOENCODING="utf-8")
    _PROCESSO = subprocess.Popen(
        [sys.executable, str(Path(base_dir) / "coletar.py"), *argumentos],
        cwd=str(base_dir),
        stdout=_ARQ_SAIDA,
        stderr=subprocess.STDOUT,
        stdin=subprocess.DEVNULL,
        env=ambiente,
    )
    return True


def parar(espera_s: float = 8.0) -> None:
    """Ctrl+C na coleta; se ela não sair a tempo, termina à força."""
    if not rodando():
        return
    try:
        if os.name == "posix":
            _PROCESSO.send_signal(signal.SIGINT)
        else:  # fora do Pi, só para testes
            _PROCESSO.terminate()
        _PROCESSO.wait(timeout=espera_s)
    except subprocess.TimeoutExpired:
        _PROCESSO.kill()
        _PROCESSO.wait(timeout=3)


def ultimas_linhas(n: int = 15) -> list[str]:
    try:
        texto = caminho_saida().read_text(encoding="utf-8", errors="replace")
    except OSError:
        return []
    return [l for l in texto.splitlines() if l.strip()][-n:]
