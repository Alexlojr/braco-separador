"""
Log e ângulos dos servos, publicados pela coleta para o painel.

Mesmo arranjo do `quadro_publicado`: quem roda a coleta é dono da serial, e o
painel não pode abrir a porta — o Arduino aceita uma conexão só, e a segunda
derrubaria a primeira no meio de um movimento. A coleta PUBLICA o estado num
arquivo em RAM; o painel LÊ.

O que vai para o painel:

- `log`: as últimas linhas que a coleta imprimiu no terminal (etapas, destino,
  latência). Captura o próprio `print`, e não uma lista paralela de mensagens,
  para o painel nunca contar uma história diferente da do terminal.
- `servos`: o ÚLTIMO ÂNGULO COMANDADO a cada junta, tirado dos `SET`/`HOME`
  que saem pela serial. Não é a posição medida — os servos não informam a
  posição (malha aberta) — e o painel diz isso. Durante um movimento, o servo
  ainda está a caminho do valor mostrado.

Nunca levanta exceção: o painel é acessório, e a coleta não pode parar porque
um arquivo não pôde ser escrito.
"""

from __future__ import annotations

import json
import os
import tempfile
import time
from collections import deque
from pathlib import Path

NOMES_PADRAO = {0: "Base", 1: "Ombro", 2: "Cotovelo", 3: "Garra"}
LINHAS_LOG = 60


def _caminho() -> Path:
    shm = Path("/dev/shm")
    base = shm if shm.is_dir() else Path(tempfile.gettempdir())
    d = base / "braco-separador"
    d.mkdir(parents=True, exist_ok=True)
    return d / "telemetria.json"


class Telemetria:
    def __init__(self, pose_repouso=None, nomes=None):
        self.nomes = dict(nomes or NOMES_PADRAO)
        self.pose_repouso = list(pose_repouso) if pose_repouso else None
        self.log = deque(maxlen=LINHAS_LOG)
        self.servos = {}
        if self.pose_repouso:
            # O firmware sobe na pose de repouso; até o primeiro comando, é o
            # melhor palpite que existe.
            for canal, ang in enumerate(self.pose_repouso):
                self.servos[canal] = {"ang": float(ang), "solto": False, "t": 0.0}
        self._pendente = ""

    # ------------------------------------------------------------ entradas
    def linha(self, texto: str):
        texto = texto.rstrip()
        if texto.strip():
            self.log.append(f"{time.strftime('%H:%M:%S')}  {texto}")
            self.publicar()

    def comando(self, linha: str):
        """Atualiza os ângulos a partir de um comando enviado pela serial."""
        partes = linha.split()
        if not partes:
            return
        cmd, agora = partes[0].upper(), time.time()
        mudou = False
        if cmd == "SET" and len(partes) >= 3:
            try:
                canal, ang = int(partes[1]), float(partes[2])
            except ValueError:
                return
            s = self.servos.setdefault(canal, {"ang": ang, "solto": False, "t": 0.0})
            s.update(ang=ang, t=agora)
            mudou = True
        elif cmd == "HOME" and self.pose_repouso:
            for canal, ang in enumerate(self.pose_repouso):
                s = self.servos.setdefault(canal, {"ang": ang, "solto": False, "t": 0.0})
                s.update(ang=float(ang), t=agora, solto=False)
            mudou = True
        elif cmd in ("DETACH", "ATTACH") and len(partes) >= 2:
            alvo = partes[1].upper()
            canais = list(self.servos) if alvo == "ALL" else []
            if alvo != "ALL":
                try:
                    canais = [int(alvo)]
                except ValueError:
                    return
            for canal in canais:
                if canal in self.servos:
                    self.servos[canal]["solto"] = (cmd == "DETACH")
            mudou = True
        if mudou:
            self.publicar()

    # ------------------------------------------------------------ saída
    def publicar(self):
        dados = {
            "t": time.time(),
            "log": list(self.log),
            "servos": [
                {"canal": c, "nome": self.nomes.get(c, f"Canal {c}"), **s}
                for c, s in sorted(self.servos.items())
            ],
        }
        destino = _caminho()
        try:
            tmp = destino.with_suffix(".json.tmp")
            tmp.write_text(json.dumps(dados, ensure_ascii=False), encoding="utf-8")
            os.replace(tmp, destino)
        except OSError:
            pass

    def espelhar(self, saida):
        """Um objeto tipo arquivo que escreve em `saida` E alimenta o log."""
        return _Espelho(saida, self)

    @staticmethod
    def limpar():
        try:
            _caminho().unlink()
        except OSError:
            pass


class _Espelho:
    def __init__(self, saida, tele: Telemetria):
        self._saida = saida
        self._tele = tele
        self._buf = ""

    def write(self, s):
        n = self._saida.write(s)
        try:
            self._buf += s
            while "\n" in self._buf:
                linha, self._buf = self._buf.split("\n", 1)
                self._tele.linha(linha)
        except Exception:
            pass
        return n

    def flush(self):
        self._saida.flush()

    def __getattr__(self, nome):
        return getattr(self._saida, nome)


def ler():
    """
    Último estado publicado, com a idade em segundos, ou None.

    Não descarta por idade: entre um ciclo e outro a coleta pode passar
    vários segundos sem imprimir nada, e o log continua valendo. Quem decide
    se a coleta está viva é o painel, pelo quadro publicado.
    """
    try:
        dados = json.loads(_caminho().read_text(encoding="utf-8"))
        idade = time.time() - float(dados.get("t", 0))
        # Idade negativa: relógio do Pi saltou no NTP (ver quadro_publicado).
        if idade < 0:
            return None
        dados["idade"] = idade
        return dados
    except (OSError, ValueError, TypeError):
        return None
