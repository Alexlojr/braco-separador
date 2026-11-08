#!/usr/bin/env python3
"""
Sequência de pega e solta em ÂNGULOS DE SERVO, sem cinemática.

Para que serve: demonstrar o braço pegando e soltando ANTES de a calibração
ficar pronta. Não usa offset, sentido, nem elo nenhum — os ângulos aqui são
os que o servo recebe, os mesmos que você digitaria num `SET`. Então isto
funciona hoje, e continua funcionando depois como teste de bancada.

    python pegada_manual.py --dry-run      # só imprime, não abre a serial
    python pegada_manual.py --passo        # espera Enter entre as etapas
    python pegada_manual.py --porta COM5

POR QUE ISTO EXISTE EM VEZ DE COLAR OS COMANDOS NO MONITOR SERIAL
-----------------------------------------------------------------
O firmware agrupa em um LOTE todo comando que chega dentro de 50 ms
(`JANELA_LOTE_MS`), e todos os servos do lote recebem a MESMA duração: a do
movimento mais longo. Isso é proposital — serve para as juntas chegarem
juntas numa pose.

Só que quando se cola um bloco de linhas no monitor serial, todas chegam no
mesmo instante, caem no mesmo lote, e a garra herda a duração do ombro. Um
fechamento de 40° esticado para os 4 s de um giro grande de ombro vira
10 °/s — abaixo da zona morta do MG90S. A garra trava e parece que o comando
não chegou.

Por isso aqui cada etapa é enviada sozinha e o script ESPERA o braço parar
(perguntando `BUSY`) antes da próxima. A garra nunca divide etapa com uma
junta do braço.
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from src.arm.mock_arm_serial import MockArmSerial, detectar_porta_arduino

# ---------------------------------------------------------------------------
# Limites do firmware. Repetidos aqui de propósito: é melhor o script recusar
# um ângulo em terra do que o firmware silenciosamente saturar no ar.
# A FONTE e o lim_min/lim_max do braco_firmware.ino: foi com ele que o braco
# foi operado na bancada. O braco.json e esta tabela copiam de la.
# testa_limites_sincronizados() em tests/test_firmware.py FALHA se divergirem.
# ---------------------------------------------------------------------------
LIMITES = {
    0: (0, 180),      # base
    1: (90, 180),     # ombro
    2: (70, 160),     # cotovelo
    3: (100, 140),   # garra
}
NOMES = {0: "base", 1: "ombro", 2: "cotovelo", 3: "garra"}

GARRA_ABERTA = 105
GARRA_FECHADA = 140

# ---------------------------------------------------------------------------
# A SEQUÊNCIA. É isto que você edita.
#
# Cada etapa é (descrição, {canal: ângulo}). Os ângulos abaixo são CHUTES
# iniciais dentro dos limites conhecidos — ajuste olhando o braço, com
# --passo, antes de gravar qualquer coisa.
#
# Duas regras que valem a pena não quebrar:
#   1. a garra sempre sozinha na etapa dela;
#   2. recolher antes de girar a base. Girar esticado é o caso em que o
#      ombro está no maior torque, que é exatamente quando o servo reclama.
# ---------------------------------------------------------------------------
SEQUENCIA = [
    ("Preparo: recolhe e abre a garra", {1: 130, 2: 140}),
    ("Abre a garra",                    {3: GARRA_ABERTA}),
    ("Gira a base para a peça",         {0: 90}),
    ("Estende por cima da peça",        {1: 160, 2: 120}),
    ("Desce até a peça",                {1: 170, 2: 110}),
    ("FECHA a garra",                   {3: GARRA_FECHADA}),
    ("Levanta com a peça",              {1: 150, 2: 130}),
    ("Recolhe para girar",              {1: 130, 2: 140}),
    ("Gira a base para o depósito",     {0: 40}),
    ("Estende sobre o depósito",        {1: 160, 2: 120}),
    ("ABRE a garra, solta a peça",      {3: GARRA_ABERTA}),
    ("Recolhe",                         {1: 130, 2: 140}),
    ("Volta ao repouso",                {0: 90, 1: 140, 2: 120}),
]

VELOCIDADE = 40.0   # °/s. Abaixo de ~23 o movimento entra na zona morta
                    # do servo e vira solavanco. 40-60 é confortável.
ASSENTAR_S = 0.4    # pausa depois de parar: o servo para no ângulo, mas o
                    # acrílico ainda balança. Importa antes de fechar a garra.


def valida(sequencia):
    """Recusa a sequência inteira antes de mover qualquer coisa."""
    erros = []
    for i, (desc, alvos) in enumerate(sequencia, start=1):
        for canal, ang in alvos.items():
            if canal not in LIMITES:
                erros.append(f"etapa {i} ({desc}): canal {canal} não existe")
                continue
            lo, hi = LIMITES[canal]
            if not lo <= ang <= hi:
                erros.append(
                    f"etapa {i} ({desc}): {NOMES[canal]} em {ang}°, "
                    f"fora do limite [{lo}, {hi}]"
                )
        canais = set(alvos)
        if 3 in canais and len(canais) > 1:
            erros.append(
                f"etapa {i} ({desc}): a garra está junto com outra junta. "
                "Separe em duas etapas — senão ela herda a duração do braço."
            )
    return erros


def main():
    ap = argparse.ArgumentParser(description="Pega e solta em ângulos de servo")
    ap.add_argument("--dry-run", action="store_true",
                    help="imprime a sequência e sai, sem abrir a serial")
    ap.add_argument("--passo", action="store_true",
                    help="espera Enter antes de cada etapa")
    ap.add_argument("--porta", default=None,
                    help="porta serial (COM5, /dev/ttyUSB0). Detecta sozinho se omitir")
    ap.add_argument("--velocidade", type=float, default=VELOCIDADE)
    args = ap.parse_args()

    erros = valida(SEQUENCIA)
    if erros:
        print("Sequência recusada, nada foi enviado:\n")
        for e in erros:
            print("  -", e)
        return 1

    print(f"{len(SEQUENCIA)} etapas, velocidade {args.velocidade:.0f} °/s\n")
    for i, (desc, alvos) in enumerate(SEQUENCIA, start=1):
        cmds = " ".join(f"SET {c} {a}" for c, a in sorted(alvos.items()))
        print(f"  {i:2d}. {desc:34s} {cmds}")

    if args.dry_run:
        print("\n--dry-run: nada foi enviado.")
        return 0

    porta = args.porta or detectar_porta_arduino()
    if porta is None:
        print("\nNenhuma porta de Arduino encontrada. Passe --porta COM5.")
        return 1

    ser = MockArmSerial(porta_arduino=porta)
    if not ser.braco_real:
        print(f"\nNão consegui abrir {porta}. O monitor serial da IDE está aberto?")
        return 1

    print(f"\nConectado em {porta}.\n")

    def manda(linha):
        ser.write((linha + "\n").encode())

    manda(f"SPEED {args.velocidade:.0f}")
    time.sleep(0.1)

    try:
        for i, (desc, alvos) in enumerate(SEQUENCIA, start=1):
            if args.passo:
                try:
                    input(f"[Enter] etapa {i}: {desc} ")
                except EOFError:
                    print("\n(sem terminal interativo; seguindo sozinho)")
                    args.passo = False
            else:
                print(f"  {i:2d}. {desc}")

            for canal, ang in sorted(alvos.items()):
                manda(f"SET {canal} {ang}")

            if not ser.aguardar_parada(timeout_s=15.0):
                print("\n  !! o braço não avisou que parou em 15 s. Parando aqui.")
                manda("STOP")
                return 1

            time.sleep(ASSENTAR_S)

            # Depois de fechar, vale ver quanto a garra está puxando: é o
            # número que falta para configurar o limite_stall_amp.
            if alvos.get(3) == GARRA_FECHADA:
                corrente = getattr(ser, "corrente", None)
                if callable(corrente):
                    leitura = corrente()
                    if leitura is not None:
                        print(f"      corrente na garra fechada: {leitura}")

        print("\nSequência concluída.")
        return 0

    except KeyboardInterrupt:
        print("\nInterrompido. Mandando STOP.")
        manda("STOP")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
