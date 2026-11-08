#!/usr/bin/env bash
# Sobe o painel de monitoramento.
#
# Separado do setup.sh de propósito: o painel abre a câmera quando não há
# coleta rodando, e um script de instalação não deve tomar um recurso
# exclusivo sem ninguém pedir.
#
# Pode rodar junto com o coletar.py. Nesse caso o painel não abre câmera
# nenhuma: ele mostra os quadros que a coleta publica.
set -euo pipefail
cd "$(dirname "$0")"

[ -d venv ] || { echo "venv não existe. Rode ./setup.sh antes."; exit 1; }
source venv/bin/activate

IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
PORTA=8501
[ -n "$IP" ] && echo "==> http://${IP}:${PORTA}"

exec streamlit run app.py \
    --server.headless true \
    --server.address 0.0.0.0 \
    --server.port "$PORTA"
