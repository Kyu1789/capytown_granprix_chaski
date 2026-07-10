#!/bin/bash
set -e
BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "[FINAL2] Copiando archivos desde: $BASE_DIR"
install -m 755 "$BASE_DIR/rescate_chaski.py" /root/rescate_chaski.py
install -m 755 "$BASE_DIR/run_rescate_corridor.sh" /root/run_rescate_corridor.sh
install -m 755 "$BASE_DIR/run_rescate_ruta.sh" /root/run_rescate_ruta.sh

python3 -m py_compile /root/rescate_chaski.py

echo "[FINAL2] Instalacion correcta."
echo "Para ejecutar el modo autonomo de pasillo:"
echo "  /root/run_rescate_corridor.sh"
echo "Para detenerlo: Ctrl+C"
