#!/usr/bin/env bash
# Ejecutar desde la Raspberry, FUERA del Docker:
# bash /media/pi/KINGSTON/final2/final2_live_plot_usb/instalar_live_plot_final2.sh

set -e

CONTAINER=${1:-thirsty_cori}
USB_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "[LIVEPLOT] Instalando en Docker: $CONTAINER"
docker start "$CONTAINER" >/dev/null 2>&1 || true

docker exec "$CONTAINER" mkdir -p /root/final2/evidencias
docker cp "$USB_DIR/live_plot_final2.py" "$CONTAINER":/root/final2/live_plot_final2.py
docker exec "$CONTAINER" chmod +x /root/final2/live_plot_final2.py
docker exec "$CONTAINER" python3 -m py_compile /root/final2/live_plot_final2.py

echo "[LIVEPLOT] Instalado OK."
echo ""
echo "Para correr dentro del Docker, NO MUEVE:"
echo "  python3 /root/final2/live_plot_final2.py"
