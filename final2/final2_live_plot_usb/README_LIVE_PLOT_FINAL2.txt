FINAL2 LIVE PLOT MATPLOTLIB

Sirve para ver grafico EN VIVO:
- Izquierda: trayectoria por /odom_raw
- Derecha: radar LiDAR /scan
- Marca intersecciones/aperturas
- Marca PARE si existe /pare_detectado

PASOS:

1) Copia esta carpeta al USB dentro de:
   KINGSTON/final2/final2_live_plot_usb/

2) En Raspberry, FUERA del Docker, corre:
   bash /media/pi/KINGSTON/final2/final2_live_plot_usb/instalar_live_plot_final2.sh

3) Entra al Docker:
   docker exec -it thirsty_cori bash

4) Activa ROS:
   source /opt/ros/humble/setup.bash
   source /root/yahboomcar_ws/install/setup.bash
   export ROS_DOMAIN_ID=20

5) Terminal 1, NO MUEVE:
   python3 /root/final2/live_plot_final2.py

6) Terminal 2, si quieres PARE, NO MUEVE:
   python3 /root/final2/pare_detector.py

7) Terminal 3, MUEVE:
   /root/VERSIONES_CASA/versionfinal1/run_versionfinal1.sh

8) Al terminar, CTRL+C en live_plot_final2.py.
   Guarda en:
   /root/final2/evidencias/live_plot_final2_final.png
   /root/final2/evidencias/live_plot_final2_final.svg
   /root/final2/evidencias/live_plot_final2_trayectoria.csv

9) Para copiar al USB, FUERA del Docker:
   mkdir -p /media/pi/KINGSTON/final2/evidencias
   docker cp thirsty_cori:/root/final2/evidencias/. /media/pi/KINGSTON/final2/evidencias/
   sync

NOTA:
Si la ventana no aparece, igual se va guardando:
   /root/final2/evidencias/live_plot_final2.png
cada 2 segundos.
