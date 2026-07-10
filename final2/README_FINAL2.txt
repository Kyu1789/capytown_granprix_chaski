FINAL2 - PRIMERA INTERSECCION A LA IZQUIERDA

Archivos:
- rescate_chaski.py              controlador modificado
- run_rescate_corridor.sh        comando recomendado para el laberinto
- run_rescate_ruta.sh            ruta temporizada original
- instalar_final2.sh             instala todo en /root

CAMBIO REALIZADO
- En mode=corridor, la primera decision de giro se fuerza a la IZQUIERDA.
- La direccion izquierda se conserva hasta que el frente quede libre.
- Despues del primer giro, vuelve el comportamiento original: lado mas libre / hand=left.
- PARE conserva prioridad.

COMO INSTALAR DESDE EL USB EN LA RASPBERRY
1. Busca la carpeta:
   find /media /mnt -maxdepth 5 -type d -iname final2 2>/dev/null

2. Ejecuta el instalador usando la ruta que aparezca. Ejemplo:
   bash /media/pi/KINGSTON/final2/instalar_final2.sh

3. Inicia el robot:
   /root/run_rescate_corridor.sh

4. Para verificar el estado desde otra terminal:
   export ROS_DOMAIN_ID=20
   ros2 topic echo /maze/metricas

En el registro debe aparecer:
FINAL2: PRIMERA INTERSECCION/DECISION FORZADA A IZQUIERDA

PARADA DE EMERGENCIA
Ctrl+C y luego:
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
