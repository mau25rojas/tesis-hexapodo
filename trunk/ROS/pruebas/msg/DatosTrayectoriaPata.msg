# datos_trayectoria_Pata.msg
# Mensaje para variables principales de trayectoria de pata

uint8     iteracion

#Periodo de un paso
float32[]   T

#delta de tiempo de trayectoria entre [0-1]
float32[]   t_Trayectoria

#Npuntos de trayectorias
float32[]   divisionTrayectoriaPata

#Desfasaje en tiempo de pata
float32[] desfasaje_t

#Factor de apoyo
float32   beta

#Distancia XY a recorrer en apoyo y transferencia
float32[] lambda

#Vector de estados: 0-apoyo, 1-transferencia
uint8[] vector_estados
uint8[] cambio_estado

#Distancia en Z a recorrer en transferencia
float32   dh

#Offset para trayectoria visto desde la pata
float32   x_Offset
float32   y_Offset
float32   z_Offset

#Angulo de pata visto desde el robot
float32[] phi

#Angulo de giro de trayectoria
float32[]   alfa

#Bandera de cambio en trayectoria
bool      cambioTrayectoria

#Distancia de correccion en Y para traslacion de tray
float32[] correccion_y	#La correccion en y siempre es hacia -y
float32[] correccion_x
int32[] correccion_ID	#ID de la correccion_x: (-1)no hay corr;(0)corr_izq-> -x;(1)corr_der-> +x;

#Informacion de posicion de patas
float32[] posicionActualSistemaPata_x
float32[] posicionActualSistemaPata_y
float32[] posicionActualSistemaPata_z
