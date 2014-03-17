# datos_trayectoria_Pata.msg
# Mensaje para variables principales de trayectoria de pata

uint8     iteracion

#Tripode al que pertenece la pata
uint8     Tripode

#Periodo de paso
float32[] T_apoyo

#delta de tiempo de trayectoria (entre 0-1)
float32[]   t_Trayectoria

#Npuntos de trayectorias
float32[]   divisionTrayectoriaPata

#Desfasaje en tiempo de pata
float32[] desfasaje_t

#Factor de apoyo
float32   beta

#Distancia XY a recorrer en apoyo
float32[] lambda_Apoyo

#Distancia XY a recorrer en transferencia
float32[] lambda_Transferencia

#Velocidad de apoyo
float32    velocidad_Apoyo

#Vector de estados: 0-apoyo, 1-transferencia
uint8[] vector_estados

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
