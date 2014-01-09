# Variable que identifica todos los parametros necesarios
# para definir una pisada por completo

# Identificacion de pata
uint8 Npata
# Variables en el plano
float32 coordenada_y
float32 coordenada_x
float32 coordenada_z
# Variables en el mapa
uint8[] coordenada_i
uint8[] coordenada_j
# Variables para calculo de trayectoria
float32   landa
float32   dh
float32[] p_Offset
float32[] alfa
float32[] desfasaje_t

