# Este mensaje contiene toda la informacion de la ubicacion
# del robot en el espacio
# Las dimensiones se manejan en metros

#Cuerpo
float32 coordenadaCuerpo_x
float32 coordenadaCuerpo_y
float32 coordenadaCuerpo_z
float32 orientacionCuerpo_roll
float32 orientacionCuerpo_pitch
float32 orientacionCuerpo_yaw
float32 velocidadCuerpo_x
float32 velocidadCuerpo_y
float32 velocidadCuerpo_z
#patas
uint8 NPata
float32[] coordenadaPata_x
float32[] coordenadaPata_y
float32[] coordenadaPata_z
float32[] coordenadaPataSistemaPata_x
float32[] coordenadaPataSistemaPata_y
float32[] coordenadaPataSistemaPata_z
float32[] coordenadaMotor1Pata_x
float32[] coordenadaMotor1Pata_y
float32[] coordenadaMotor1Pata_z
#fuerza patas
float32[]  pataTipFuerza_x
float32[]  pataTipFuerza_y
float32[]  pataTipFuerza_z
uint8[]  pataApoyo
#Centro de Masa
float32 centroMasaCuerpo_x
float32 centroMasaCuerpo_y
float32 centroMasaCuerpo_z
float32[] centroMasaPata_x
float32[] centroMasaPata_y
float32[] centroMasaPata_z
