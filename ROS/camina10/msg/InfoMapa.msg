# Este mensaje contiene toda la informacion del mapa
# dividido en celdas. Contiene informacion de celdas
# vacias y celdas con obstaculos
# Las dimensiones se manejan en metros
# y->i ... x->j

#total de celdas
uint8 nCeldas_i
uint8 nCeldas_j
uint8 iCelda
float32 tamanoCelda_i
float32 tamanoCelda_j
#obstaculos
uint8 cantidadObstaculos
uint8[] coordenadaObstaculo_i
uint8[] coordenadaObstaculo_j
#patas
uint8 NPata
uint8[] coordenadaPata_i
uint8[] coordenadaPata_j
float32[] coordenadaPata_x
float32[] coordenadaPata_y
#fuerza patas
uint8[]  pataApoyo
#ajuste
bool correccion
uint8[] coordenadaPreAjuste_i
uint8[] coordenadaPreAjuste_j
uint8[] coordenadaAjuste_i
uint8[] coordenadaAjuste_j

