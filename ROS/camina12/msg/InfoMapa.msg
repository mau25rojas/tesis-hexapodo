# Este mensaje contiene toda la informacion del mapa
# dividido en celdas. Contiene informacion de celdas
# vacias y celdas con obstaculos
# Las dimensiones se manejan en metros
# y->i ... x->j

#obstaculos
uint8 cantidadObstaculos
uint8[] coordenadaObstaculo_i
uint8[] coordenadaObstaculo_j
#patas
uint8 nPata
uint8[] coordenadaPata_i
uint8[] coordenadaPata_j
float32[] coordenadaPata_x
float32[] coordenadaPata_y
#ajuste
bool correccion
uint8[] coordenadaAjuste_i
uint8[] coordenadaAjuste_j

