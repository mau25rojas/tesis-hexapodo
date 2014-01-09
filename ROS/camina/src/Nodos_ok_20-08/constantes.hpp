/******************************************************************
Header que incluye constantes y demas valores globales para la aplicacion
*******************************************************************/

/******************************************************************
Lynxmotion: 
-Longitud de eslabones del robot. Dimensiones - metros (m)
-Masa de eslabones del robot. Dimensiones - Kilogramos (Kg)
 ..recordar que no existe un L1 como tal, es la unión de los motores 1 y 2
*******************************************************************/
//ID patas
#define Pata1 1
#define Pata2 2
#define Pata3 3
#define Pata4 4
#define Pata5 5
#define Pata6 6
//Rotacion patas
#define rotacion_Pata1 30*pi/180
#define rotacion_Pata2 -30*pi/180
#define rotacion_Pata3 90*pi/180
#define rotacion_Pata4 -90*pi/180
#define rotacion_Pata5 150*pi/180
#define rotacion_Pata6 -150*pi/180
//Longitud de eslabones
#define L1 0.052
#define L2 0.06
#define L3 0.105
//Propiedades de cuerpo
#define anguloPatas 60
#define radioCuerpo 0.14
//Masa de eslabones
#define masa_Base 0.66
#define masa_Motor 0.048
#define masa_L1 0.10799
#define masa_L2 0.02591
#define masa_L3 0.06771
//Centro de masa de eslabones
#define cm_L1 0.01453
#define cm_L2 0.02946
#define cm_L3 0.01792
//Constantes
#define pi 3.141592
#define Npatas 6
#define Neslabones 3
#define lambda_paso 0.01
#define lambda_maximo 0.05
#define teta_Offset -30
#define umbralFuerzaApoyo 0.02
//Constantes para envio serial
#define Nmotores 18
#define NmotorP 3
#define canal_Pata1 0
#define canal_Pata2 16
#define canal_Pata3 4
#define canal_Pata4 20
#define canal_Pata5 8
#define canal_Pata6 24
//Trayectoria ID
#define TrayectoriaCuadrada 1
#define TrayectoriaEliptica 2
//Espacio de Trabajo
#define EspacioTrabajo_Y 0.2
#define EspacioTrabajo_X 0.2	//Nota: el espacio de trabajo va hacia +-X, este valor representa la mitad
#define Npuntos 4
