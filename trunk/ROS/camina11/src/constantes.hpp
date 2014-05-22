/******************************************************************
Header que incluye constantes y demas valores globales para la aplicacion
*******************************************************************/

/******************************************************************
Lynxmotion: 
-Longitud de eslabones del robot. Dimensiones - metros (m)
-Masa de eslabones del robot. Dimensiones - Kilogramos (Kg)
 ..recordar que no existe un L1 como tal, es la unión de los motores 1 y 2
*******************************************************************/
#ifndef __CONSTANTES_HPP__
#define __CONSTANTES_HPP__

//includes
#include <stdio.h>
#include "camina11/vector3d.hpp"

//ID patas
#define Pata1 1
#define Pata2 2
#define Pata3 3
#define Pata4 4
#define Pata5 5
#define Pata6 6
//Longitud de eslabones
#define L1 0.0280
#define L2 0.0595
//#define L3 0.1076
#define L3 0.1112
//Propiedades de cuerpo
#define anguloPatas 60*pi/180
#define radioCuerpo 0.14
#define posicionPata1_x radioCuerpo*cos(anguloPatas)
#define posicionPata1_y radioCuerpo*sin(anguloPatas)
#define posicionPata2_x radioCuerpo
#define posicionPata2_y 0
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
#define lambda_maximo 0.09
#define lambda_minimo 0.015
#define teta_Offset -30*pi/180
#define umbralFuerzaApoyo 0.009
#define umbral_Z_Apoyo 0.0045
#define vel_teorica 0.0225
#define vel_esperada 0.018
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
#define TrayInicio -0.04
#define EspacioTrabajo_X1 -0.04
#define EspacioTrabajo_X2 0.04
#define EspacioTrabajo_Y1 0.06
#define EspacioTrabajo_Y2 -0.05
#define Npuntos 4
#define FinEspacioTrabajo_y 0.055
//Tranformaciones
#define Mundo_Pata 0
#define Pata_Mundo 1
//Tripodes
#define T1 1
#define T2 2
//Estados
#define Transferencia 0
#define Apoyo 1
#define EsperaTransferencia 2
#define EsperaApoyo 3
//Correccion
#define Correccion_menosX 0
#define Correccion_masX	1

//float rotacionPata[Npatas];
//punto3d origenPatas[Npatas];
typedef struct {
		int i;		//Coordenadas matriz
		int j;
		punto3d O;	//Origen cartesiano
		punto3d P1;	//Punto arriba-izquierda
		punto3d P2;	//Punto arriba-derecha
		punto3d P3;	//Punto abajo-izquierda
		punto3d P4;	//Punto abajo-derecha
		}Obstaculo;
//--Definiciones de funciones
//void transformacion_yxTOij(int *ptr_ij, float y, float x, int nCeldas_i, int nCeldas_j, float LongitudCeldaY,float LongitudCeldaX);
punto3d TransportaPunto(punto3d Punto_in,float L_traslacion, float ang_rotacion);
punto3d TransformacionHomogenea(punto3d Punto_in, punto3d L_traslacion, float ang_rotacion);
punto3d TransformacionHomogenea_Inversa(punto3d Punto_in, punto3d L_traslacion, float ang_rotacion);
punto3d Transformada_Mundo_Pata(int nPata, int modo, punto3d P_in, punto3d P_ubicacionRobot, float theta_Rob);
void EspacioDeTrabajo_Robot(int nPata, punto3d *ptr_EDT, float phi, punto3d Offset);

//#else
//extern float rotacionPata[Npatas];
//extern punto3d rotacionPata[Npatas];
#endif // __CONSTANTES_HPP__
