/*
 *  Cinematicas.hpp
 *
 *
 *  hecho por NikiTa
 *  Copyright 2011
 *
 *  Modificado y adaptado para usa juntal Cinempaticas.cpp
 */

/*Declaracion de funciones*/

#ifndef _CINEMATICAS_H_
#define _CINEMATICAS_H_

//#include "Parametros.h"

/*******FUNCIONES USADAS PARA CALCULAR TRAYECTORIAS Y CINEM�TICAS*******/
void Cin_directa(int P, Articulaciones ang, punto3d* punto);               //Cinem�tica directa de una pata P del cuadr�pedo
void Cin_Inversa(int P, punto3d PuntoVistoCM, Articulaciones* sol1);        //Cinem�tica inversa de una pata P del cuadr�pedo
void Tray_Rect(Const_caminado consta, float t, int pata, punto3d* trayect); //Generador de un punto de una trayectoria rectangular para un instnate 0<=t<=1
void Tray_Trian(Const_caminado consta, float t, int pata, punto3d* trayect);//Generador de un punto de una trayectoria triangular para un instante 0<=t<=1
void Tray_Elip(Const_caminado consta, float t, int pata, punto3d* trayect); //Generador de un punto de una trayectoria el�ptica para un instante 0<=t<=1

/*******FUNCIONES USADAS POR LOS SENSORES*******/
void det_colision(int pata, float t, float beta, double fuerza);            //Detecta la presencia de un obst�culo (hueco o resalte) en la trayectoria

/*******FUNCIONES USADAS PARA EL C�LCULO DE CENTROS DE MASA*******/
void cm_eslabon(punto3d* centro, Articulaciones ang, int articulacion, float L);        //Calcula el centro de masa de un eslabon respecto a la base de la pata.
void cm_pata(punto3d* cm, punto3d esl1, punto3d esl2, punto3d esl3);                    //Calcula el centro de masa de la pata usando la funcion anterior.
void cm_cuad(punto3d* cm, punto3d pata1, punto3d pata2, punto3d pata3, punto3d pata4);  //Calcula el centro de masa del cuadrupedo usando la funcion anterior.

//void pos_servos(Articulaciones* ang, int pata);                          //Determina los �ngulos q1,q2,q3 reales de una pata del robot

// Calcula del margen de estabilidad de un cuadr�pedo, toma en cuenta si est�n apoyadas o no las 4 patas. Tambi�n devuuelve el centroide del pol�gono de apoyo.
float me_cuad(punto3d P, punto3d* centro, double f1, double f2, double f3, double f4, Articulaciones p1, Articulaciones p2, Articulaciones p3, Articulaciones p4);
// Determina el punto m�s cercano al pol�gono de apoyo correspondiente al margen de estabilidad del punto P
punto3d punto_me_cuad(punto3d P, double f1, double f2, double f3, double f4, Articulaciones p1, Articulaciones p2, Articulaciones p3, Articulaciones p4);
#endif // _CINEMATICAS_H_
