/* -*- Mode: C; indent-tabs-mode: t; c-basic-offset: 4; tab-width: 4 -*- */
/*
 * analisis.hpp
 * Copyright (C) Jose Cappelletto, Dimitar Ralev 2009 <>
 *
 * convexhull is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * convexhull is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* Libreria con la implementaciones de funciones necesarias para hacer
	diferentes tipos de análisis empleando datos definidos en vector3d.cpp

*/

#ifndef _ANALISIS_H_
#define _ANALISIS_H_

//#include "vector3d.hpp"
//#include <mem.h>
punto3d rot_punto_ea(vector3d, float, punto3d); 	//rotación de un punto alrededor de un vector
plano3d rot_plano_vector(vector3d, float, plano3d);		//rotación de un plano alrededor de un vector
punto3d rot_punto_recta(recta3d, float, punto3d); 			//rotación de un punto alrededor de un recta
plano3d rot_plano_recta(recta3d, float, plano3d); 			//rotación de un plano alrededor alrededor de un recta
int puntoMasLejano (vector3d v, punto3d *P, int k);
//fn que devuelve el indice (o el punto3d?) del punto mas lejano en la direccion de un vector v,
//dado un set de puntos P, de tamaño k
plano3d planoProyeccion (vector3d v, punto3d *P, int k);
//fn que devuelve el plano normal a v, que pasa por el punto mas lejano en la
//direccion de v, dado un set P[k]
plano3d planoProyectar (vector3d v, punto3d *P, punto3d *Q, int k);
//fn que devuelve el plano normal a v, que pasa por el punto mas lejano en la
//direccion de v, dado un set P[k], y calcula las proyecciones de Q[k]=P[k] sobre PLANO
void planoProyectarXY (vector3d v, punto3d *P, punto3d *Q, int k);
//fn que calcula las proyecciones de los puntos P sobre un plano normal a v, y que pasa por el
//punto mas lejano de esa dirección, para luego girar el conjunto completo al plano XY

int punto_pivote(punto3d *P, int k);								//P es la lista de "k" puntos a ordenar, dando como resultado el índice del mismo
void BubbleSort (float *d, int *indice, int k); 				//Bubble Sort de un arreglo de float (d) de tamaño k, devuelve el arreglo ordenado indices
void sort_by_angle(punto3d *P, int k, int pivote, int *Q);	//P es la lista de "k" puntos a ordenar, con "pivote" el índice del pivote, y Q el arreglo de índices ordenados
punto3d centroide(punto3d *P, int k);								//Determina el centroide de un arreglo P de k puntos

//funciones empleadas para visualizar las coordenadas de diferentes elementos
void printfvector3d(char *, vector3d);
void printfrecta3d(char *, recta3d);
void printfplano3d(char *, plano3d);

#endif // _ANALISIS_H_
