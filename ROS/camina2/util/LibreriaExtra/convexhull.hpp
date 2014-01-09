/* -*- Mode: C; indent-tabs-mode: t; c-basic-offset: 4; tab-width: 4 -*- */
/*
 * convexhull.hpp
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

/* Libreria con implementaciones de Convex-Hull, basado en tipos de datos definidos en vector3d.cpp

	> Listado de algoritmos para calculo del Convex Hull
   - Graham Scan: 				O(n log n) - Plano
   - Gift wrapping / Jarvis	O(nh)		  - 3d?
   - Kirkpatrick-Seidel			O(n log h) - Plano
   - Chan							O(n log h) - 2D-3D
*/

#ifndef _CONVEXHULL_H_
#define _CONVEXHULL_H_

//#include "analisis.hpp"
//#include "vector3d.hpp"

/*
RECOMENDACIONES: revisar prototipos de funciones parciales de implementacion de convex hull
para mejorar nombres de funcion, y que los argumentos se pidan siempre en el mismo orden
(datos, numero de elementos, datos salida, etc)
*/

/*Graham Scan (2D)
	PRE: Z = 0, todos los puntos estan contenidos en el plano XY (caso estatico, donde solo existe aceleracion -gz)
	Pasos para calcular el convex Hull por Graham Scan en 2D:
   	i)Encontrar el punto más "bajo", si hay empate, se elige el más "izquierdo"
      ii)Ordenar de forma creciente los otros puntos, respecto al ángulo que forman con el primero
      	(en este caso es equivalente ordenar de forma decreciente por la cotangente que forman)
      iii)El segundo punto, el de mayor cotangente PERTENECE al cascarón
      iv)Se determina el ángulo formado entre los vectores de: los últimos dos puntos y el último con el nuevo
      	(ej: AB con BC), esto se puede hacer usando el producto cruz (fórmula equivalente: (Xab)(Ybc)-(Yab)(Xbc):
         -Si el resultado es 0, son colineales
         -Si el resultado es positivo, hay un giro a la izquierda
         -Si el resultado es negativo, hay un giro a la derecha
      v)En caso de tener giro a la derecha (o colineal) se elimina el último punto y se prueba con los dos anteriores
      vi)Al llegar a un giro a la izquierda (producto cruz positivo) se añade el punto nuevo como el último de la secuencia
      vii)El algoritmo termina cuando se llega al punto inicial
*/

// P: lista de puntos en el plano XY, Q: lista de puntos pertenecientes al convex hull, k, numero de puntos en P
int convexhull_graham (punto3d *P, punto3d *Q, int k);
int punto_convexhull2 (punto3d P, punto3d *hull, int k);		//Determina si el Punto P está contenido en el cascarón convexo definido por *hull.
int punto_convexhull (punto3d P, punto3d *hull, int k);			//Determina si el Punto P está contenido en el cascarón convexo definido por *hull
																						//usando el centroide para determinar si el punto esta del mismo lado.
float margen_est (punto3d P, punto3d *hull, int k);				//Determina el margen de estabilidad del punto P, respecto al cascaron hull de k puntos:
                                                               	//es 0, si el punto esta en el borde, positivo dentro y negativo fuera del convex hull.
punto3d punto_est (punto3d P, punto3d *hull, int k);			//Determina el punto más cercano del borde del cascarón correspondiente al margen de
																//estabilidad del punto P


#endif // _CONVEXHULL_H_
