/* -*- Mode: C; indent-tabs-mode: t; c-basic-offset: 4; tab-width: 4 -*- */
/*
 * convexhull.cpp
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
   - Kirkpatrick–Seidel			O(n log h) - Plano
   - Chan							O(n log h) - 2D-3D
*/


#include <math.h>
#include <stdlib.h>

//#include "vector3d.hpp"
//#include "analisis.hpp"
#include "convexhull.hpp"

#define MACH_EPS 1E-19
//#define MACH_EPS 1E-15
#define min(X,Y) ((X) < (Y) ? (X) : (Y))

int convexhull_graham (punto3d *P, punto3d *Q, int k)
{
	int pivote = punto_pivote(P, k);	//Primero se busca le punto más bajo, si hay empate es el de la izquierda
	int *indice_ch = new int[k+1];
	sort_by_angle(P, k, pivote, indice_ch);	//Se ordenan de forma creciente por el ángulo
   int i, M = 2;
   indice_ch[k] = indice_ch[0];
	for (i=2; i<=k; i++)		  		//Se proceden a calcular los giros para cada nuevo punto
   {

   	while ((P[indice_ch[M-1]].x-P[indice_ch[M-2]].x)*(P[indice_ch[i]].y-P[indice_ch[M-2]].y)-(P[indice_ch[M-1]].y-P[indice_ch[M-2]].y)*(P[indice_ch[i]].x-P[indice_ch[M-2]].x) <= 0)
      	M--;
		M++;
      indice_ch[M-1] = indice_ch[i];
   }
   M--;
	for (i=0; i<M; i++)
   {
   	Q[i] = P[indice_ch[i]];
   }
   delete indice_ch;
   planoProyectarXY (vector3d(0,0,1), Q, Q, M); //Ponemos los puntos del ConvexHull coplanares
   return M;
}

int punto_convexhull2 (punto3d P, punto3d *hull, int k)
{
	punto3d *arreglo = new punto3d[k+1];
   memcpy((void*)arreglo, (void*)hull, sizeof(punto3d)*(k+1));
	arreglo[k] = P;
   int puntos = convexhull_graham(arreglo, arreglo, (k+1));
   if (puntos == (k+1))
   {
      delete arreglo;
   	return (0);    			//El cascarón nuevo tiene un punto más que el cascarón original, el nuevo punto estaba fuera del original
   }
	register int i;
	for (i=0; i<puntos; i++)
   {
   	if((arreglo[i].x == P.x)&&(arreglo[i].y == P.y))
      {
			delete arreglo;
      	return (0);				//El punto pertenece al nuevo cascarón, es decir, estaba fuera del original
      }
   }
	delete arreglo;
   return (1);		            //El cascarón no cambia, el punto estaba en su interior
}

int punto_convexhull (punto3d punto, punto3d *hull, int k)
{
	register int i;
   punto3d centro = centroide(hull, k);
   punto3d P = punto3d(punto.x, punto.y, centro.z);
   for (i=0; i<k; i++)
   {
   	if((((centro-hull[i])^(hull[(i+1)%k]-hull[i])).z>0)!=(((P-hull[i])^(hull[(i+1)%k]-hull[i])).z>0)) return (0);
   }
   /*   for (i=1; i<k; i++)
   {
   	if((((centro-hull[i-1])^(hull[i]-hull[i-1])).z>0)!=(((P-hull[i-1])^(hull[i]-hull[i-1])).z>0)) return (0);
   }
	if((((centro-hull[k])^(hull[0]-hull[k])).z>0)!=(((P-hull[k])^(hull[0]-hull[k])).z>0)) return (0);*/
   return (1);		            //El puntos esta del mismo lado de los semiplanos que el ctentroide, esta en su interior.
}

float margen_est (punto3d P, punto3d *arreglo, int k)
{
	/*punto3d *hull = new punto3d[k+1];		//Creo un nuevo arreglo donde el ultimo punto es el primero
   memcpy((void*)hull, (void*)arreglo, sizeof(punto3d)*(k+1));
	hull[k] = arreglo[0];
   punto3d P = punto3d(punto.x, punto.x, hull[0].z);*/
   register int i;
	punto3d *hull = new punto3d[k+1];
   for (i=0; i<k; i++)
   {
		hull[i] = punto3d(arreglo[i].x,arreglo[i].y,P.z);
   }
   hull[k] = hull[0];
	if (punto_convexhull(P, hull, k)==1)	//Punto esta dentro o en el convex_hull, hay que calcular la distancia a las rectas que lo forman
   {
//   	register int i;
      float margen = 3E38;
   	for (i=0; i<k; i++)
      {
			recta3d r = recta3d(hull[i], (hull[i+1]-hull[i]));   //Defino una recta que pasa por los dos vertices
         if ((r.distancia(P))<margen) margen = r.distancia(P);  //Verifico si la nueva distancia minima es menor a las anteriores
      }
      delete hull;
      return margen;
   }
   float margen = 3E38;
//	register int i; 				//Calculo si el punto esta fuera del convex hull
   for (i=0; i<k; i++)
   {
      register float d_min = min(P.distancia(hull[i]), P.distancia(hull[i+1]));                       //Distancia minima es la menor a los vertices
      if ((((P.distancia(hull[i])-P.distancia(hull[i+1])) <= MACH_EPS) && ((((P-hull[i])%(hull[i+1]-hull[i]))-M_PI_2) < MACH_EPS)) ||   //GRAN "if" que verifica si la proyeccion del punto esta en el segmento
          (((P.distancia(hull[i])-P.distancia(hull[i+1])) > MACH_EPS) && ((((hull[i]-hull[i+1])%(P-hull[i+1]))-M_PI_2) < MACH_EPS)))
     	{
      	recta3d r = recta3d(hull[i], (hull[i+1]-hull[i]));
         d_min = r.distancia(P);  	//De ser cierto, la distancia minima es la proyeccion a la recta
      }
/*   	register float d1 = P.distancia(hull[i]);
		register float d2 = P.distancia(hull[i+1]);
      register float d_min = min(d1, d2);                       //Distancia minima es la menor a los vertices
      if (((d1 < d2) && ((P-hull[i])%(hull[i+1]-hull[i])) < M_PI_2) ||   //GRAN "if" que verifica si la proyeccion del punto esta en el segmento
          ((d2 < d1) && ((hull[i+1]-hull[i])%(P-hull[i])) < M_PI_2))
     	{
      	recta3d r = recta3d(hull[i], (hull[i+1]-hull[i]));
         d_min = r.distancia(r.proyeccion(P));  	//De ser cierto, la distancia minima es la proyeccion a la recta
      }*/
      //printf("\n P.distancia(hull[i]) es: %.3f P.distancia(hull[i+1]) es: %.3f \n", P.distancia(hull[i]), P.distancia(hull[i+1]));
      //printf("\n (P-hull[i])(hull[i+1]-hull[i]) es: %.3f y pi/2 es %.3f\n", (P-hull[i])%(hull[i+1]-hull[i]), M_PI_2);
      //printf("\n ((hull[i]-hull[i+1])(P-hull[i+1]) es: %.3f \n", ((hull[i]-hull[i+1])%(P-hull[i+1])));
      if (d_min<margen) margen = d_min;         //Verifico si la nueva distancia minima es menor a las anteriores
   }
   delete hull;
   return (-margen);
}

punto3d punto_est (punto3d P, punto3d *arreglo, int k)

{
   register int i;
	punto3d *hull = new punto3d[k+1];
	punto3d limite;
   for (i=0; i<k; i++)
   {
		hull[i] = punto3d(arreglo[i].x,arreglo[i].y,P.z);
   }
   hull[k] = hull[0];
	if (punto_convexhull(P, hull, k)==1)	//Punto esta dentro o en el convex_hull, hay que calcular la distancia a las rectas que lo forman
   {
	   float margen = 3E38;
   	for (i=0; i<k; i++)
      {
			recta3d r = recta3d(hull[i], (hull[i+1]-hull[i]));   //Defino una recta que pasa por los dos vertices
         if ((r.distancia(P))<margen) 				  	//Verifico si la nueva distancia minima es menor a las anteriores
			{
				margen = r.distancia(P);
         	limite = r.proyeccion(P);					//De ser asi, determino el nuevo punto limte como la proyeccion de P sobre r
         }
      }
      delete hull;
      return limite;
   }
   float margen = 3E38;
   for (i=0; i<k; i++)			//Caso caudno el punto esta fuera del convex hull
   {
      punto3d punto_min;
      register float d_min = min(P.distancia(hull[i]), P.distancia(hull[i+1]));                       //Distancia minima es la menor a los vertices
		if ((P.distancia(hull[i])-P.distancia(hull[i+1])) <= MACH_EPS) punto_min = hull[i];					//Punto borde es el vértice más cercano
		else punto_min = hull[i+1];
      if ((((P.distancia(hull[i])-P.distancia(hull[i+1])) <= MACH_EPS) && ((((P-hull[i])%(hull[i+1]-hull[i]))-M_PI_2) < MACH_EPS)) ||   //GRAN "if" que verifica si la proyeccion del punto esta en el segmento
          (((P.distancia(hull[i])-P.distancia(hull[i+1])) > MACH_EPS) && ((((hull[i]-hull[i+1])%(P-hull[i+1]))-M_PI_2) < MACH_EPS)))
     	{
      	recta3d r = recta3d(hull[i], (hull[i+1]-hull[i]));
         d_min = r.distancia(P);
         punto_min = r.proyeccion(P);  //De ser cierto, el punto borde es la proyeccion de P al segmento
      }
      if (d_min<margen)				//Verifico si la nueva distancia minima es menor a las anteriores
      {
         margen = d_min;    		// Nueva distancia mínima y borde
      	limite = punto_min;
      }
   }
   delete hull;
   return (limite);
}
