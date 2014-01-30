/* -*- Mode: C; indent-tabs-mode: t; c-basic-offset: 4; tab-width: 4 -*- */
/*
 * analisis.cpp
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

//#include "vector3d.cpp"
//#include "convexhull.hpp"
#include "analisis.hpp"

punto3d rot_punto_ea(vector3d eje, float angulo, punto3d p)
{
    register float A = 1-cos(angulo);  //Se definen los elementos de la matriz de rotación
    register float r11 = eje.x*eje.x*A+cos(angulo);
    register float r12 = eje.x*eje.y*A-eje.z*sin(angulo);
    register float r13 = eje.x*eje.z*A+eje.y*cos(angulo);
    register float r21 = eje.y*eje.x*A+eje.z*sin(angulo);
    register float r22 = eje.y*eje.y*A+cos(angulo);
    register float r23 = eje.y*eje.z*A-eje.x*sin(angulo);
    register float r31 = eje.z*eje.x*A-eje.y*sin(angulo);
    register float r32 = eje.z*eje.y*A+eje.x*sin(angulo);
    register float r33 = eje.z*eje.z*A+cos(angulo);
    register float x = (vector3d(r11, r12, r13))*vector3d(p);	//Producto punto para cada coordenada
    register float y = (vector3d(r21, r22, r23))*vector3d(p);
    register float z = (vector3d(r31, r32, r33))*vector3d(p);
    return(punto3d(x, y, z));
}

plano3d rot_plano_vector(recta3d r, float angulo, plano3d p)
{
    punto3d p_trasladado = rot_punto_ea(r.vector, angulo, p.punto);
    vector3d n_rotada = rot_punto_ea(r.vector, angulo, punto3d(p.normal));
    return(plano3d(p_trasladado,n_rotada));
}

punto3d rot_punto_recta(recta3d r, float angulo, punto3d p)
{
    return(punto3d(vector3d(rot_punto_ea(r.vector, angulo, p))+vector3d(r.punto)));
}

plano3d rot_plano_recta(recta3d r, float angulo, plano3d p)
{
    punto3d p_trasladado = rot_punto_recta(r, angulo, p.punto);
    vector3d n_rotada = rot_punto_ea(r.vector, angulo, punto3d(p.normal));
    return(plano3d(p_trasladado,n_rotada));
}

int puntoMasLejano (vector3d v, punto3d *P, int k)
{
    //para cada punto, hallo la proyeccion en la direccion de v, inicialmente agarro cero como la mejor solucion
    register int i, imax = 0;
    punto3d pmax = P[imax]; //mejor punto
    float d, dmax = vector3d(pmax)*v; //distancia maxima (mejor hasta ahora)

    for (i=1; i<k; i++)
    {
        if ((d=vector3d(P[i])*v)>dmax)
        {
            imax = i;
            dmax = d;
        }
    }
    return imax;
}

plano3d planoProyeccion (vector3d v, punto3d *P, int k)
{
    return plano3d(P[puntoMasLejano(v,P,k)],v);
}

plano3d planoProyectar (vector3d v, punto3d *P, punto3d *Q, int k)
{
    plano3d pi_ = planoProyeccion (v, P, k); //calculo el plano de proyeccion
    //luego proyecto cada punto sobre el plano
    for (register int i=0; i<k; i++)
        Q[i]=pi_.proyeccion(P[i]);
    return pi_;
}

void planoProyectarXY (vector3d v, punto3d *P, punto3d *Q, int k)
{
    planoProyectar (v, P, P, k); //calculo los puntos en el plano de proyeccion con dirección v
    vector3d eje = v^(vector3d(0,0,1));
    float angulo = acos((v*(vector3d(0,0,1)))/(v.norma()*(vector3d(0,0,1)).norma()));
    for (register int i=0; i<k; i++)
    {
        Q[i] = rot_punto_ea(eje, angulo, P[i]);
    }
}

int punto_pivote(punto3d *P, int k)
{
    //primero determinamos el punto con la menor coordenada X, si hay empate, se agarra el de menor coordenada Y
    int i, i_pivote = 0;
    punto3d pivote = P[0];
    for (i=1; i<k; i++)
    {
        if ((P[i].y < pivote.y)||((P[i].y == pivote.y)&&(P[i].x < pivote.x))) //es un punto con menor X o tiene el mismo X pero menor Y
        {
            i_pivote = i;
            pivote = P[i]; //reemplazo y marco el nuevo punto
        }
    }
    return(i_pivote);
}

void sort_by_angle(punto3d *P, int k, int pivote, int *Q)
{
    int i;
    float *angulo = new float[k];	//creo arreglo de K elementos para las cotnagentes con respecto al punto inicial
    for (i=0; i<k; i++)					//Primero se determina el angulo de cada punto respecto al pivote
    {
        if (i == pivote) angulo[i] = -1;			 	//Caso, punto pivote forzamos que sea el primer elemento
        else
        {
            angulo[i] = atan2((P[i].y-P[pivote].y), (P[i].x-P[pivote].x));	//Formula de angulo por arco tangente
            if (angulo[i] < 0) angulo[i] = angulo[i] + M_PI;
        }
    }
    BubbleSort (angulo, Q, k);		//Se ordenan los índices respecto a los angulos
    delete angulo;
}

void BubbleSort (float *d, int *indice, int k) //Bubble Sort de un arreglo de float de tamaño k, devuelve el vector ordenado y las posiciones de los indices
{
    register int i,j,p;
    bool swap;
    for (i=0; i<k; i++) indice[i]=i;
    for (i=0; i<k; i++)
    {
        swap = false;
        for (j=0; j<k-1; j++)
            if (d[indice[j]]>d[indice[j+1]]) //esta al reves?
            {
                p=indice[j+1];
                indice[j+1]=indice[j];
                indice[j]=p;
                swap = true;
            }
        if (!swap) break; //si no hizo un cambio en este ciclo, significa que ya esta ordenada
    }
}

punto3d centroide(punto3d *P, int k)
{
    int indice;
    vector3d p = vector3d(0,0,0);
    for (indice=0; indice<k; indice++) p = p + P[indice];
    p.x = p.x/k;
    p.y = p.y/k;
    p.z = p.z/k;
    return punto3d(p);
}

void printfvector3d(char *str, vector3d v)
{
    sprintf (str, "X: %.3f\tY: %.3f\tZ: %.3f", v.x, v.y, v.z);
}

void printfrecta3d(char *str, recta3d r)
{
    sprintf (str, "Punto: X: %.3f\tY: %.3f\tZ: %.3f\t Vector: X: %.3f\tY: %.3f\tZ: %.3f\t", r.punto.x, r.punto.y, r.punto.z, r.vector.x, r.vector.y, r.vector.z);
}

void printfplano3d(char *str, plano3d p)
{
    sprintf (str, "Punto: X: %.3f\tY: %.3f\tZ: %.3f\t Normal: X: %.3f\tY: %.3f\tZ: %.3f\t", p.punto.x, p.punto.y, p.punto.z, p.normal.x, p.normal.y, p.normal.z);
}
