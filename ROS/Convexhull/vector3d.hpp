/* -*- Mode: C; indent-tabs-mode: t; c-basic-offset: 4; tab-width: 4 -*- */
/*
 * vector3d
 * Copyright (C) Jose Cappelletto, Dimitar Ralev 2009
 *
 * vector3d is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * vector3d is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/******************************************************************************
	COMENTARIOS:
******************************************************************************/

#ifndef _VECTOR3D_H_
#define _VECTOR3D_H_

//#include <stdio.h>
//#include <math.h>

class punto3d;      //definiciones volatiles de cada clase para poder hacer operaciones
class vector3d;     //entre cada uno de ellos. Luego se define el prototipo y cuerpo
class recta3d;      //de las clases
class plano3d;      //plano
class segmento3d;   //segmento de recta (R3)

class punto3d
{
public:
    punto3d();								//constructor nulo de punto3d
    punto3d(float, float, float); 	//constructor de punto3d a partir de las coordenadas
    punto3d(vector3d);					//constructor (CAST) a partir de vector3d
    vector3d operator -(punto3d);    //operador - resta entre dos puntos, devuelve un vector3d
    bool		operator ==(punto3d);	//operador == comparacion entre dos puntos
    float x, y, z;							//coordenadas X,Y,Z del punto3d
    float distancia(punto3d);			//calcula la distancia entre dos puntos
};

class vector3d: public punto3d
{
public:

    vector3d();								//constructor nulo de vector3d
    vector3d(punto3d);					//constructor de vector a partir de punto3d
    vector3d(punto3d, punto3d);		//constructor de vector a partir de 2 punto3d
    vector3d(float,float,float);		//constructor de vector a partir de coordenadas independientes

    vector3d operator -();				//operador -, vector con sentido inverso
    bool		operator ==(vector3d);	//operador == comparacion entre dos vectores
    vector3d operator +(vector3d);	//operador + suma de 2 vectores
    vector3d operator +=(vector3d);	//operador + suma de 2 vectores
    vector3d operator -(vector3d);	//operador - resta de 2 vectores

    vector3d operator -=(vector3d);	//operador - resta de 2 vectores
    vector3d operator *(float);		//operador * multiplicacion de vector y escalar
    vector3d operator *=(float);		//operador * multiplicacion de vector y escalar
    float 	operator *(vector3d);	//operador * producto punto entre dos vectores (producto interno)
    vector3d operator ^(vector3d);	//operador ^ producto matricial entre dos vectores (producto cruz)
    float 	operator %(vector3d);   //operador % devuelve el ángulo mínimo entre dos vectores

    float norma(void);				 	//devuelve la norma L2 (modulo) del vector
    void normalizar(void);				//normaliza el vector (norma=1)
    vector3d escalar(float);			//escala por un numero real cada uno de los elementos del vector
//   float 	distancia(vector3d);	 	//calcula la distancia entre dos vectores
    vector3d proy_ort(vector3d);   	//calcula la proyección ortogonal de un vector sobre el actual.
    int 		ortogonal(vector3d);	 	//determina si dos vectores son ortogonales
    int 		paralelo(vector3d);     //Determina si dos vectores son paralelas

protected:

private:

};

class recta3d			// forma punto-direccion de la recta
{
public:
    recta3d();							//constructor nulo de recta3d
    recta3d(punto3d,vector3d);		//constructor a partir de punto-vector
    recta3d(punto3d,punto3d);		//constructor a partir de dos puntos

    punto3d punto;          		//punto pivote contenido en la recta
    vector3d vector;					//vector direccion de la recta

    bool	operator ==(recta3d);	//operador == comparacion entre dos rectas
    float distancia(punto3d);		//Distancia minima desde un punto a esta recta
    float	distancia(recta3d);		//Distancia mínima desde una recta a esta recta
    int 	ortogonal(recta3d);		//Determina si dos rectas son ortogonales
    int 	paralelo(recta3d);      //Determina si dos rectas son paralelas
    int	alabeada(recta3d);		//Determina si dos rectas son alabeadas
    punto3d proyeccion(punto3d);  //Devuelve el punto de la recta más cercano al punto dado
    recta3d recta_minima (recta3d); //Devuelve la recta de minima distancia entre 'r' y 'this'

protected:

private:

};

//class segmento3d								// forma punto-normal al plano
//{
//public:
//    segmento3d();								//constructor nulo de plano
//    segmento(punto3d, punto3d); //constructor a partir de 2 puntos
//
//    punto3d ini, fin;   //por convencion el segmento arranca en ini y termina en fin
//
//    float		longitud();		    //longitud del segmento de recta
//    vector3d    direccion();        //vector que va de 'ini' hasat 'fin'
//    punto3d     proyeccion (punto3d);   //devuelve el punto del segmento mas cercano a un punto dado
//
//protected:
//
//private:
//
//};

class plano3d								// forma punto-normal al plano
{
public:
    plano3d();								//constructor nulo de plano
    plano3d(punto3d, punto3d, punto3d); //constructor a partir de 3 puntos (no colineales)
    plano3d(punto3d, vector3d);		//constructor a partir del vector y la normal
    plano3d(punto3d, recta3d);			//constructor a partir de un punto y una recta (que no pase por el punto)
    plano3d(recta3d, recta3d);			//constructor a partir de un punto y una recta (que no pase por el punto)

    punto3d punto;
    vector3d normal;

    float		distancia(punto3d);		//Distancia minima punto al plano
    float		distancia(recta3d); 		//Distancia minima recta al plano
    float		distancia(plano3d); 		//Distancia entre dos planos paralelos
    punto3d  interseccion(recta3d);	//Devuelve el punto de interseccion entre una recta y este plano
    recta3d  interseccion(plano3d);	//Devuelve la recta de interseccion entre un plano y este plano
    punto3d 	proyeccion(punto3d);		//Devuelve el punto del plano más cercano al punto dado

protected:

private:

};

#endif // _VECTOR3D_H_
