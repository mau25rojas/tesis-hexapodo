/* -*- Mode: C; indent-tabs-mode: t; c-basic-offset: 4; tab-width: 4 -*- */
/*
 * vector3d
 * Copyright (C) Jose Cappelletto 2009 <>
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

#include "vector3d.hpp"
#include <math.h>
#include <cstddef>
#define MACH_EPS 1E-19

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//FUNCIONES PARA EL PUNTO
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
punto3d::punto3d() // Constructor sin argumentos de la clase punto3d.
{
    x = y = z = 0; //Asigno el vector nulo cada uno de los componentes
}

punto3d::punto3d(vector3d v)	//constructor (CAST) a partir de vector3d
{
    x = v.x;
    y=v.y;
    z=v.z; //asigno cada uno de los elementos del punto los mismo elementos del vector
}

punto3d::punto3d(float nx, float ny, float nz) // Constructor con argumentos de la clase punto3d.
{
    x = nx;
    y = ny;
    z = nz; //asigno nuevos valores individualmente (3x float)
}

vector3d punto3d::operator -(punto3d p)    //operador resta entre dos puntos, devuelve un vector
{
    return (vector3d(p,*this)); //devuelvo un vector que va desde el punto 'p' hasta 'this'
}

bool punto3d::operator ==(punto3d p)
{
    if (((fabs(x-p.x)<=MACH_EPS) && (fabs(y-p.y)<=MACH_EPS)) && (fabs(z-p.z)<=MACH_EPS)) return true; //devuelve 'true' si cada uno de los elementos son iguales
    return false;
}

float punto3d::distancia(punto3d p) //calcula la distancia entre dos puntos
{
    return vector3d(*this,p).norma(); //creo un vector entre los dos puntos y hallo su norma, lo cual nos da la distancia
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//FUNCIONES PARA EL VECTOR
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

vector3d::vector3d()	//constructor de vector nulo
{
    x = y = z = 0; //asignacion nula a cada uno de los elementos del vector
}

vector3d::vector3d(punto3d p)	//constructor de vector a partir de punto3d
{
    x= p.x;
    y= p.y;
    z= p.z; //asigno cada uno de los elementos de punto3d al vector3d
}

vector3d::vector3d(punto3d a, punto3d b)  //constructor de vector a partir de 2 puntos
{
    //el vector resultante va desde 'a' hasta 'b'
    x = b.x-a.x;
    y = b.y-a.y;
    z = b.z-a.z; //el vector lo obtengo por resta de cada componente
}

vector3d::vector3d(float nx,float ny,float nz)	//constructor de vector a partir de las 3 coordenadas
{
    x = nx;
    y = ny;
    z= nz; //asigno nuevos valores de cada una de las componentes del vector
}

vector3d vector3d::operator -() //operador -, vector con sentido inverso
{
    vector3d v; //creo un vector temporal
    v.x = -x;
    v.y = -y;
    v.z = -z; //invierto cada elemento del vector
    return v; //devuelvo la copia del vector
}

bool vector3d::operator ==(vector3d v)	//operador == comparacion entre dos vectores
{
    if ((fabs(x-v.x)<=MACH_EPS) && (fabs(y-v.y)<=MACH_EPS) && (fabs(z-v.z)<=MACH_EPS)) return true;
    return false;
}

float vector3d::norma(void)
{
    //calculo la norma L2 (modulo) del vector, como la raiz de la suma de los cuadrados de cada dimension
    return sqrt(x*x+y*y+z*z);
}

void vector3d::normalizar(void)
{
    float n = this->norma();		  //calculo la norma 'n' del vector
    if (n!=0) this->escalar(1/n); //si es distinta de cero procedo, escalando por '1/n'
    //si la norma fuese cero, no habria que hacer nada (vector NULO)
}

vector3d vector3d::escalar(float f)
{
    this->x*=f;
    this->y*=f;
    this->z*=f;
    return *this; //escala cada uno de los elementos del vector por un factor real 'f'
}

vector3d vector3d::operator +(vector3d v)
{
    vector3d u;
    u.x = x + v.x;
    u.y = y + v.y;
    u.z = z + v.z; //suma por separado cada uno de los componentes de los vectores (this + v)
    return u;		//devuelve una copia con el resultado de la suma de (this + v)
}

vector3d vector3d::operator +=(vector3d v)
{
    x+=v.x;
    y+=v.y;
    z+=v.z;  //suma por separado cada uno de los componentes de los vectores (this + v)
    return *this;  //devuelve this ya modificado con la suma de (this + v)
}

vector3d vector3d::operator -(vector3d v)
{
    vector3d u;
    u.x = x - v.x;
    u.y = y - v.y;
    u.z = z - v.z;	//resto por separado cada uno de los componentes de los vectores (this + v)
    return u;		//devuelve una copia con el resultado de la resta de (this + v)
}

vector3d vector3d::operator -=(vector3d v)
{
    x-=v.x;
    y-=v.y;
    z-=v.z;  //resta por separado cada uno de los componentes de los vectores (this - v)
    return *this;  //devuelve this ya modificado con la resta de (this - v)
}

vector3d vector3d::operator *(float f)
{
    vector3d u;
    u.x = x * f;
    u.y = y * f;
    u.z = z * f;//escala por separado cada uno de los componentes de los vectores (u*f)
    return u;	//devuelve la copia 'u'
}

vector3d vector3d::operator *=(float f)
{
    x*=f;
    y*=f;
    z*=f;
    return *this; //escala por separado cada uno de los componentes de los vectores y devuelve a this modificado
}

float vector3d::operator *(vector3d v)
{
    //calcula el producto punto (proyeccion) entre dos vectores.
    return x*v.x + y*v.y + z*v.z;	 //devuelve la suma de los productos de cada componente
}

vector3d vector3d::operator ^(vector3d v)
{
    vector3d r;
    r.x = y*v.z - z*v.y; //calculo cada uno de los componentes resultantes del producto matricial entre 'this' y 'v'
    r.y = z*v.x - x*v.z; //tambien conocido como producto cruz
    r.z = x*v.y - y*v.x;
    return r; //devuelve una copia, con el vector resultante del producto cruz
}

float vector3d::operator %(vector3d v) //calcula el angulo minimo entre 2 vectores
{
    if ((this->norma())*(v.norma()) == 0) return NULL; //chequeo que ninguno sea nulo
    return acos(*this*v/((this->norma())*(v.norma()))); //arcocoseno de la proyeccion sobre el producto de las normas
}

vector3d vector3d::proy_ort(vector3d v) //proyeccion ortogonal de v sobre this
{
    vector3d a = *this; //a = copia de this
    register float na = a.norma(); //na = norma de 'a'
    if (na == 0) return (a); //si es nulo me salgo
    return (a*(a*v)*(1/(na*na))); //escala el vector 'a' por el producto interno 'a.v' y lo divide entre la norma de 'a'
}

int vector3d::ortogonal(vector3d v)
{
    if ((*this*v) == 0) return (1); //devuelve 1 si el producto punto es nulo (son ortogonales)
    return (0);
}

int vector3d::paralelo(vector3d v)    //Hay que asegurarse que no esté usando vectores nulos
{
    if ((*this^v).norma() == 0) return (1); //si la norma del producto cruz es nulo, son paralelos
    return (0);
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//FUNCIONES PARA LA RECTA
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

recta3d::recta3d()	//constructor nulo de recta3d
{
    vector = vector3d(0,0,0); //creo vector nulo (0,0,0)
    punto = punto3d(0,0,0);
}

recta3d::recta3d(punto3d p,vector3d v)	//constructor a partir de punto-vector
{
    punto = p;
    vector = v; //asignacion directa del vector y del punto
}

recta3d::recta3d(punto3d p1,punto3d p2) //constructos a partir de dos puntos
{
    punto = p1; //asignacion del primer punto como P de la recta
    vector = p2 - p1; //el vector es la resta del punto final (p2) - punto inicial (p1)
}

bool recta3d::operator ==(recta3d r)	//operador == comparacion entre dos rectas
{
    return ((punto==r.punto)&&(vector==r.vector)); //devuelve el resultado de la comparacion de cada elemento de la recta
}

float recta3d::distancia(punto3d p) //devuelve la distancia desde un punto p a la recta
{
    //calcula la norma del vector de minima distancia.
    //el vector de minima distancia se halla proyectando el vector desde el punto 'p' hasta
    //cualquier punto de la recta 'q', sobre el vector de la recta, y moviendose desde 'q' en la direccion
    //del vector de la recta, un valor igual  a esa proyeccion
    return ((p - punto) - vector.proy_ort(p - punto)).norma();
}


float recta3d::distancia(recta3d r)
{
    vector3d producto = vector^r.vector;
    if (producto.norma() == 0) return r.distancia(punto); //si son nulos, son paralelos, la distancia es la mínima del punto de una recta a la otra
    return ((fabs((r.punto-punto)*producto))/(producto.norma()));
}

int recta3d::ortogonal(recta3d r)
{
    return (vector.ortogonal(r.vector)); //si los vectores son ortogonales, las rectas tambien son ortogonales
}

int recta3d::paralelo(recta3d r)    //Hay que asegurarse que no esté usando vectores nulos
{
    if ((r.vector^vector).norma() == 0) return (1);
    return (0);
    //PROBLEMA: uno o ambos vectores de las rectas sean nulos
}

int recta3d::alabeada(recta3d r) // determina si dos rectas son alabeadas (no se tocan)
{
    //calculo la proyeccion de un vector que va entre los dos puntos bases de las dos rectas, y el
    //vector producto cruz entre los dos vectores directores de las rectas.
    //cuando se tocan, esa proyeccion debe dar cero
    if ((r.punto-punto)*(vector^r.vector) != 0) return(1);
    return (0);
}

punto3d recta3d::proyeccion(punto3d p)	//Devuelve el punto de la recta más cercano al punto dado
{
    //Creo una copia (normalizada) del vector director de la recta
    vector3d v = vector;
    v.normalizar();
    //Proyecto el vector que va del punto director hasta el punto p, con el vector director
    float f = (p-punto)*v;
    //El punto proyectado esta a esa distancia 'f', a partir del punto director, en la direccion del vector director
    return punto3d(vector3d(punto) + v*f);
}

recta3d recta3d::recta_minima(recta3d r)
{
    //la recta de minima distancia va desde los puntos que se obtienen de la interseccion de alguna
    //de las rectas y del plano perpendicular a la otra recta
    vector3d v = vector^r.vector;
    plano3d p(r.punto, r.vector^v);
    return (recta3d(p.interseccion(*this),v));
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//FUNCIONES PARA EL SEGMENTO
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
segmento3d::segmento3d(punto3d a, punto3d b) // Constructor con argumentos de la clase segmento3d.
{
    ini = a;
    fin = b;
}

float segmento3d::longitud()    //devuelve la longitud de un segmento de recta3d
{
    vector3d v(fin-ini);    //creo un vector que va de 'ini' hasat 'fin'
    return v.norma();       //devuelvo su norma
}

vector3d segmento3d::direccion()    //devuelve un vector que va de 'ini' hasta 'fin'
{
    vector3d v(fin-ini);    //creo un vector que va de 'ini' hasat 'fin'
    return v;       //devuelvo su norma
}

//funcion que halla el punto3d mas cercano de un segmento3d desde un punto3d 'p'
punto3d segmento3d::proyeccion(punto3d p)
{
    //Hay que determinar a cual de las 3 regiones pertenece. Si pertenece a la region del medio, el punto3d
    //resultante es el obtenido proyectando el punto 'p' a la recta3d que pasa por el segmento3d

    //Primero hallamos el punto proyeccion del punto3d 'p' sobre la recta3d que pasa por el segmento3d

    //Construyo una recta a partir de los puntos3d que definen el segmento3d
    recta3d r(ini,fin);

    //Proyecto el punto 'p' sobre esa recta 'r'
    punto3d proy = r.proyeccion(p);

    //Construyo los dos vectores desde 'p' hasta los extremos
    vector3d vi(p,ini);
    vector3d vf(p,fin);

    //Si 'proy' pertenece a la region del medio, ambos vectores 'vi' y 'vf' deben de apuntar a la misma direccion
    //Calculo la proyeccion (producto punto) de los dos vectores 'vi' y 'vf'
    float f =vi*vf;

    //Si apuntan a direcciones opuestas, es que esta en la region central, y el producto punto deberia dar negativo
    //En ese caso, el punto 'proy' proyectado si es el mas cercano al segmento de recta
    if (f<0) return (proy);

    //Sino, entonces veo cual de los dos extremos esta mas cerca. Para ello comparo la distancia (norma del vector)
    if (vi.norma() < vf.norma()) return (ini);
    else return (fin);
}

//funcion que determina si dos segmentos (this / a) se intersectan
//en caso de existir la interseccion, calcula el punto en cuestion y lo almacena en 'p'
bool segmento3d::interseccion(segmento3d a, punto3d* p)
{
    //para que exista interseccion, cada segmento de recta debe de separar los puntos del otro, en dos hemiplanos distintos (y de forma reciproca)
    //primero determinamos si los puntos que forman al segmento 'a' estan a ambos lados de la recta que define (this)
    //para eso usamos la funcion auxiliar que determina en que hemiplano esta un punto 'p'
    int reg1,reg2;
    //determino en que region esta cada punto del segmento de recta 'a' con respecto a la recta 'this'
    reg1 = this->hemiplano(a.ini);
    reg2 = this->hemiplano(a.fin);
    //si ambos resultados son del mismo signo, entonces los dos puntos estan en la misma region (no hay interseccion)
    if ((reg1*reg2) > 0) return false;

    //ahora prueba el reciproco
    reg1 = a.hemiplano(ini);
    reg2 = a.hemiplano(fin);
    //si ambos resultados son del mismo signo, entonces los dos puntos estan en la misma region (no hay interseccion)
    if ((reg1*reg2) > 0) return false;

    //si llega a este punto entonces los segmentos de recta se intersectan, y el punto resultante es la interseccion
    //convencional de las rectas directoras de cada segmento

    punto3d p1 = ini;
    punto3d p2 = a.ini;
    vector3d v1(ini,fin);
    vector3d v2(a.ini,a.fin);

    register float k1, k2, temp1, temp2;

    if (v1.x==0) v1.x = 0.000001;
    temp1 = p1.y + (p2.x-p1.x)*(v1.y/v1.x) - p2.y;
    temp2 = v2.y - v1.y*(v2.x/v1.x);

    if (temp2==0) temp2 = 0.00001; //para preveer divisiones entre cero
    k2 = temp1/temp2;

    p->x = p2.x + k2*v2.x;
    p->y = p2.y + k2*v2.y;
    p->z = p2.z + k2*v2.z; //OJO: Z deberia ser cero siempre ya que esto parte de la hipotesis del plano XY

    return true;
}

//IMPORTANTE: esta funcion asume que los puntos existen en el plano XY (se asume Z=0)
//funcion que calcula en que parte (relativa) del plano se encuentra un punto 'p' con respecto a una recta
//si es <0 pertenece a la region I, si es >0 pertenece a la region II, si =0 esta sobre la recta
int segmento3d::hemiplano(punto3d p)
{
    //para ello, calculamos la proyeccion de un vector 'u' que va desde el punto 'ini' al punto de entrada 'p', sobre
    //un vector 'v' perpendicular a la recta (segmento)

    //creo el vector 'u' que va de 'ini' a 'p'
    vector3d u(ini,p);

    //para crear el vector 'v', rotamos el vector 'director' del segmento de recta: el vector que va de 'ini' a 'fin'
    vector3d v(ini,fin);
    float temp;
    temp = v.x;
    v.x = -v.y;
    v.y = temp;
    //estas operaciones arriba es el equivalente a rotar +90 grados (antihorario) al vector director del segmento de recta.
    //dicho vector queda apuntando hacia la region I
    temp = u*v; //calculamos la proyeccion del vector 'u' sobre este vector normal al segmento de la recta.
    //el signo del resultado indica en que region está
    if (temp>0) return (1);
    else if (temp<0) return (-1);
    else return (0);
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//FUNCIONES PARA EL PLANO
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

plano3d::plano3d()	//constructor nulo de plano
{
    punto = vector3d(0,0,0);
    normal = punto;	//creo los vectores nulos para el punto y la normal
}

plano3d::plano3d(punto3d a, punto3d b, punto3d c) //constructor a partir de 3 puntos (no colineales)
{
    punto = vector3d(a); //agarro cualquier punto
    vector3d u(a,b), v(b,c);
    normal = u ^ v; //la normal es el producto cruz de los dos vectores a-b y b-c
}

plano3d::plano3d(punto3d p, vector3d v) //constructor a partir del vector y la normal (directo)
{
    punto = vector3d(p);
    normal = v; //lo hago por asignacion directa
}

plano3d::plano3d(punto3d p, recta3d r)	 //constructor a partir de un punto y una recta (que no pase por el punto)
{
    punto = vector3d(p); //asigno el punto
    normal = vector3d(vector3d(p)-r.punto) ^ r.vector;
    //la normal es el producto cruz entre un vector desde la recta al punto y el vector de la recta
}

plano3d::plano3d(recta3d ra, recta3d rb)	//constructor a partir de un punto y una recta (que no pase por el punto)
{
    *this = plano3d(punto3d(ra.punto), rb); //reciclando codigo
}

float plano3d::distancia(punto3d p) //devuelve la distancia desde un punto hasta la superficie del plano
{
    register float na = normal.norma();
    if (na == 0) return (NULL);
    return ((p-punto)*normal)/na;
}

float plano3d::distancia(recta3d r)	//devuelve la distancia entre una recta y un plano
{
    if (normal.ortogonal(r.vector)==0) return 0; //verifico primero si son paralelos la recta y el plano, si no distancia es 0
    return(distancia(r.punto));	 // La distancia es la del punto que define la recta al plano
}

float plano3d::distancia(plano3d p) 		//Distancia entre dos planos paralelos
{
    if ((normal^p.normal).norma() != 0) return 0; //verifico si de verdad son paralelos, si no distancia es 0
    return (distancia(p.punto));   // La distancia es la del punto que define la recta al plano
}

punto3d plano3d::interseccion(recta3d r)	//devuelve el punto de interseccion entre una recta y un plano
{
    if (normal.ortogonal(r.vector)!=0) return r.punto; //me aseguro que la recta no sea paralela al plano
    return punto3d(vector3d(r.punto)+r.vector*(((punto-r.punto)*normal)/(normal*r.vector)));
}

recta3d  plano3d::interseccion(plano3d p)	//devuelve la recta de interseccion entre un plano y este plano
{
    recta3d r;
    r.vector = normal^(normal^(p.normal));
    r.punto = punto;
    return (recta3d(interseccion(r),normal^p.normal));
}

punto3d plano3d::proyeccion(punto3d p)	//Devuelve el punto del plano más cercano al punto dado
{
    return(interseccion(recta3d(p,normal)));	//Sacando la intersección de una recta normal al plano que pasa por el punto dado
}


