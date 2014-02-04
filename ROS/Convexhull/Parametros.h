/*
 *  Parametros.h
 *
 *
 *  Created by Diego Leal on 8/1/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

/*
	Revisar y comparar las estructuras de datos, asi como los nombres de los parametros (largos, centros, masas) para que sigan un estandar de nomenclatura, y evitar colisiones con nombres de otros elementos.
*/

#include <stdio.h>

typedef struct {
		float X;
		float Y;
		float Z;
		}PuntoCartesiano;

typedef struct {
		float q1;
		float q2;
		float q3;
		}Articulaciones;

typedef struct {
		float beta;     //Factor de apoyo
		float lambda;   //Longitud del paso
		float h;        //Altura del cuerpo
		float ab;       //Ancho (apertura de la piernas)
		float alto;     //Si se va a cambiar la configuracion incial de robot de q2=q3=15, se debe cambiar este valor
		}Const_caminado;


float L1=3.05, L2=7.4, L3=13.66, d1=17.8, d2=21.7; //Medidas en Centimetros y gramos-> pasarlas a MKS
float masa1=0.160, masa2=0.2310, masa3=0.14979, masa_base=0.20986;
float cm_L1=1.122, cm_L2=3.69, cm_L3=4.939;
