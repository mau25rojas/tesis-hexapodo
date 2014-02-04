/*
 *  Cinematica_directa.cpp
 *
 *
 *  Created by Diego Leal on 8/1/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 *  Modificado y adaptado por NiKiTa
 */

#include <math.h>

//#include "vector3d.cpp"
//#include "analisis.cpp"
//#include "convexhull.cpp"
#include "cinematicas.hpp"

#define NUMpuntos 51 ///NUMERO DE PUNTOS EN LA TRAYECTORIA. RECOMENDACION: PONER UN BUEN NUM +1, P.E 51

/************FUNCION a partir del tiempo t dice el punto x,y,z desde el CM para determinada pata***********/

/************Trayectoria Cuadrada***********/
void Tray_Rect(Const_caminado consta, float t, int pata, punto3d* trayect){

    trayect->y=consta.ab;

	if(t>=0 && t<=consta.beta){
        trayect->z=0;
        trayect->x= consta.lambda/2 -consta.lambda*t/consta.beta;
    }

	else{
		float t1;
		t1= consta.beta + (1-consta.beta)/(2 + consta.lambda/consta.h);
		float t2;
		//t2= 1 - (1-consta.beta)/(2 +consta.lambda/consta.h);    Cambio por la fórmula NiK.
		t2= 1 - consta.lambda/consta.h*(1-consta.beta)/(2 +consta.lambda/consta.h);
        if(t>consta.beta && t<=t1){
            trayect->x=-consta.lambda/2;
            trayect->z= consta.h*(t-consta.beta)/(t1-consta.beta);
        }
        if(t>t1 && t<=t2){
            trayect->z=consta.h;
            trayect->x= -consta.lambda/2 + (t-t1)*consta.lambda/(t2-t1);
        }
        if(t>t2 && t<=1){
            trayect->x=consta.lambda/2;
            trayect->z= consta.h-consta.h*(t-t2)/(1-t2);
    }
    }

    switch (pata){
        case 1:
            trayect->x = trayect->x - d2/2;
            trayect->y = -trayect->y - d1/2; //Corregi signo de trayec->Y. Seria mejor cambiar en consta.ab?
        break;
        case 2:
            trayect->x = trayect->x + d2/2;
            trayect->y = -trayect->y - d1/2; //Corregi signo de trayec->Y. Seria mejor cambiar en consta.ab?
        break;
        case 3:
            trayect->x = trayect->x - d2/2;
            trayect->y = trayect->y + d1/2;
        break;
        case 4:
            trayect->x = trayect->x + d2/2;
            trayect->y = trayect->y + d1/2;
        break;
    }
 // if(pata==1 || pata==2)
 // trayect->z=-trayect->z-15.5753;
//  else
    trayect->z=trayect->z-consta.alto; //Si se va a cambiar la configuracion incial de robot de q2=q3=15, se debe cambiar este valor
}

/************Trayectoria Triangular***********/
 void Tray_Trian(Const_caminado consta, float t, int pata, punto3d* trayect){

    trayect->y=consta.ab;

    if(t>=0 && t<=consta.beta){
        trayect->z=0;
        trayect->x= consta.lambda/2 -consta.lambda*t/consta.beta;
    }
    else{
        float t1;
        t1= consta.beta + (1 - consta.beta)/2;
        if(t>consta.beta && t<=t1){
            trayect->x= -consta.lambda/2 + consta.lambda/2*(t-consta.beta)/(t1-consta.beta);
            trayect->z= consta.h*(t-consta.beta)/(t1-consta.beta);
            //printf("altura h-> Z = %f, posicion X = %f\n", trayect->z, trayect->x);
        }
        if(t>t1 && t<=1){
            trayect->x= consta.lambda/2*(t-t1)/(1-t1);
            trayect->z= consta.h - consta.h*(t-t1)/(1-t1);
            //printf("altura h-> Z = %f, posicion X = %f\n", trayect->z, trayect->x);
        }
    }

    switch (pata){
    case 1:
        trayect->x = trayect->x - d2/2;
        trayect->y = -trayect->y - d1/2; //Corregi signo de trayec->Y. Seria mejor cambiar en consta.ab?
    break;
    case 2:
        trayect->x = trayect->x + d2/2;
        trayect->y = -trayect->y - d1/2; //Corregi signo de trayec->Y. Seria mejor cambiar en consta.ab?
    break;
    case 3:
        trayect->x = trayect->x - d2/2;
        trayect->y = trayect->y + d1/2;
    break;
    case 4:
        trayect->x = trayect->x + d2/2;
        trayect->y = trayect->y + d1/2;
    break;
    }
// if(pata==1 || pata==2)
// trayect->z=-trayect->z-15.5753;
//  else
    trayect->z=trayect->z-consta.alto; //Si se va a cambiar la configuracion incial de robot de q2=q3=15, se debe cambiar este valor
}

/************Trayectoria Elíptica***********/
 void Tray_Elip(Const_caminado consta, float t, int pata, punto3d* trayect){

    trayect->y=consta.ab;

    if(t>=0 && t<=consta.beta){
        trayect->z=0;
        trayect->x= consta.lambda/2 -consta.lambda*t/consta.beta;
    }
    else{
        float t1;
        t1= consta.beta + (1 - consta.beta)/2;
        if(t>consta.beta && t<=t1){
            trayect->x= -consta.lambda/2*cos(M_PI/2*(t-consta.beta)/(t1-consta.beta));
            trayect->z= consta.h*sin(M_PI/2*(t-consta.beta)/(t1-consta.beta));
            //printf("altura h-> Z = %f, posicion X = %f\n", trayect->z, trayect->x);
        }
        if(t>t1 && t<=1){
            trayect->x= consta.lambda/2*sin(M_PI/2*(t-t1)/(1-t1));
            trayect->z= consta.h*cos(M_PI/2*(t-t1)/(1-t1));
            //printf("altura h-> Z = %f, posicion X = %f\n", trayect->z, trayect->x);
        }
    }

    switch (pata){
    case 1:
        trayect->x = trayect->x - d2/2;
        trayect->y = -trayect->y - d1/2; //Corregi signo de trayec->Y. Seria mejor cambiar en consta.ab?
    break;
    case 2:
        trayect->x = trayect->x + d2/2;
        trayect->y = -trayect->y - d1/2; //Corregi signo de trayec->Y. Seria mejor cambiar en consta.ab?
    break;
    case 3:
        trayect->x = trayect->x - d2/2;
        trayect->y = trayect->y + d1/2;
    break;
    case 4:
        trayect->x = trayect->x + d2/2;
        trayect->y = trayect->y + d1/2;
    break;
    }
// if(pata==1 || pata==2)
// trayect->z=-trayect->z-15.5753;
//  else
    trayect->z=-trayect->z-consta.alto; //Si se va a cambiar la configuracion incial de robot de q2=q3=15, se debe cambiar este valor
}

/**************FUNCION QUE CALCULA LA CINEMATICA DIRECTA***************/
void Cin_directa(int P, Articulaciones angulo, punto3d* punto){
//printf("%f\n",ang.q2);
 //float X,Y,Z;
 float x,y;
 Articulaciones ang;
 /*Correcciones para el Cuadrupedo*/
 ang.q2 = angulo.q2 + M_PI/2;
 ang.q3 = angulo.q3 - 0.8416;
 //Correcciones para estar de acuerdo con el modelo de Webots
 ang.q1 = angulo.q1 - M_PI/2;
 //printf("ojo %f %f\n",M_PI,-ang.q2);
 ang.q2 = M_PI/2 - ang.q2;
 //printf("%f\n",ang.q1);

 //printf("%f\n",ang.q2);

 ang.q3 = (-ang.q3 + 2.3 + M_PI/2)-2*M_PI;
 //printf("%f %f %f\n",ang.q1*180/M_PI,ang.q2*180/M_PI,ang.q3*180/M_PI);
 //printf("%f\n",ang.q3);
 ////////////
 punto->x = L1*cos(ang.q1) + L2*cos(ang.q1)*cos(ang.q2) + L3*cos(ang.q1)*cos(ang.q2)*cos(ang.q3) - L3*cos(ang.q1)*sin(ang.q2)*sin(ang.q3);
 punto->y = L1*sin(ang.q1) + L2*sin(ang.q1)*cos(ang.q2) + L3*sin(ang.q1)*cos(ang.q2)*cos(ang.q3) - L3*sin(ang.q1)*sin(ang.q2)*sin(ang.q3);
 punto->z = L2*sin(ang.q2) + L3*sin(ang.q2)*cos(ang.q3) + L3*cos(ang.q2)*sin(ang.q3);
 x=punto->x;
 y=punto->y;
 switch (P){

  case 1:
   //punto->x = x;
   //punto->y = y;
   punto->x = punto->x - d2/2;
   punto->y = punto->y - d1/2;
   break;
  case 2:
   //punto->x = y;
   //punto->y = -x;
   punto->x = punto->x + d2/2;
   punto->y = punto->y - d1/2;
   break;

  case 3:
   //punto->x = -x;
   //punto->y = -y;
   punto->x = punto->x - d2/2;
   punto->y = -punto->y + d1/2;
   break;
  case 4:
   //punto->x = -x;
   //punto->y = -y;
   punto->x = punto->x + d2/2;
   punto->y = -punto->y + d1/2;
   break;
  }
}

/****************FUNCION QUE CALCULA LA CINEMATICA INVERSA******************/

void Cin_Inversa(int P, punto3d PuntoVistoCM, Articulaciones* sol2) {



 float Pxy, L23, beta1, beta2, alpha2, theta;

 punto3d PuntoVistoPata;


 switch (P) {
   case 1:
   PuntoVistoPata.x= PuntoVistoCM.x + d2/2;
   PuntoVistoPata.y= PuntoVistoCM.y + d1/2;
   break;

  case 2:
   PuntoVistoPata.x= PuntoVistoCM.x - d2/2;
   PuntoVistoPata.y= PuntoVistoCM.y + d1/2;
   break;

  case 3:
   PuntoVistoPata.x= d2/2 + PuntoVistoCM.x;
   PuntoVistoPata.y= -d1/2 + PuntoVistoCM.y;
   break;
  case 4:
   PuntoVistoPata.x= -d2/2 + PuntoVistoCM.x;
   PuntoVistoPata.y= -d1/2 + PuntoVistoCM.y;
   break;
   }

 sol2->q1 = atan2((PuntoVistoPata.y),(PuntoVistoPata.x));


 Pxy = sqrt((PuntoVistoPata.x - L1*cos(sol2->q1))*(PuntoVistoPata.x - L1*cos(sol2->q1)) + (PuntoVistoPata.y - L1*sin(sol2->q1))*(PuntoVistoPata.y - L1*sin(sol2->q1)));
 L23= sqrt(Pxy*Pxy + PuntoVistoCM.z*PuntoVistoCM.z);

 beta1 = acos((-L23*L23 +L2*L2 +L3*L3)/(2*L2*L3));
 beta2 = -acos((-L23*L23 +L2*L2 +L3*L3)/(2*L2*L3));


 sol2->q3 = M_PI - beta2;


 alpha2 = asin(L3*sin(beta2)/L23);

 theta = atan2(PuntoVistoCM.z,Pxy);


 sol2->q2 = theta - alpha2;

 sol2->q1=sol2->q1 +M_PI/2;
 sol2->q2=M_PI/2-sol2->q2;
 sol2->q3=-(sol2->q3 + 2*M_PI-2.3-M_PI/2);

  /*Correcciones Cuadrupedo*/
 if(P==3 || P==4)
 sol2->q1=-M_PI+sol2->q1;

 sol2->q2 =sol2->q2 -M_PI/2;
 sol2->q3=sol2->q3 +0.8416 +6.283185;

 //printf("%f %f %f\n", sol2->q1*180/M_PI, sol2->q2*180/M_PI, sol2->q3*180/M_PI);


}

/*****FUNCION QUE DETERMINA SI HAY UNA OBSTÁCULO EN UN PUNTO DETERMINADO EN UNA TRAYECTORIA*****/
void det_colision(int pata, float t, float beta, double fuerza){

//Detecto que hay fuerza en la parte de transición de la trayectoria -> Obstáculo
if (fuerza  != 0 && (t > beta)){
    printf("Se ha detectado una colision por la pata %d en t=%0.2f\n", pata, t);
    }

//Detecto que no hay fuerza en la parte de apoyo de la trayectoria -> Hueco
if (fuerza == 0 && (t < beta)){
    printf("Se ha detectado un hueco por la pata %d en t=%0.2f\n", pata, t);
    }
}

/**********FUNCION QUE CALCULA EL CENTRO DE MASA DE UN ESLABÓN**********/
void cm_eslabon(punto3d* centro, Articulaciones angulo, int articulacion, float L){

Articulaciones ang;
 ang.q2 =angulo.q2 +M_PI/2;
 ang.q3=angulo.q3 -0.8416;
 //Correcciones para estar de acuerdo con el modelo de Webots
 ang.q1 = angulo.q1 - M_PI/2;
 //printf("ojo %f %f\n",M_PI,-ang.q2);
 ang.q2 = M_PI/2 - ang.q2;
 //printf("%f\n",ang.q1);

 //printf("%f\n",ang.q2);

 ang.q3 = (-ang.q3 + 2.3 + M_PI/2)-2*M_PI;
 //printf("%f %f %f\n",ang.q1*180/M_PI,ang.q2*180/M_PI,ang.q3*180/M_PI);
 //printf("%f\n",ang.q3);

if (articulacion == 3){
    centro->x = L1*cos(ang.q1) + L2*cos(ang.q1)*cos(ang.q2) + L*cos(ang.q1)*(cos(ang.q2+ang.q3));
    centro->y = L1*sin(ang.q1) + L2*sin(ang.q1)*cos(ang.q2) + L*sin(ang.q1)*(cos(ang.q2+ang.q3));
    centro->z = L2*sin(ang.q2) + L*(sin(ang.q2+ang.q3));
}else if (articulacion == 2){
    centro->x = cos(ang.q1)*(L1 + L*cos(ang.q2));
    centro->y = sin(ang.q1)*(L1 + L*cos(ang.q2));
    centro->z = L*sin(ang.q2);
}else if (articulacion == 1){
    centro->x = cos(ang.q1)*L;
    centro->y = sin(ang.q1)*L;
    centro->z = 0;
}
else{
    printf("No es posible sacar el calculo, articulacion invalida!");
}
}

/**********FUNCION QUE CALCULA EL CENTRO DE MASA DE UN PATA EN FUNCIÓN DE SUS ESLABÓN**********/
void cm_pata(punto3d* cm, punto3d esl1, punto3d esl2, punto3d esl3){

cm->x = (esl1.x*masa1 + esl2.x*masa2 + esl3.x*masa3);
cm->y = (esl1.y*masa1 + esl2.y*masa2 + esl3.y*masa3);
cm->z = (esl1.z*masa1 + esl2.z*masa2 + esl3.z*masa3);
}

/*****FUNCION QUE CALCULA EL CENTRO DE MASA DEL CUADRUPEDO TOMANDO SUS 4 PATAS******/
void cm_cuad(punto3d* cm, punto3d pata1, punto3d pata2, punto3d pata3, punto3d pata4){

float masa = masa1 + masa2 + masa3;
/*pata1->X = masa*((pata1->X/masa)-d2/2);
pata1->Y = masa*((-pata1->X/masa)-d1/2);

pata2->X = masa*((pata2->X/masa)+d2/2);
pata2->Y = masa*((-pata2->X/masa)-d1/2);

pata3->X = masa*((pata3->X/masa)-d2/2);
pata3->Y = masa*((pata3->X/masa)+d1/2);

pata4->X = masa*((pata4->X/masa)+d2/2);
pata4->Y = masa*((+pata4->X/masa)+d1/2);
*/
cm->x = ((masa*((-pata1.x/masa)-d2/2)+masa*((pata2.x/masa)+d2/2)+masa*((-pata3.x/masa)-d2/2)+masa*((pata4.x/masa)+d2/2)) + 0*masa_base)/(4*masa+masa_base);
cm->y = ((masa*((-pata1.y/masa)-d1/2)+masa*((-pata2.y/masa)-d1/2)+masa*((pata3.y/masa)+d1/2)+masa*((+pata4.y/masa)+d1/2)) + 0*masa_base)/(4*masa+masa_base);
cm->z = ((pata1.z+pata2.z+pata3.z+pata4.z) + 0*masa_base)/(masa+masa_base);

}

/**********FUNCION QUE DEVUELVE LOS GRADOS DE LA ARTICULACIONES DE UNA PATA***********/
/*void pos_servos(Articulaciones* ang, int pata){

    if(pata==1){
        ang.q1 = servo_get_position(p1q1);
        ang.q2 = servo_get_position(p1q2);
        ang.q3 = servo_get_position(p1q3);
    }
    if(pata==2){
        ang->q1 = servo_get_position(p2q1);
        ang->q2 = servo_get_position(p2q2);
        ang->q3 = servo_get_position(p2q3);
    }
    if(pata==3){
        ang->q1 = servo_get_position(p3q1);
        ang->q2 = servo_get_position(p3q2);
        ang->q3 = servo_get_position(p3q3);
    }
    if(pata==4){
        ang->q1 = servo_get_position(p4q1);
        ang->q2 = servo_get_position(p4q2);
        ang->q3 = servo_get_position(p4q3);
    }
}
*/
/*****FUNCIÓN QUE SACA EL MARGEN DE ESTABILIDAD Y CENTROIDE EMPLEANDO LIBRERIAS DE CONVEXHULL*****/
float me_cuad(punto3d P, punto3d* centro, double f1, double f2, double f3, double f4, Articulaciones p1, Articulaciones p2, Articulaciones p3, Articulaciones p4){

double f_minima = 0.00001;
vector3d v;
v = vector3d(0,0,1);        // OJO: hay que definir si se toma en consideración otra fuerza además de la gravedad
punto3d centro_masa;

if(f1<=f_minima || f2<=f_minima || f3<=f_minima || f4<=f_minima){ //Si hay una pata que NO está apoyada
    punto3d poligono[3];
    punto3d proyectado[3];
    //printf("Hay 3 patitas");
    if(f1<=f_minima){                           // Determino la pata que no esta apoyada
        Cin_directa(2, p2, &poligono[0]);       // y defino el poligono de apoyo con las otras
        Cin_directa(3, p3, &poligono[1]);
        Cin_directa(4, p4, &poligono[2]);
    } else if(f2<=f_minima){
        Cin_directa(1, p2, &poligono[0]);
        Cin_directa(3, p3, &poligono[1]);
        Cin_directa(4, p4, &poligono[2]);
    } else if(f3<=f_minima){
        Cin_directa(1, p2, &poligono[0]);
        Cin_directa(2, p3, &poligono[1]);
        Cin_directa(4, p4, &poligono[2]);
    } else {
        Cin_directa(1, p2, &poligono[0]);
        Cin_directa(2, p3, &poligono[1]);
        Cin_directa(3, p4, &poligono[2]);
    }
    plano3d plano = planoProyectar(v, &poligono[0], proyectado, 3); // Genero el plano normal a la gravedad
    *centro = centroide(proyectado, 3);                             // Calculo el centroide del plano de apoyo
    centro_masa = plano.proyeccion(P);
    int cuantos = convexhull_graham(proyectado, poligono, 3);       // Se hace el Graham Scan para ordena los puntos, requisisto del margen
    return margen_est(P, proyectado, cuantos);                      // Calculo el margen de estabilidad del punto P.

} else{                                         // Todas las patas están apoyadas
    //printf("Hay 4 patitas");
    punto3d poligono[4];
    punto3d proyectado[4];
    Cin_directa(1, p1, &poligono[0]);          // Defino el polígono de apoyo dado por las 4 patas
    Cin_directa(2, p2, &poligono[1]);
    Cin_directa(3, p3, &poligono[2]);
    Cin_directa(4, p4, &poligono[3]);
    plano3d plano = planoProyectar(v, &poligono[0], proyectado, 4); // Genero el plano normal a la gravedad
    *centro = centroide(proyectado, 4);                             // Calculo el centroide del plano de apoyo
    centro_masa = plano.proyeccion(P);
    int cuantos = convexhull_graham(proyectado, poligono, 4);       // Se hace el Graham Scan para ordena los puntos, requisisto del margen
    return margen_est(centro_masa, poligono, cuantos);              // Calculo el margen de estabilidad del punto P.
}

}

punto3d punto_me_cuad(punto3d P, double f1, double f2, double f3, double f4, Articulaciones p1, Articulaciones p2, Articulaciones p3, Articulaciones p4){

double f_minima = 0.00001;
vector3d v;
v = vector3d(0,0,1);        // OJO: hay que definir si se toma en consideración otra fuerza además de la gravedad
punto3d centro_masa;

if(f1<=f_minima || f2<=f_minima || f3<=f_minima || f4<=f_minima){ //Si hay una pata que NO está apoyada
    punto3d poligono[3];
    punto3d proyectado[3];
    //printf("Hay 3 patitas");
    if(f1<=f_minima){                           // Determino la pata que no esta apoyada
        Cin_directa(2, p2, &poligono[0]);       // y defino el poligono de apoyo con las otras
        Cin_directa(3, p3, &poligono[1]);
        Cin_directa(4, p4, &poligono[2]);
    } else if(f2<=f_minima){
        Cin_directa(1, p2, &poligono[0]);
        Cin_directa(3, p3, &poligono[1]);
        Cin_directa(4, p4, &poligono[2]);
    } else if(f3<=f_minima){
        Cin_directa(1, p2, &poligono[0]);
        Cin_directa(2, p3, &poligono[1]);
        Cin_directa(4, p4, &poligono[2]);
    } else {
        Cin_directa(1, p2, &poligono[0]);
        Cin_directa(2, p3, &poligono[1]);
        Cin_directa(3, p4, &poligono[2]);
    }
    plano3d plano = planoProyectar(v, &poligono[0], proyectado, 3); // Genero el plano normal a la gravedad
    int cuantos = convexhull_graham(proyectado, poligono, 3);       // Se hace el Graham Scan para ordena los puntos, requisisto del margen
    return punto_est(P, proyectado, cuantos);                      // Calculo el margen de estabilidad del punto P.

} else{                                         // Todas las patas están apoyadas
    //printf("Hay 4 patitas");
    punto3d poligono[4];
    punto3d proyectado[4];
    Cin_directa(1, p1, &poligono[0]);          // Defino el polígono de apoyo dado por las 4 patas
    Cin_directa(2, p2, &poligono[1]);
    Cin_directa(3, p3, &poligono[2]);
    Cin_directa(4, p4, &poligono[3]);
    plano3d plano = planoProyectar(v, &poligono[0], proyectado, 4); // Genero el plano normal a la gravedad
    centro_masa = plano.proyeccion(P);
    int cuantos = convexhull_graham(proyectado, poligono, 4);       // Se hace el Graham Scan para ordena los puntos, requisisto del margen
    return punto_est(centro_masa, poligono, cuantos);              // Calculo el margen de estabilidad del punto P.
}

}
