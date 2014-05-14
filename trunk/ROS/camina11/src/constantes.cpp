#include "ros/ros.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "camina11/constantes.hpp"
#include "camina11/vector3d.hpp"
//Descripcion del cuerpo del robot
float rotacionPata[Npatas]={30*pi/180,-30*pi/180,90*pi/180,-90*pi/180,150*pi/180,-150*pi/180};
float origenPatas_x[Npatas]={-posicionPata1_x,posicionPata1_x,-posicionPata2_x,posicionPata2_x,-posicionPata1_x, posicionPata1_x};
float origenPatas_y[Npatas]={ posicionPata1_y,posicionPata1_y, posicionPata2_y,posicionPata2_y,-posicionPata1_y,-posicionPata1_y};
//origenPatas[0].x=-posicionPata1_x;	origenPatas[3].x= posicionPata2_x;
//origenPatas[0].y= posicionPata1_y; 	origenPatas[3].y= posicionPata2_y;
//origenPatas[1].x= posicionPata1_x; 	origenPatas[4].x=-posicionPata1_x;
//origenPatas[1].y= posicionPata1_y; 	origenPatas[4].y=-posicionPata1_y;
//origenPatas[2].x=-posicionPata2_x; 	origenPatas[5].x= posicionPata1_x;
//origenPatas[2].y= posicionPata2_y; 	origenPatas[5].y=-posicionPata1_y;


//-- Funcion para transformar del espacio R2 XY a el espacio de celdas IJ
/*En matriz de mapa las coordenadas van de i=[0,99], j=[0,19] */
void transformacion_yxTOij(int *ptr_ij, float y, float x, int nCeldas_i, int nCeldas_j, float LongitudCeldaY,float LongitudCeldaX){
    ptr_ij[0] = (int) (nCeldas_i/2 - floor(y/LongitudCeldaY)-1);
    ptr_ij[1] = (int) (nCeldas_j/2 + floor(x/LongitudCeldaX));
}
//-- Funcion traslación simple de punto
punto3d TransportaPunto(punto3d Punto_in,float L_traslacion, float ang_rotacion){

    punto3d Punto_out;

    Punto_out.x=Punto_in.x - L_traslacion*sin(ang_rotacion);
    Punto_out.y=Punto_in.y + L_traslacion*cos(ang_rotacion);

    return(Punto_out);
}

//-- Funcion transformacion homogenea de punto
punto3d TransformacionHomogenea(punto3d Punto_in, punto3d L_traslacion, float ang_rotacion){

    punto3d Punto_out;

    Punto_out.x = L_traslacion.x + Punto_in.x*cos(ang_rotacion) - Punto_in.y*sin(ang_rotacion);
    Punto_out.y = L_traslacion.y + Punto_in.x*sin(ang_rotacion) + Punto_in.y*cos(ang_rotacion);
    Punto_out.z = L_traslacion.z + Punto_in.z;

    return(Punto_out);
}

//-- Funcion transformacion homogenea inversa de punto
punto3d TransformacionHomogenea_Inversa(punto3d Punto_in, punto3d L_traslacion, float ang_rotacion){

    punto3d Punto_out;

    Punto_out.x = (-L_traslacion.x*cos(ang_rotacion)-L_traslacion.y*sin(ang_rotacion)) + Punto_in.x*cos(ang_rotacion) + Punto_in.y*sin(ang_rotacion);
    Punto_out.y = (L_traslacion.x*sin(ang_rotacion)-L_traslacion.y*cos(ang_rotacion)) - Punto_in.x*sin(ang_rotacion) + Punto_in.y*cos(ang_rotacion);
    Punto_out.z = (-L_traslacion.z) + Punto_in.z;
//    Punto_out.z = Punto_in.z;

    return(Punto_out);
}

punto3d Transformada_Mundo_Pata(int nPata, int modo, punto3d P_in, punto3d P_ubicacionRobot, float theta_Rob){
    punto3d P_Robot, P_out, origenPata;

    origenPata.x = origenPatas_x[nPata];
    origenPata.y = origenPatas_y[nPata];
    origenPata.z = 0.0;
//---NOTA: LA TRANSFORMACION SOBRE Z NO SE REALIZA
    switch (modo){
        case Pata_Mundo:
        //--Primera transformacion: Sistema ROBOT en el Sistema MUNDO
            P_Robot = TransformacionHomogenea(P_in,origenPata,rotacionPata[nPata]);
        //--Segunda transformacion: del Sistema PATA en el Sistema MUNDO
            P_out = TransformacionHomogenea(P_Robot,P_ubicacionRobot,theta_Rob);
        break;

        case Mundo_Pata:
        //--Devolvemos segunda transformacion
            P_Robot = TransformacionHomogenea_Inversa(P_in,P_ubicacionRobot,theta_Rob);
        //--Devolvemos primera transformacion: del Sistema ROBOT al Sistema MUNDO
            P_out = TransformacionHomogenea_Inversa(P_Robot,origenPata,rotacionPata[nPata]);
        break;

        default:
            ROS_ERROR("Transformada_Mundo_Pata: Modo incorrecto");
        break;
    }
    return(P_out);
}
