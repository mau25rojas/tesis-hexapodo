#include "ros/ros.h"
#include "../msg_gen/cpp/include/camina/datos_ini.h"
#include "../msg_gen/cpp/include/camina/coordenadas.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>

void trayectoria (float *p, float landa, float dh, float vel_1, float vel_2, float t, int s);

camina::coordenadas coord;
camina::datos_ini dat;

ros::Publisher chatter_pub;

void datosCallback(const camina::datos_ini msg)
{
  float t_2=0;
  float coord_1[3] = {0, 0, 0};   //coordenadas X, Y, Z

	t_2 = msg.t;

        coord_1[0] = 0.2271;
        coord_1[1] = 0.1311;       //La posicion Y por default es
        coord_1[2] = -0.0475;

   coord.x = coord_1[0];
   coord.y = coord_1[1];
   coord.z = coord_1[2];
   coord.count = t_2;

    ROS_INFO("I say:[count = %.3f, x = %.3f, y = %.3f, z = %.3f]", coord.count,coord.x,coord.y,coord.z);

   chatter_pub.publish(coord);
}

int main(int argc, char **argv){

  ros::init(argc, argv, "parametrizacion");

  ros::NodeHandle n;

  chatter_pub = n.advertise<camina::coordenadas>("coordenadas", 1000);
  ros::Subscriber sub = n.subscribe("datos", 1000, datosCallback);

  ros::Rate loop_rate(1);

  ros::spin();

  return 0;


}

/*Funcion de calculo de trayectoria

Parametros:
landa: dlongitud de paso (metros)
dh: altura de levantamiento de pata (metros)
beta: intervalo de tiempo de apoyo (fraccion del periodo) - (normalizado entre 0-1)
t: fraccion del periodo. Cada cuanto se actualiza la trayectoria - (normalizado entre 0-1)
*/

void trayectoria (float *p, float landa, float dh, float vel_1, float vel_2, float t, int s){

float coord_z=0.0, coord_x=0.0;

    switch (s){

    case 0:

        coord_x = -landa/2 + vel_1*t ;
        coord_z = -dh/2;

    break;

    case 1:

        coord_x = landa/2;
        coord_z = - dh/2 + vel_2*t;

    break;

    case 2:

        coord_x = landa/2 - vel_2*t;
        coord_z = dh/2;

    break;

    case 3:

        coord_x = -landa/2;
        coord_z = dh/2 - vel_2*t;

    break;
    }

    p[0] = coord_x;
    p[2] = coord_z;


}

