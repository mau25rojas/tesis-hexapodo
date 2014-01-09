#include "ros/ros.h"
//usando los valores de cada uno de los eslabones (definidos en "constantes.hpp")
#include "constantes.hpp"
#include "../msg_gen/cpp/include/camina/datos_ini.h"
#include "../msg_gen/cpp/include/camina/motor.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>

#define pi 3.141592

void cinversa (float *p, float x, float y, float z);
void trayectoria (float *p, float landa, float dh, float vel_1, float vel_2, float t, int s);

camina::datos_ini dat;
camina::motor mq;

ros::Publisher chatter_pub;

void datosCallback(const camina::datos_ini msg)
{
  int s_2=0;
  float beta_2=0, landa_2=0, dh_2=0, t_1=0, t_2=0, vel_1=0, vel_2=0;
  float coord_1[3] = {0, 0, 0};   //coordenadas X, Y, Z
  float q[3] = {0,0,0};	//Angulos de motores q1, q2, q3
  float *ptr1, *ptr2;
  float y_off=0.0, z_off=0.0;

    //---Parametros temporales----
	beta_2 = msg.beta;
	landa_2 = msg.landa;
	dh_2 = msg.dh;
	t_1 = msg.t1;
	t_2 = msg.t;
	mq.count=msg.count;

	vel_1 = landa_2/beta_2;
	vel_2 = (2*dh_2+landa_2)/(1-beta_2);

    //---------------------------------

    //---Offsets para la trayectoria-----
    y_t = 0.1;
    z_t = -0.05;

    //---Apuntadores a salidas----
    ptr1 = coord_1;
    ptr2 = q;


    //---Caso parte 1 trayectoria----

    	if ((0<=t_2) && (t_2<beta_2))
        {
            s_2=0;
            trayectoria(ptr1, landa_2, dh_2, vel_1, vel_2, t_1, s_2);

            //Pata 3
            mq.Npata = 3;
            
	 cinversa(ptr2,coord_1[0],coord_1[1],coord_1[2]);
            mq.q1 = q[0];
            mq.q2 = q[1];
            mq.q3 = q[2];

        }
    //---------------------------------
    	
    //---Caso parte 2 trayectoria----

     if ((beta_2<=t_2) && (t_2<(dh_2/vel_2)+beta_2))
    	{
            s_2=1;
            trayectoria(ptr1, landa_2, dh_2, vel_1, vel_2, t_1, s_2);

            //Pata 3
            mq.Npata = 3;
            cinversa(ptr2,coord_1[0],coord_1[1],coord_1[2]);
            mq.q1 = q[0];
            mq.q2 = q[1];
            mq.q3 = q[2];

        }
      //---------------------------------
    	
      //---Caso parte 3 trayectoria----

     if (((dh_2/vel_2)+beta_2<=t_2) && (t_2<((dh_2+landa_2)/vel_2)+beta_2))
    	{
            s_2=2;
            trayectoria(ptr1, landa_2, dh_2, vel_1, vel_2, t_1, s_2);
	    //Pata 3
            mq.Npata = 3;

            cinversa(ptr2,coord_1[0],coord_1[1],coord_1[2]);
            mq.q1 = q[0];
            mq.q2 = q[1];
            mq.q3 = q[2];

        }
      //---------------------------------
    	
      //---Caso parte 4 trayectoria----


    	if ((((dh_2+landa_2)/vel_2)+beta_2)<=t_2 && (t_2<=1))
    	{
            s_2=3;
            trayectoria(ptr1, landa_2, dh_2, vel_1, vel_2, t_1, s_2);
	
            //Pata 3
            mq.Npata = 3;
            
	    cinversa(ptr2,coord_1[0],coord_1[1],coord_1[2]);
            mq.q1 = q[0];
            mq.q2 = q[1];
            mq.q3 = q[2];

        }
      //--------------------------------- 	
      //ROS_INFO("I say1:[count =%.3f, x = %.3f, y = %.3f, z = %.3f]",  t_2, coord_1[0], coord_1[1], coord_1[2]);
      ROS_INFO("I say2:[count=%.1f, pata=%.1f, q1 = %.3f, q2 = %.3f, q3 = %.3f]",mq.count,mq.Npata, q[0],q[1],q[2]);

	//---Publica angulos motores----   
	chatter_pub.publish(mq);
}

int main(int argc, char **argv){

  ros::init(argc, argv, "parametrizacion");

  ros::NodeHandle n;

  chatter_pub = n.advertise<camina::motor>("motores", 100);
  ros::Subscriber sub = n.subscribe("datos", 100, datosCallback);

//  ros::Rate loop_rate(0.25);

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

//--------Funcion de cinematica inversa--------------------------

void cinversa (float *p, float x, float y, float z){

	float ang_rot = 0;
	float p1=0, p2=0, L23=0, beta=0, teta=0, gamma=0, arg=0, arg1=0, px=0, py=0, pz=0;
	float xoff=0, yoff=0.15, zoff=0.05;

    px = cos(-ang_rot)*(x+xoff) + sin(-ang_rot)*(y+yoff);
    py = -sin(-ang_rot)*(x+xoff) + cos(-ang_rot)*(y+yoff);
    pz = z+zoff;

    //calculo q1
	p[0] = atan2(px,py);

	p1 = sqrt(px*px + py*py) - L1;
	p2 = pz;

	L23 = sqrt(p1*p1 + p2*p2);

    //calculo q3
	arg = (L2*L2 + L3*L3 - L23*L23)/(2*L2*L3);
	if (fabs(arg)>=1) {
        ROS_INFO("ERROR acos q3");
        return;
	}
	beta = acos(arg);
    p[2] = pi - beta;

	//calculo q2
	arg1= (L3/L23)*sin(beta);
	if (abs(arg1)>=1) {
        ROS_INFO("ERROR asin q2");
        return;
	}
	teta = atan2(-p2,p1);
	gamma = asin(arg1);
	//p[1] = pi/2 - (teta - gamma);
	p[1] = teta - gamma;
}


