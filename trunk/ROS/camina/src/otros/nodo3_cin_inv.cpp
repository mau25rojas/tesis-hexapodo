#include "ros/ros.h"
//usando los valores de cada uno de los eslabones (definidos en "constantes.hpp")
#include "constantes.hpp"
#include "../msg_gen/cpp/include/camina/coordenadas.h"
#include "../msg_gen/cpp/include/camina/motor.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>

#define pi 3.141592

void cinversa1 (float *p, float x, float y, float z);
void cinversa2 (float *p, float x, float y, float z);

camina::coordenadas coord;
camina::motor mq;

ros::Publisher chatter_pub;

void coordCallback(const camina::coordenadas msg)
{
  float Px=0, Py=0, Pz=0, count;
  float q[3] = {0,0,0};	//Angulos de motores q1, q2, q3
  float *ptr;

    ptr = q;

	Px = msg.x;
	Py = msg.y;
	Pz = msg.z;
	count = msg.count;

        //Pata 1
        mq.Npata = 1;
        cinversa1(ptr,Px,Py,Pz);
        mq.q1 = q[0];
        mq.q2 = q[1];
        mq.q3 = q[2];

    ROS_INFO("I say:[count=%.3f, pata%.1f, q1 = %.3f, q2 = %.3f, q3 = %.3f]", msg.count, mq.Npata, q[0],q[1],q[2]);

   chatter_pub.publish(mq);
}

int main(int argc, char **argv){

  ros::init(argc, argv, "cin_inv");

  ros::NodeHandle n;

  chatter_pub = n.advertise<camina::motor>("motores", 1000);
  ros::Subscriber sub = n.subscribe("coordenadas", 1000, coordCallback);

  ros::Rate loop_rate(0.25);

  ros::spin();

  return 0;


}

/*Funcion de cinematica inversa

*/


//Pata1
void cinversa1 (float *p, float x, float y, float z){

	float p1=0, p2=0, L23=0, beta=0, teta=0, gamma=0, arg=0, arg1=0, px=0, py=0, pz=0;
	float xoff=0.1299, yoff=0.075, zoff=0.05;

    px = cos(-pi/3)*(x-xoff) + sin(-pi/3)*(y-yoff);
    py = -sin(-pi/3)*(x-xoff) + cos(-pi/3)*(y-yoff);
    pz = z-zoff;

    //calculo q1
	p[0] = atan2(px,py);

	p1 = sqrt(px*px + py*py) - L1;
	p2 = pz;

	L23 = sqrt(p1*p1 + p2*p2);

    //calculo q3
	arg = (L2*L2 + L3*L3 - L23*L23)/(2*L2*L3);
	if (fabs(arg)>=1) {
        ROS_INFO("ERROR acos");
        return;
	}
	beta = acos(arg);
    p[2] = pi - beta;

	//calculo q2
	arg1= (L3/L23)*sin(beta);
	if (abs(arg1)>=1) {
        ROS_INFO("ERROR asin");
        return;
	}
	teta = atan2(-p2,p1);
	gamma = asin(arg1);
	p[1] = pi/2 - (teta - gamma);
}

//Pata 2
void cinversa2 (float *p, float x, float y, float z){

	float p1=0, p2=0, L23=0, beta=0, teta=0, gamma=0, arg=0, arg1=0, px=0, py=0, pz=0;
	float xoff=0.1299, yoff=0.075, zoff=0.05;

    px = cos(-pi/3)*(x-xoff) + sin(-pi/3)*(y-yoff);
    py = -sin(-pi/3)*(x-xoff) + cos(-pi/3)*(y-yoff);
    pz = z-zoff;

    //calculo q1
	p[0] = -atan2(px,py);

	p1 = sqrt(px*px + py*py) - L1;
	p2 = pz;

	L23 = sqrt(p1*p1 + p2*p2);

    //calculo q3
	arg = (L2*L2 + L3*L3 - L23*L23)/(2*L2*L3);
	if (fabs(arg)>=1) {
        ROS_INFO("ERROR acos");
        return;
	}
	beta = acos(arg);
    p[2] = -(pi - beta);

	//calculo q2
	arg1= (L3/L23)*sin(beta);
	if (abs(arg1)>=1) {
        ROS_INFO("ERROR asin");
        return;
	}
	teta = atan2(-p2,p1);
	gamma = asin(arg1);
	p[1] = -(pi/2 - (teta - gamma));

}
