#include "ros/ros.h"
#include "std_msgs/String.h"
#include "math.h"
#include "../msg_gen/cpp/include/camina/datos_ini.h"


camina::datos_ini dat;

int main(int argc, char **argv)
{
  float T=0, T1=0, div=0, t_aux=0, vel_1=0, vel_2=0;
  float t1=0,t2=0,t3=0,t4=0;

  ros::init(argc, argv, "datos");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<camina::datos_ini>("datos", 100);

  ros::Rate loop_rate(10);

  //Periodo total
  T = 1;
  //Divisiones del periodo total
  div = 100;
  T1 = T/div;

  int count = 0;
  while (ros::ok() && t_aux<=1)
  {
     dat.beta = 0.5;
     dat.landa = 0.15;	// 15cm
     dat.dh = 0.05; 	// 5cm

     t_aux = t_aux + T1;
     dat.t = t_aux;

     vel_1 = dat.landa/dat.beta;
	 vel_2 = (2*dat.dh+dat.landa)/(1-dat.beta);

    	if ((0<=t_aux) && (t_aux<dat.beta))
        {
            t1 = t_aux;
            dat.t1 = t1;
        }
    	if ((dat.beta<=t_aux) && (t_aux<((dat.dh/vel_2)+dat.beta)))
    	{
            t2 = t_aux;
            dat.t1 = t2-t1;
        }
    	if (((dat.dh/vel_2)+dat.beta<=t_aux) && (t_aux<((dat.dh+dat.landa)/vel_2)+dat.beta))
    	{
            t3 = t_aux;
            dat.t1 = t3-t2;
        }
    	if ((((dat.dh+dat.landa)/vel_2)+dat.beta)<=t_aux && (t_aux<1))
    	{
            t4 = t_aux;
            dat.t1 = t4-t3;
        }

    ROS_INFO("datos publicando: %i \t t:%.3f \t t1:%.3f", count, dat.t, dat.t1);

    chatter_pub.publish(dat);

    ros::spinOnce();

    ++count;

    loop_rate.sleep();
  }


  return 0;
}
