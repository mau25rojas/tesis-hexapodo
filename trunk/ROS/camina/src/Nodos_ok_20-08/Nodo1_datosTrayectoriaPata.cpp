#include "ros/ros.h"
#include "std_msgs/String.h"
#include "math.h"
//Librerias propias usadas
#include "constantes.hpp"
#include "camina/v_repConst.h"
// Used data structures:
#include "vrep_common/VrepInfo.h"
#include <geometry_msgs/PoseStamped.h>
#include "../msg_gen/cpp/include/camina/DatosTrayectoriaPata.h"
#include "../msg_gen/cpp/include/camina/SenalesCambios.h"
// Used API services:
#include "vrep_common/simRosGetObjectPose.h"

// variables Globales
ros::Publisher chatter_pub1;
bool simulationRunning=true;
bool sensorTrigger=false;
float simulationTime=0.0f;
float delta_t=0.0;
int cuenta=0;
camina::DatosTrayectoriaPata datoTrayectoriaPata;
//camina::SenalesCambios senalesCambios;

// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}

void datosCallback(const camina::DatosTrayectoriaPata msg_DatosTrayectoriaPata)
{
//        senalesCambios.cambioPata=false;
        datoTrayectoriaPata.t_Trayectoria = delta_t;
        datoTrayectoriaPata.iteracion=cuenta;
        delta_t = delta_t + 1/msg_DatosTrayectoriaPata.divisionTrayectoriaPata;
        ++cuenta;
        if (fabs(delta_t-1)<0.001)
        {
            delta_t=0.0;
            cuenta=0;
        }
        datoTrayectoriaPata.pataMovimiento= msg_DatosTrayectoriaPata.pataMovimiento;
        datoTrayectoriaPata.beta = msg_DatosTrayectoriaPata.beta;
        datoTrayectoriaPata.landa = msg_DatosTrayectoriaPata.landa;
        datoTrayectoriaPata.dh = msg_DatosTrayectoriaPata.dh;
        datoTrayectoriaPata.p_Offset = msg_DatosTrayectoriaPata.p_Offset;
        datoTrayectoriaPata.alfa = msg_DatosTrayectoriaPata.alfa;
        datoTrayectoriaPata.phi = msg_DatosTrayectoriaPata.phi;
        datoTrayectoriaPata.desfasaje_t=msg_DatosTrayectoriaPata.desfasaje_t;
        //ROS_INFO("%.3f, %.3f, %.3f\n",datoTrayectoriaPata.p_Offset[0],datoTrayectoriaPata.p_Offset[1],datoTrayectoriaPata.p_Offset[2])
        chatter_pub1.publish(datoTrayectoriaPata);
//        chatter_pub2.publish(senalesCambios);
}

int main(int argc, char **argv)
{
  /*Inicio nodo de ROS*/
  ros::init(argc, argv, "Nodo1_datosTrayectoriaPata");
  ros::NodeHandle n;
  ROS_INFO("Nodo1_datosTrayectoriaPata just started\n");

  //Topicos susbcritos y publicados
  chatter_pub1 = n.advertise<camina::DatosTrayectoriaPata>("datosTrayectoriaPataSalida", 100);
//  chatter_pub2 = n.advertise<camina::SenalesCambios>("CambioPata", 100);
  ros::Subscriber sub = n.subscribe("datosTrayectoriaPataEntrada", 100, datosCallback);
  ros::Subscriber subInfo=n.subscribe("/vrep/info",1,infoCallback);


  while (ros::ok() && simulationRunning)
  {
        ros::spinOnce();
  }

    ROS_INFO("Adios1!");
    ros::shutdown();
    return 0;
}

