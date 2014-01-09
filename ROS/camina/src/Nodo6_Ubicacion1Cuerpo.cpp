#include <stdio.h>
#include <string.h>
#include <iostream>
#include <stdlib.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
//Librerias propias usadas
#include "constantes.hpp"
#include "camina/v_repConst.h"
// Used data structures:
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include "../msg_gen/cpp/include/camina/UbicacionRobot.h"
// Used API services:
#include "vrep_common/VrepInfo.h"
#include "vrep_common/JointSetStateData.h"
#include "vrep_common/simRosGetObjectPose.h"
#include "vrep_common/simRosEnableSubscriber.h"
#include "vrep_common/simRosReadForceSensor.h"

// Global variables:
bool simulationRunning=true;
bool sensorTrigger=false;
float simulationTime=0.0f;
double tiempo_ahora=0.0, tiempo_anterior=0.0;
float delta_t=0.0;
float delta_x=0.0, delta_y=0.0, x_anterior=0.0, y_anterior=0.0;
//FILE *fp;
//Clientes y Servicios
ros::ServiceClient client_simRosGetObjectPose;
vrep_common::simRosGetObjectPose srv_simRosGetObjectPose; //Servicio para obtener posicion del robot
ros::ServiceClient client_simRosReadForceSensor;
vrep_common::simRosReadForceSensor srv_simRosReadForceSensor; //Servicio para obtener posicion del robot

// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}

// Main code:
int main(int argc,char* argv[])
{
	// (when V-REP launches this executable, V-REP will also provide the argument list)
	//numero de argumentos que mande (se excluye el fantasma que el manda solo)
//	float Periodo=0.0, f=0.0;
	int Narg=0, handleArana=0, k=0;
	camina::UbicacionRobot ubicacionRobot;
    geometry_msgs::PoseStamped CuerpoPose;
    tf::Quaternion CuerpoOrientacion_Q;
    tfScalar roll, pitch, yaw;
    float velocidad_Apoyo=0.0, T=0.0, beta=0.0, alfa=0.0;
//    float PisadaProxima_y=0.0, PisadaProxima_x=0.0;

	Narg=5;

	if (argc>=Narg)
	{
        handleArana=atoi(argv[1]);
        velocidad_Apoyo = atof(argv[2]);
        T = atof(argv[3]);
        beta = atof(argv[4]);
        alfa = atof(argv[5]);
	}
	else
	{
		ROS_ERROR("Nodo6_1:Indique argumentos completos!\n");
		return (0);
	}
	// Create a ROS node:
	int _argc = 0;
	char** _argv = NULL;
	ros::init(_argc,_argv,"Nodo6_Ubicacion1Cuerpo");
	if(!ros::master::check()) return(0);

	ros::NodeHandle node;
	ROS_INFO("Nodo6_Ubicacion1Cuerpo just started\n");

//-- Topicos susbcritos y publicados
    ros::Subscriber subInfo=node.subscribe("/vrep/info",1,infoCallback);
    ros::Publisher chatter_pub = node.advertise<camina::UbicacionRobot>("Ubicacion1Cuerpo", 100);
//-- Clientes y Servicios
    client_simRosGetObjectPose=node.serviceClient<vrep_common::simRosGetObjectPose>("/vrep/simRosGetObjectPose");
//    client_simRosReadForceSensor=node.serviceClient<vrep_common::simRosReadForceSensor>("/vrep/simRosReadForceSensor");
    srv_simRosGetObjectPose.request.relativeToObjectHandle=-1;

//-- Creamos archivo para guardar posiciones de patas
//    fp = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina/datos/Nodo6.txt","w+");
//    int cuentaPataApoyo=0, pataApoyo[Npatas];
    /* Velocidad de transmision */
//    Periodo = 0.5;
//    f=1/Periodo;
//    ros::Rate loop_rate(f);  //Frecuencia [Hz]
	while (ros::ok() && simulationRunning)
	{
//		printf("Nodo6_1\n");
		ros::spinOnce();
//		loop_rate.sleep();
    //-- Primero buscamos posicion del cuerpo de Arana
		srv_simRosGetObjectPose.request.handle=handleArana;

	    if (client_simRosGetObjectPose.call(srv_simRosGetObjectPose)&&(srv_simRosGetObjectPose.response.result!=-1))
        {
            x_anterior = ubicacionRobot.coordenadaCuerpo_x;
            y_anterior = ubicacionRobot.coordenadaCuerpo_y;

            CuerpoPose = srv_simRosGetObjectPose.response.pose;
//            ROS_INFO("posicion: x=%.3f; y=%.3f; z=%.3f\n", CuerpoPose.pose.position.x,CuerpoPose.pose.position.y,CuerpoPose.pose.position.z);
            ubicacionRobot.coordenadaCuerpo_x = CuerpoPose.pose.position.x;
            ubicacionRobot.coordenadaCuerpo_y = CuerpoPose.pose.position.y;
            tf::quaternionMsgToTF(CuerpoPose.pose.orientation,CuerpoOrientacion_Q);
            tf::Matrix3x3(CuerpoOrientacion_Q).getRPY(roll, pitch, yaw);
            ubicacionRobot.orientacionCuerpo_roll = roll;
            ubicacionRobot.orientacionCuerpo_pitch = pitch;
            ubicacionRobot.orientacionCuerpo_yaw = yaw;
//            ROS_INFO("orientacion: z=%.3f\n", ubicacionRobot.orientacionCuerpo_yaw);

            tiempo_ahora = ros::Time::now().toSec();
            delta_t = (float) (tiempo_ahora - tiempo_anterior);

            delta_x = fabs(ubicacionRobot.coordenadaCuerpo_x-x_anterior);
            delta_y = fabs(ubicacionRobot.coordenadaCuerpo_y-y_anterior);
//            ROS_INFO("delta_t=%.3f, delta_x=%.3f, delta_y=%.3f\n",delta_t,delta_x,delta_y);
            if (delta_t==0) {
                ubicacionRobot.velocidadCuerpo_x = 0.0;
                ubicacionRobot.velocidadCuerpo_y = 0.0;
            } else {
                ubicacionRobot.velocidadCuerpo_x = delta_x/delta_t;
                ubicacionRobot.velocidadCuerpo_y = delta_y/delta_t;
            }
//            ROS_INFO("v_x=%.3f, v_y=%.3f\n",ubicacionRobot.velocidadCuerpo_x,ubicacionRobot.velocidadCuerpo_y);
            tiempo_anterior = tiempo_ahora;
        } else {
             ROS_ERROR("Nodo6_1: servicio de posicion cuerpo no funciona\n");
        }
        chatter_pub.publish(ubicacionRobot);
	}
//	fclose(fp);
	ROS_INFO("Adios6_1!");
	ros::shutdown();
	return(0);
}
