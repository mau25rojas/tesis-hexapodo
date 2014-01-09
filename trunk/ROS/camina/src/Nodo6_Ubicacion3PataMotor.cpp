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
FILE *fp;
//Clientes y Servicios
ros::ServiceClient client_simRosGetObjectPose;
vrep_common::simRosGetObjectPose srv_simRosGetObjectPose; //Servicio para obtener posicion del robot

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
    int Narg=0, k=0 ;
	int PataTipHandle[Npatas], Pata_Motor1Handle[Npatas];
	camina::UbicacionRobot ubicacionRobot;
    geometry_msgs::PoseStamped PataTipPose[Npatas];
//    float PisadaProxima_y=0.0, PisadaProxima_x=0.0;

	Narg=12;

	if (argc>=Narg)
	{
        for (k=0;k<Npatas;k++) PataTipHandle[k] = atoi(argv[1+k]);
        for (k=0;k<Npatas;k++) Pata_Motor1Handle[k] = atoi(argv[1+Npatas+k]);
	}
	else
	{
		ROS_ERROR("Nodo6_3:Indique argumentos completos!\n");
		return (0);
	}
//-- Inicializacion de variables del mensaje
    for (k=0;k<Npatas;k++) {
        ubicacionRobot.coordenadaPataSistemaPata_y.push_back(0);
        ubicacionRobot.coordenadaPataSistemaPata_x.push_back(0);
        ubicacionRobot.coordenadaPataSistemaPata_z.push_back(0);
    }
	// Create a ROS node:
	int _argc = 0;
	char** _argv = NULL;
	ros::init(_argc,_argv,"Nodo6_Ubicacion3PataMotor");
	if(!ros::master::check()) return(0);

	ros::NodeHandle node;
	ROS_INFO("Nodo6_Ubicacion3PataMotor just started\n");

//-- Topicos susbcritos y publicados
    ros::Subscriber subInfo=node.subscribe("/vrep/info",1,infoCallback);
    ros::Publisher chatter_pub = node.advertise<camina::UbicacionRobot>("Ubicacion3PataMotor", 100);
//-- Clientes y Servicios
    client_simRosGetObjectPose=node.serviceClient<vrep_common::simRosGetObjectPose>("/vrep/simRosGetObjectPose");

//-- Creamos archivo para guardar posiciones de patas
//    fp = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina/datos/Nodo6.txt","w+");
//    int cuentaPataApoyo=0, pataApoyo[Npatas];
    /* Velocidad de transmision */
//    Periodo = 0.5;
//    f=1/Periodo;
//    ros::Rate loop_rate(f);  //Frecuencia [Hz]
	while (ros::ok() && simulationRunning)
	{
//		printf("Nodo6_3\n");
		ros::spinOnce();
//		loop_rate.sleep();
        for (k=0;k<Npatas;k++){
            srv_simRosGetObjectPose.request.handle=PataTipHandle[k];
        //--------------------------------------------------------------
        //---- Obtengo posicion de tip de pata para cada sistema de pata
            srv_simRosGetObjectPose.request.relativeToObjectHandle=Pata_Motor1Handle[k];
            if (client_simRosGetObjectPose.call(srv_simRosGetObjectPose)&&(srv_simRosGetObjectPose.response.result!=-1)){

                PataTipPose[k] = srv_simRosGetObjectPose.response.pose;
//                printf("Pata apoyo[%d]=%d\n",k,ubicacionRobot.pataApoyo[k]);
                ubicacionRobot.coordenadaPataSistemaPata_x[k] = PataTipPose[k].pose.position.x;
                ubicacionRobot.coordenadaPataSistemaPata_y[k] = PataTipPose[k].pose.position.y;
                ubicacionRobot.coordenadaPataSistemaPata_z[k] = PataTipPose[k].pose.position.z;
//                printf("Pata: %d: x= %.3f, y= %.3f, z= %.3f\n",k+1,ubicacionRobot.coordenadaPataSistemaPata_x[k],ubicacionRobot.coordenadaPataSistemaPata_y[k],ubicacionRobot.coordenadaPataSistemaPata_z[k]);

            } else {
                ROS_ERROR("Nodo6_3: servicio de posicion pata no funciona\n");
                ROS_ERROR("Nodo6_3: respuesta = %d\n",srv_simRosGetObjectPose.response.result);
            }
        }
        chatter_pub.publish(ubicacionRobot);
	}
//	fclose(fp);
	ROS_INFO("Adios6_3!");
	ros::shutdown();
	return(0);
}
