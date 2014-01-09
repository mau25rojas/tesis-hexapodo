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
    int Narg=0, k=0;
	int PataTipHandle[Npatas],FuerzaSensorHandle[Npatas];
	camina::UbicacionRobot ubicacionRobot;
    geometry_msgs::PoseStamped PataTipPose[Npatas];

	Narg=12;

	if (argc>=Narg)
	{
        for (k=0;k<Npatas;k++) PataTipHandle[k] = atoi(argv[1+k]);
        for (k=0;k<Npatas;k++) FuerzaSensorHandle[k] = atoi(argv[1+Npatas+k]);
	}
	else
	{
		ROS_ERROR("Nodo6_2:Indique argumentos completos!\n");
		return (0);
	}
//-- Inicializacion de variables del mensaje
    for (k=0;k<Npatas;k++) {
        ubicacionRobot.coordenadaPata_y.push_back(0);
        ubicacionRobot.coordenadaPata_x.push_back(0);
        ubicacionRobot.coordenadaPata_z.push_back(0);
        ubicacionRobot.pataTipFuerza_z.push_back(0);
        ubicacionRobot.pataApoyo.push_back(0);
    }
	// Create a ROS node:
	int _argc = 0;
	char** _argv = NULL;
	ros::init(_argc,_argv,"Nodo6_Ubicacion2Patas");
	if(!ros::master::check()) return(0);

	ros::NodeHandle node;
	ROS_INFO("Nodo6_Ubicacion2Patas just started\n");

//-- Topicos susbcritos y publicados
    ros::Subscriber subInfo=node.subscribe("/vrep/info",1,infoCallback);
    ros::Publisher chatter_pub = node.advertise<camina::UbicacionRobot>("Ubicacion2Patas", 100);
//-- Clientes y Servicios
    client_simRosGetObjectPose=node.serviceClient<vrep_common::simRosGetObjectPose>("/vrep/simRosGetObjectPose");
    client_simRosReadForceSensor=node.serviceClient<vrep_common::simRosReadForceSensor>("/vrep/simRosReadForceSensor");
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
//		printf("Nodo6_2\n");
		ros::spinOnce();
//		loop_rate.sleep();
        for (k=0;k<Npatas;k++){
            srv_simRosGetObjectPose.request.handle=PataTipHandle[k];
        //--------------------------------------------------------------
        //---- Obtengo posicion de tip de pata en el mundo
            srv_simRosGetObjectPose.request.relativeToObjectHandle=-1;
            if (client_simRosGetObjectPose.call(srv_simRosGetObjectPose)&&(srv_simRosGetObjectPose.response.result!=-1))
            {
                PataTipPose[k] = srv_simRosGetObjectPose.response.pose;
                ubicacionRobot.coordenadaPata_x[k] = PataTipPose[k].pose.position.x;
                ubicacionRobot.coordenadaPata_y[k] = PataTipPose[k].pose.position.y;
                ubicacionRobot.coordenadaPata_z[k] = PataTipPose[k].pose.position.z;
//                ROS_INFO("Nodo6: ubicacion[%d]: x0= %.3f, y0= %.3f, z=%.3f",k+1,ubicacionRobot.coordenadaPata_x[k],ubicacionRobot.coordenadaPata_y[k],ubicacionRobot.coordenadaPata_z[k]);

//                PisadaProxima_y=ubicacionRobot.coordenadaPata_y[k] + (lambda_maximo+velocidad_Apoyo*(1-beta)*T)*cos((ubicacionRobot.orientacionCuerpo_yaw-teta_Offset)+alfa);
//                PisadaProxima_x=ubicacionRobot.coordenadaPata_x[k] + (lambda_maximo+velocidad_Apoyo*(1-beta)*T)*sin((ubicacionRobot.orientacionCuerpo_yaw-teta_Offset)+alfa);
//                ROS_INFO("Nodo6: ubicacion[%d]: x1= %.3f, y1= %.3f",k+1,PisadaProxima_x,PisadaProxima_y);

//                srv_simRosReadForceSensor.request.handle=FuerzaSensorHandle[k];
//                if (client_simRosReadForceSensor.call(srv_simRosReadForceSensor)&&(srv_simRosReadForceSensor.response.result!=-1))
//                {
//                    //PataTipFuerza=srv_simRosReadForceSensor.response.force;
//                    ubicacionRobot.pataTipFuerza_z[k]=srv_simRosReadForceSensor.response.force.z;
                if (ubicacionRobot.coordenadaPata_z[k]<=umbral_Z_Apoyo) {
//                    if (ubicacionRobot.pataTipFuerza_z[k]>=umbralFuerzaApoyo) {
                    //-- La pata esta en apoyo
                        ubicacionRobot.pataApoyo[k]=1;
//                        printf("P[%d] apoyo\n",k+1);
                    } else {
                    //-- La pata esta en transferencia
                        ubicacionRobot.pataApoyo[k]=0;
                    }
//                    fprintf(fp,"%d\t",ubicacionRobot.pataApoyo[k]);

//                }  else {
//                    ROS_ERROR("Nodo6_2: servicio de sensor de fuerza no funciona\n");
//                    ROS_ERROR("Nodo6_2: respuesta = %d\n",srv_simRosReadForceSensor.response.result);
//                }
//                printf("Nodo6: Pata apoyo[%d]=%d\n",k,ubicacionRobot.pataApoyo[k]);
            } else {
                ROS_ERROR("Nodo6_2: servicio 1 de posicion pata no funciona\n");
                ROS_ERROR("Nodo6_2: respuesta = %d\n",srv_simRosGetObjectPose.response.result);
            }
        }
       chatter_pub.publish(ubicacionRobot);
	}
//	fclose(fp);
	ROS_INFO("Adios6_2!");
	ros::shutdown();
	return(0);
}
