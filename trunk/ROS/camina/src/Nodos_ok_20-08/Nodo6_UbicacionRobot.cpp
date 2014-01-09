#include <stdio.h>
#include <string.h>
#include <iostream>
#include <stdlib.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
//Librerias propias usadas
#include "constantes.hpp"
#include "vector3d.hpp"
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
	int Narg=0, handleArana=0, k=0;
	int PataTipHandle[Npatas],FuerzaSensorHandle[Npatas],Pata_Motor1Handle[Npatas];
	float periodo, f;
	camina::UbicacionRobot ubicacionRobot;
    geometry_msgs::PoseStamped CuerpoPose;
    geometry_msgs::PoseStamped PataTipPose[Npatas];
    tf::Quaternion CuerpoOrientacion_Q;
    tfScalar roll, pitch, yaw;

	Narg=13;

	if (argc>=Narg)
	{
        handleArana=atoi(argv[1]);
        for (k=0;k<Npatas;k++) PataTipHandle[k] = atoi(argv[2+k]);
        for (k=0;k<Npatas;k++) FuerzaSensorHandle[k] = atoi(argv[2+Npatas+k]);
        for (k=0;k<Npatas;k++) Pata_Motor1Handle[k] = atoi(argv[2+2*Npatas+k]);
	}
	else
	{
		ROS_ERROR("Nodo6:Indique argumentos completos!\n");
		return (0);
	}
    // Inicializacion de variables del mensaje
    for (k=0;k<Npatas;k++) {
        ubicacionRobot.coordenadaPata_y.push_back(0);
        ubicacionRobot.coordenadaPata_x.push_back(0);
        ubicacionRobot.coordenadaPata_z.push_back(0);
        ubicacionRobot.coordenadaPataSistemaPata_y.push_back(0);
        ubicacionRobot.coordenadaPataSistemaPata_x.push_back(0);
        ubicacionRobot.coordenadaPataSistemaPata_z.push_back(0);
        ubicacionRobot.pataTipFuerza_z.push_back(0);
        ubicacionRobot.pataApoyo.push_back(0);
    }
	// Create a ROS node:
	int _argc = 0;
	char** _argv = NULL;
	ros::init(_argc,_argv,"Nodo6_UbicacionArana");
	if(!ros::master::check()) return(0);

	ros::NodeHandle node;
	ROS_INFO("Nodo6_UbicacionArana just started\n");

    periodo=0.1;
    f=1/periodo;
    ros::Rate loop_rate(f);  //Frecuencia [Hz]

    //Topicos susbcritos y publicados
    ros::Subscriber subInfo=node.subscribe("/vrep/info",1,infoCallback);
    ros::Publisher chatter_pub = node.advertise<camina::UbicacionRobot>("UbicacionRobot", 100);
    //Clientes y Servicios
    client_simRosGetObjectPose=node.serviceClient<vrep_common::simRosGetObjectPose>("/vrep/simRosGetObjectPose");
    client_simRosReadForceSensor=node.serviceClient<vrep_common::simRosReadForceSensor>("/vrep/simRosReadForceSensor");
    srv_simRosGetObjectPose.request.relativeToObjectHandle=-1;

    // Creamos archivo para guardar posiciones de patas
//    fp = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina/datos/XYPatas.txt","w+");
//    int cuentaPataApoyo=0, pataApoyo[Npatas];
	while (ros::ok() && simulationRunning)
	{
		ros::spinOnce();
		loop_rate.sleep();
		// Primero buscamos posicion del cuerpo de Arana
		srv_simRosGetObjectPose.request.handle=handleArana;
		srv_simRosGetObjectPose.request.relativeToObjectHandle=-1;
	    if (client_simRosGetObjectPose.call(srv_simRosGetObjectPose)&&(srv_simRosGetObjectPose.response.result!=-1))
        {
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
        } else {
             ROS_ERROR("Nodo 6: servicio de posicion cuerpo no funciona\n");
            }

        for (k=0;k<Npatas;k++){
            ros::spinOnce();
//            loop_rate.sleep();
            srv_simRosGetObjectPose.request.handle=PataTipHandle[k];
        //--------------------------------------------------------------
        //---- Obtengo posicion de tip de pata en el mundo
            srv_simRosGetObjectPose.request.relativeToObjectHandle=-1;
            if (client_simRosGetObjectPose.call(srv_simRosGetObjectPose)&&(srv_simRosGetObjectPose.response.result!=-1))
            {
                PataTipPose[k] = srv_simRosGetObjectPose.response.pose;
                ubicacionRobot.coordenadaPata_y[k] = PataTipPose[k].pose.position.y;
                ubicacionRobot.coordenadaPata_x[k] = PataTipPose[k].pose.position.x;
                ubicacionRobot.coordenadaPata_z[k] = PataTipPose[k].pose.position.z;
//                printf("Nodo6: [%d]: y= %.3f, x= %.3f, z= %.3f\n",k+1,ubicacionRobot.coordenadaPata_y[k],ubicacionRobot.coordenadaPata_x[k],ubicacionRobot.coordenadaPata_z[k]);
                srv_simRosReadForceSensor.request.handle=FuerzaSensorHandle[k];
                if (client_simRosReadForceSensor.call(srv_simRosReadForceSensor)&&(srv_simRosReadForceSensor.response.result!=-1))
                {
                    //PataTipFuerza=srv_simRosReadForceSensor.response.force;
                    ubicacionRobot.pataTipFuerza_z[k]=srv_simRosReadForceSensor.response.force.z;
                    if (ubicacionRobot.pataTipFuerza_z[k]>=umbralFuerzaApoyo) {
                        //La pata esta en apoyo
                        ubicacionRobot.pataApoyo[k]=1;
                    } else {
                        //La pata esta en transferencia
                        ubicacionRobot.pataApoyo[k]=0;
                        }
                }  else {
                    ROS_ERROR("Nodo 6: servicio de sensor de fuerza no funciona\n");
                    ROS_ERROR("Nodo 6: respuesta = %d\n",srv_simRosReadForceSensor.response.result);
                }
//                printf("Nodo6: Pata apoyo[%d]=%d\n",k,ubicacionRobot.pataApoyo[k]);
            } else {
                ROS_ERROR("Nodo 6: servicio 1 de posicion pata no funciona\n");
                ROS_ERROR("Nodo 6: respuesta = %d\n",srv_simRosGetObjectPose.response.result);
            }
        }

        for (k=0;k<Npatas;k++){
            ros::spinOnce();
//            loop_rate.sleep();
            srv_simRosGetObjectPose.request.handle=PataTipHandle[k];
        //--------------------------------------------------------------
        //---- Obtengo posicion de tip de pata para cada sistema de pata
            srv_simRosGetObjectPose.request.relativeToObjectHandle=Pata_Motor1Handle[k];
            if (client_simRosGetObjectPose.call(srv_simRosGetObjectPose)&&(srv_simRosGetObjectPose.response.result!=-1)){

                PataTipPose[k] = srv_simRosGetObjectPose.response.pose;
//                printf("Pata apoyo[%d]=%d\n",k,ubicacionRobot.pataApoyo[k]);
                ubicacionRobot.coordenadaPataSistemaPata_y[k] = PataTipPose[k].pose.position.y;
                ubicacionRobot.coordenadaPataSistemaPata_x[k] = PataTipPose[k].pose.position.x;
                ubicacionRobot.coordenadaPataSistemaPata_z[k] = PataTipPose[k].pose.position.z;
//                printf("Pata: %d: x= %.3f, y= %.3f, z= %.3f\n",k+1,ubicacionRobot.coordenadaPataSistemaPata_x[k],ubicacionRobot.coordenadaPataSistemaPata_y[k],ubicacionRobot.coordenadaPataSistemaPata_z[k]);

            } else {
                ROS_ERROR("Nodo 6: servicio 2 de posicion pata no funciona\n");
                ROS_ERROR("Nodo 6: respuesta = %d\n",srv_simRosGetObjectPose.response.result);
            }
        }
//        ---------------------------------
//        fprintf(fp,"\n");
//        fprintf(fp,"%.3f\t%.3f\t",ubicacionRobot.coordenadaCuerpo_x,ubicacionRobot.coordenadaCuerpo_y);
//        fprintf(fp,"%d\t",cuentaPataApoyo);
//        for(k=0;k<Npatas;k++){
//            if(pataApoyo[k]==1){
//                fprintf(fp,"%.3f\t%.3f\t",ubicacionRobot.coordenadaPata_x[k],ubicacionRobot.coordenadaPata_y[k]);
//            }
//        }
//        fprintf(fp,"\n");
        chatter_pub.publish(ubicacionRobot);
	}
//	fclose(fp);
	ROS_INFO("Adios6!");
	ros::shutdown();
	return(0);
}
