#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <ros/ros.h>
#include "camina/v_repConst.h"
#include "../msg_gen/cpp/include/camina/AngulosMotor.h"
// Used data structures:
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include "vrep_common/JointSetStateData.h"
#include "vrep_common/VrepInfo.h"
// Used API services:
#include "vrep_common/simRosSetJointState.h"
#include "vrep_common/simRosGetJointState.h"
#include "vrep_common/simRosEnableSubscriber.h"
// Definiciones
#define pi 3.14159
// Global variables:
bool simulationRunning=true;
bool sensorTrigger=false;
float simulationTime=0.0f;
sensor_msgs::JointState patasMotores;
float q[3] = {0.0,0.0,0.0};	//Angulos de motores q1, q2, q3

ros::ServiceClient client_SetJointStates;
vrep_common::simRosSetJointState srv_SetJointStates; //Servicio para setear los joints de vrep

ros::ServiceClient client_GetJointStates;
vrep_common::simRosGetJointState srv_GetJointStates; //Servicio para obtener los joints de vrep

// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}

void motoresCallback(const camina::AngulosMotor qMotor)
{
    q[0]=qMotor.q1-pi/2;
    q[1]=qMotor.q2;
    q[2]=qMotor.q3-pi/2;

    srv_SetJointStates.request.values[qMotor.Npata*3-3+0] = q[0];
    srv_SetJointStates.request.values[qMotor.Npata*3-3+1] = q[1];
    srv_SetJointStates.request.values[qMotor.Npata*3-3+2] = q[2];
    //ROS_INFO("4-Angulos q1q2q3:[Npata=%d, ite=%d, q1= %.3f, q2= %.3f, q3= %.3f]", qMotor.Npata, qMotor.iteracion, q[0]*180.0/pi,q[1]*180.0/pi,q[2]*180.0/pi);

    if(qMotor.Npata==6)
    //Al recibir pata # manda todos los mensajes
    {
        if (client_SetJointStates.call(srv_SetJointStates)&&(srv_SetJointStates.response.result!=-1))
        {
               //printf("motoresCallback funciona\n");
        } else{
               printf("motoresCallback NOOO funciona\n");
        }

//        if (client_GetJointStates.call(srv_GetJointStates)&&(srv_GetJointStates.response.result!=-1))
//        {
//                patasMotores = srv_GetJointStates.response.state;
//                ROS_INFO("Motor12: %.3f, Motor2: %.3f, Motor3: %.3f", patasMotores.position[0]*180.0/pi,patasMotores.position[1]*180.0/pi,patasMotores.position[2]*180.0/pi);
//        }
    }
}
// Main code:
int main(int argc,char* argv[])
{
	// (when V-REP launches this executable, V-REP will also provide the argument list)
	//numero de argumentos que mande (se excluye el fantasma que el manda solo)
	int Narg=0;
	Narg=18;

	if (argc>Narg)
	{
        //printf("argumento 1: %d\n",atoi(argv[1]));
        //printf("argumento 2: %d\n",atoi(argv[2]));
        //printf("argumento 3: %d\n",atoi(argv[3]));
	}
	else
	{
		printf("Indique argumentos!\n");
		return (0);
	}
	// Create a ROS node:
	int _argc = 0;
	char** _argv = NULL;
	ros::init(_argc,_argv,"Nodo3_RosVrep");

	if(!ros::master::check())
		return(0);

	ros::NodeHandle node;
	ROS_INFO("Nodo3_RosVrep just started\n");

    //Subscripcion a nodo que publica posicion deseada de motores
    ros::Subscriber subMq=node.subscribe("DatosDeMotores", 100, motoresCallback);
    ros::Subscriber subInfo=node.subscribe("/vrep/info",1,infoCallback);
    client_SetJointStates=node.serviceClient<vrep_common::simRosSetJointState>("/vrep/simRosSetJointState");
    client_GetJointStates=node.serviceClient<vrep_common::simRosGetJointState>("/vrep/simRosGetJointState");

    srv_GetJointStates.request.handle=sim_handle_all;
    srv_SetJointStates.request.handles.resize(Narg,0);
    srv_SetJointStates.request.setModes.resize(Narg,0);
    srv_SetJointStates.request.values.resize(Narg,0);
    //Damos nombres de handles al servicio
    for (int i=0; i < Narg; i++)
    {
        srv_SetJointStates.request.handles[i]=atoi(argv[i+1]);
        srv_SetJointStates.request.setModes[i]=1;
    }
	while (ros::ok() && simulationRunning)
	{
		ros::spinOnce();
		//printf("Hola de Arana_RosVrep\n");
	}
	ROS_INFO("Adios3!");
	ros::shutdown();
	return(0);
}

