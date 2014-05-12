#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <ros/ros.h>
//Librerias propias usadas
#include "constantes.hpp"
#include "pruebas/v_repConst.h"
// Used data structures:
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include "pruebas/AngulosMotor.h"
// Used API services:
#include "vrep_common/VrepInfo.h"
#include "vrep_common/JointSetStateData.h"
#include "vrep_common/simRosSetJointState.h"
#include "vrep_common/simRosEnableSubscriber.h"
// Global variables:
bool simulationRunning=true;
bool sensorTrigger=false;
float simulationTime=0.0f;
sensor_msgs::JointState patasMotores;
float q[3] = {0.0,0.0,0.0};	//Angulos de motores q1, q2, q3

ros::ServiceClient client_SetJointStates;
vrep_common::simRosSetJointState srv_SetJointStates; //Servicio para setear los joints de vrep

// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}

void motoresCallback(const pruebas::AngulosMotor qMotor)
{
    q[0]=qMotor.q1;
    q[1]=qMotor.q2;
    q[2]=qMotor.q3;

    srv_SetJointStates.request.values[qMotor.Npata*3-3+0] = q[0];
    srv_SetJointStates.request.values[qMotor.Npata*3-3+1] = q[1];
    srv_SetJointStates.request.values[qMotor.Npata*3-3+2] = q[2];
    //ROS_INFO("4-Angulos q1q2q3:[Npata=%d, ite=%d, q1= %.3f, q2= %.3f, q3= %.3f]", qMotor.Npata, qMotor.iteracion, q[0]*180.0/pi,q[1]*180.0/pi,q[2]*180.0/pi);

//    if(qMotor.Npata==6)
//    //Al recibir pata # manda todos los mensajes
//    {
        if (client_SetJointStates.call(srv_SetJointStates)&&(srv_SetJointStates.response.result!=-1))
        {
               //printf("motoresCallback funciona\n");
        } else{
               ROS_ERROR("Nodo 3: servicio setjoints no funciona\n");
        }
//    }
}
// Main code:
int main(int argc,char* argv[])
{
	// (when V-REP launches this executable, V-REP will also provide the argument list)
	//numero de argumentos que mande (se excluye el fantasma que el manda solo)
	int Narg=0;
	Narg=3;

	if (argc>Narg)
	{	}
	else
	{
		ROS_ERROR("Nodo3: Indique argumentos!\n");
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

    srv_SetJointStates.request.handles.resize(Narg,0);
    srv_SetJointStates.request.setModes.resize(Narg,0);
    srv_SetJointStates.request.values.resize(Narg,0);
    //Damos nombres de handles al servicio
    for (int i=0;i<Narg;i++)
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












//#include <stdio.h>
//#include <iostream>
//#include <stdlib.h>
//#include <ros/ros.h>
//#include <time.h>
//#include "boost/date_time/posix_time/posix_time.hpp"
////Librerias propias usadas
//#include "constantes.hpp"
//#include "pruebas/v_repConst.h"
//// Used data structures:
//#include <std_msgs/Float64.h>
//#include <sensor_msgs/JointState.h>
////#include "pruebas/AngulosMotor.h"
//// Used API services:
//#include "vrep_common/VrepInfo.h"
//#include "vrep_common/JointSetStateData.h"
//#include "vrep_common/simRosSetJointState.h"
//#include "vrep_common/simRosEnableSubscriber.h"
//// Global variables:
//bool simulationRunning=true;
//bool sensorTrigger=false;
//float simulationTime=0.0f;
//sensor_msgs::JointState patasMotores;
//float q[3] = {0.0,0.0,0.0};	//Angulos de motores q1, q2, q3
//float tiempo_ahora=0.0;
//boost::posix_time::ptime timer_1,timer_2, timer_3, timer_4;
//boost::posix_time::time_duration diff_t;
//ros::ServiceClient client_SetJointStates;
//vrep_common::simRosSetJointState srv_SetJointStates; //Servicio para setear los joints de vrep
//ros::Subscriber subInfo1;
//
//// Topic subscriber callbacks:
//void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
//{
//	simulationTime=info->simulationTime.data;
//	simulationRunning=(info->simulatorState.data&1)!=0;
//}
//
//void posicionCallback(sensor_msgs::JointState msgMotor)
//{
//    timer_4 = boost::posix_time::microsec_clock::local_time();
////    ROS_INFO("posicion motores: %.4f,%.4f,%.4f",msgMotor.position[0],msgMotor.position[1],msgMotor.position[2]);
//    diff_t = timer_2-timer_4;
//    tiempo_ahora = (float) fabs(diff_t.total_microseconds());
//    ROS_INFO("tiempo recibido:%.2f,pos:%.5f",tiempo_ahora,msgMotor.position[0]);
//}
//
//// Main code:
//int main(int argc,char* argv[])
//{
//	// (when V-REP launches this executable, V-REP will also provide the argument list)
//	//numero de argumentos que mande (se excluye el fantasma que el manda solo)
//	int Narg=0;
//	std::string joints_stream;
//	Narg=3;
//
//	if (argc>=Narg)
//	{	joints_stream = argv[4]; }
//	else
//	{
//		ROS_ERROR("Nodo3: Indique argumentos!\n");
//		return (0);
//	}
//	// Create a ROS node:
//	int _argc = 0;
//	char** _argv = NULL;
//	ros::init(_argc,_argv,"Nodo3_RosVrep");
//
//	if(!ros::master::check())
//		return(0);
//
//	std::string topic("/vrep/");
//	topic+=joints_stream;
//
//	ros::NodeHandle node;
//	ROS_INFO("Nodo3_RosVrep just started");
//    timer_1 = boost::posix_time::microsec_clock::local_time();
//
//    //Subscripcion a nodo que publica posicion deseada de motores
////    ros::Subscriber subMq=node.subscribe("DatosDeMotores", 100, motoresCallback);
//    ros::Subscriber subInfo=node.subscribe("/vrep/info",1,infoCallback);
//    client_SetJointStates=node.serviceClient<vrep_common::simRosSetJointState>("/vrep/simRosSetJointState");
//    subInfo1=node.subscribe(topic,100,posicionCallback);
//
//    srv_SetJointStates.request.handles.resize(3,0);
//    srv_SetJointStates.request.setModes.resize(3,0);
//    srv_SetJointStates.request.values.resize(3,0);
//    //Damos nombres de handles al servicio
//    for (int i=0;i<3;i++)
//    {
//        srv_SetJointStates.request.handles[i]=atoi(argv[i+1]);
//        srv_SetJointStates.request.setModes[i]=1;
//    }
//
//    srv_SetJointStates.request.values[0] = 0.0;
//    srv_SetJointStates.request.values[1] = 0.0;
//    srv_SetJointStates.request.values[2] = 0.0;
//    float Periodo, f;
//
//    Periodo = 0.1;
//    f=1/Periodo;
//    ros::Rate loop_rate(f);  //Frecuencia [Hz]
//
//    timer_2 = boost::posix_time::microsec_clock::local_time();
//	while (ros::ok() && simulationRunning)
//	{
//		loop_rate.sleep();
//		ros::spinOnce();
//		srv_SetJointStates.request.values[0] = srv_SetJointStates.request.values[0] + pi/180;
//		if (client_SetJointStates.call(srv_SetJointStates)&&(srv_SetJointStates.response.result!=-1)){
//            timer_3 = boost::posix_time::microsec_clock::local_time();
//        } else{
//           ROS_ERROR("Nodo 3: servicio setjoints no funciona\n");
//        }
//        diff_t = timer_2-timer_3;
//        tiempo_ahora = (float) fabs(diff_t.total_microseconds());
//        ROS_INFO("tiempo:%.2f,pos:%.5f",tiempo_ahora,srv_SetJointStates.request.values[0]);
//	}
//
//	ROS_INFO("Adios3!");
//	ros::shutdown();
//	return(0);
//}

