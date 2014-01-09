#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
//Librerias propias usadas
#include "constantes.hpp"
#include "../util/comm/comm.hpp"
#include "camina/v_repConst.h"
// Used data structures:
#include <camina/AnguloParaVrep.h>
#include <camina/commData.h>
#include <sensor_msgs/JointState.h>
// Used API services:
#include "vrep_common/VrepInfo.h"
#include "vrep_common/simRosGetJointState.h"

// Global variables:
bool simulationRunning=true;
bool sensorTrigger=false;
float simulationTime=0.0f;
//Clientes y Servicios
ros::ServiceClient client_GetJointStates;
vrep_common::simRosGetJointState srv_GetJointStates; //Servicio para obtener los joints de vrep

// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}

int main(int argc, char **argv) //Inicia el Manejo de cada uno de los nodos
{
    int Narg=0, MotoresHandles[Nmotores], k=0;
	float periodo=0.0, f=0.0;
    camina::AnguloParaVrep pata[Npatas];
    sensor_msgs::JointState Motores[Nmotores];
	Narg=Nmotores;

	if (argc>=Narg)
	{
        for (k=0;k<Nmotores;k++) MotoresHandles[k] = atoi(argv[1+k]);
	}
	else
	{
		ROS_ERROR("vrep_to_arana: Indique argumentos!\n");
		return (0);
	}

	ros::init(argc, argv, "vrep_to_arana");
	ros::NodeHandle n;
	//Subscripciones y publicaciones
    ros::Subscriber subInfo1=n.subscribe("/vrep/info",1,infoCallback);
	ros::Publisher q_calculados = n.advertise<camina::AnguloParaVrep>("arana/leg_position", 50);
    // Clientes y servicios
    client_GetJointStates=n.serviceClient<vrep_common::simRosGetJointState>("/vrep/simRosGetJointState");
    srv_GetJointStates.request.handle=sim_handle_all;

    periodo=0.05;
    f=1/periodo;
    ros::Rate loop_rate(f);  //Frecuencia [Hz]
//	ros::Rate loop_rate(20);

	while (ros::ok() && simulationRunning){
  	    for(k=0;k<Nmotores;k++){
            srv_GetJointStates.request.handle=MotoresHandles[k];
            if (client_GetJointStates.call(srv_GetJointStates)&&(srv_GetJointStates.response.result!=-1))
            {
                Motores[k] = srv_GetJointStates.response.state;
    //                ROS_INFO("Motor1: %.3f, Motor2: %.3f, Motor3: %.3f", Motores.position[0]*180.0/pi,Motores.position[1]*180.0/pi,Motores.position[2]*180.0/pi);
            } else{
                   ROS_ERROR("Nodo vrep_to_arana: servicio getjoints no funciona\n");
            }
        }

//		if (client_GetJointStatesPublisher.call(srv_GetJointStatesPublisher)&&(srv_GetJointStatesPublisher.response.result!=-1))
//    		{
//      		Motores = srv_GetJointStatesPublisher.response.state;
//			//Pata 1
//			pata[0].Npata = 0;
//			pata[0].Q1 = Motores.position[0]*180/pi;
//			pata[0].Q2 = Motores.position[1]*180/pi;
//			pata[0].Q3 = Motores.position[2]*180/pi;
//			//Pata 2
//			pata[1].Npata = 1;
//			pata[1].Q1 = Motores.position[3]*180/pi;
//			pata[1].Q2 = Motores.position[4]*180/pi;
//			pata[1].Q3 = Motores.position[5]*180/pi;
//			//Pata 3
//			pata[2].Npata = 2;
//			pata[2].Q1 = Motores.position[6]*180/pi;
//			pata[2].Q2 = Motores.position[7]*180/pi;
//			pata[2].Q3 = Motores.position[8]*180/pi;
//			//Pata 4
//			pata[3].Npata = 3;
//			pata[3].Q1 = Motores.position[9]*180/pi;
//			pata[3].Q2 = Motores.position[10]*180/pi;
//			pata[3].Q3 = Motores.position[11]*180/pi;
//			//Pata 5
//			pata[4].Npata = 4;
//			pata[4].Q1 = Motores.position[12]*180/pi;
//			pata[4].Q2 = Motores.position[13]*180/pi;
//			pata[4].Q3 = Motores.position[14]*180/pi;
//			//Pata 6
//			pata[5].Npata = 5;
//			pata[5].Q1 = Motores.position[15]*180/pi;
//			pata[5].Q2 = Motores.position[16]*180/pi;
//			pata[5].Q3 = Motores.position[17]*180/pi;
        for(k=0;k<Npatas;k++){
                pata[k].Npata = k;
                pata[k].Q1 = Motores[0+k*NmotorP].position[0]*180/pi;
                pata[k].Q2 = Motores[1+k*NmotorP].position[0]*180/pi;
                pata[k].Q3 = Motores[2+k*NmotorP].position[0]*180/pi;
        }
        q_calculados.publish(pata[0]);
        q_calculados.publish(pata[1]);
        q_calculados.publish(pata[2]);
        q_calculados.publish(pata[3]);
        q_calculados.publish(pata[4]);
        q_calculados.publish(pata[5]);
		loop_rate.sleep();
	}
    ROS_INFO("Adios vrep_to_arana!");
	ros::shutdown();
	return(0);
}

