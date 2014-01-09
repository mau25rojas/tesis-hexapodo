#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
//Librerias propias usadas
#include "constantes.hpp"
#include "../util/comm/comm.hpp"
#include "camina2/v_repConst.h"
// Used data structures:
#include <camina2/AnguloParaVrep.h>
#include <camina2/commData.h>
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
    camina2::AnguloParaVrep pata[Npatas];
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
	ros::Publisher q_calculados = n.advertise<camina2::AnguloParaVrep>("arana/leg_position", 50);
    // Clientes y servicios
    client_GetJointStates=n.serviceClient<vrep_common::simRosGetJointState>("/vrep/simRosGetJointState");
//    srv_GetJointStates.request.handle=sim_handle_all;

//    periodo=1;
//    f=1/periodo;
//    ros::Rate loop_rate(f);  //Frecuencia [Hz]
	ros::Rate loop_rate(100);

	while (ros::ok() && simulationRunning){

        loop_rate.sleep();
        ros::spinOnce();

  	    for(k=0;k<Nmotores;k++){
            srv_GetJointStates.request.handle=MotoresHandles[k];
            if (client_GetJointStates.call(srv_GetJointStates)&&(srv_GetJointStates.response.result!=-1))
            {
                Motores[k] = srv_GetJointStates.response.state;

            } else{
                   ROS_ERROR("Nodo vrep_to_arana: servicio getjoints no funciona\n");
            }
        }
        for(k=0;k<Npatas;k++){
            pata[k].Npata = k;
            pata[k].Q1 = Motores[0+k*3].position[0]*180/pi;
            pata[k].Q2 = Motores[1+k*3].position[0]*180/pi;
            pata[k].Q3 = Motores[2+k*3].position[0]*180/pi;
        }
//        ROS_INFO("Motor1: %.3f, Motor2: %.3f, Motor3: %.3f", Motores[0].position[0]*180.0/pi,Motores[1].position[0]*180.0/pi,Motores[2].position[0]*180.0/pi);
        q_calculados.publish(pata[0]);
//        q_calculados.publish(pata[1]);
//        q_calculados.publish(pata[2]);
//        q_calculados.publish(pata[3]);
//        q_calculados.publish(pata[4]);
//        q_calculados.publish(pata[5]);
	}
    ROS_INFO("Adios vrep_to_arana!");
	ros::shutdown();
	return(0);
}

