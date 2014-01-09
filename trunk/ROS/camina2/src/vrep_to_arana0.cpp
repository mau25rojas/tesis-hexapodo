#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <camina2/AnguloParaVrep.h>
#include <camina2/commData.h>
#include "constantes.hpp"
#include "../util/comm/comm.hpp"
#include "camina2/v_repConst.h"
#include <sensor_msgs/JointState.h>
#include "vrep_common/simRosGetJointState.h"

/*	Definicion de las variables globales a utilizar y de los nodos de comunicación	*/
#define LIM_SUP_Q1 2250
#define LIM_INF_Q1 770
#define LIM_SUP_Q2 1950
#define LIM_INF_Q2 1040
#define LIM_SUP_Q2_NEG 2060
#define LIM_INF_Q2_NEG 1150
#define LIM_SUP_Q3 2250
#define LIM_INF_Q3 1150
#define LIM_SUP_Q3_NEG 1950
#define LIM_INF_Q3_NEG 850
#define OFFSET_Q1 1550.0
#define OFFSET_Q2 1550.0
#define OFFSET_Q3 1550.0
ros::Publisher q_calculados;
// Used API services:
#include "vrep_common/VrepInfo.h"
#include "vrep_common/simRosGetJointState.h"
// Global variables:
bool simulationRunning=true;
bool sensorTrigger=false;
float simulationTime=0.0f;

//VREP
ros::ServiceClient client_GetJointStatesPublisher;
vrep_common::simRosGetJointState srv_GetJointStatesPublisher; //Servicio para obtener los joints de vrep
camina2::AnguloParaVrep pata[6];
sensor_msgs::JointState Motores;

// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}

int main(int argc, char **argv) //Inicia el Manejo de cada uno de los nodos
{
	ros::init(argc, argv, "vrep_to_arana");
	ros::NodeHandle n;
	ros::Subscriber subInfo1=n.subscribe("/vrep/info",1,infoCallback);
	q_calculados = n.advertise<camina2::AnguloParaVrep>("arana/leg_position", 50);
    client_GetJointStatesPublisher=n.serviceClient<vrep_common::simRosGetJointState>("/vrep/simRosGetJointState");
    srv_GetJointStatesPublisher.request.handle=sim_handle_all;
	ros::Rate loop_rate(20);
	while (ros::ok() && simulationRunning)
  	{
        loop_rate.sleep();
        ros::spinOnce();
		if (client_GetJointStatesPublisher.call(srv_GetJointStatesPublisher)&&(srv_GetJointStatesPublisher.response.result!=-1))
    		{
      		Motores = srv_GetJointStatesPublisher.response.state;
			pata[0].Npata = 0; //Pata 1
			pata[0].Q1 = Motores.position[2]*180/pi;
			pata[0].Q2 = Motores.position[6]*180/pi;
			pata[0].Q3 = Motores.position[12]*180/pi;
			pata[1].Npata = 1; //Pata 2
			pata[1].Q1 = Motores.position[1]*180/pi;
			pata[1].Q2 = Motores.position[11]*180/pi;
			pata[1].Q3 = Motores.position[17]*180/pi;
			pata[2].Npata = 2; //Pata 3
			pata[2].Q1 = Motores.position[3]*180/pi;
			pata[2].Q2 = Motores.position[7]*180/pi;
			pata[2].Q3 = Motores.position[13]*180/pi;
			pata[3].Npata = 3; //Pata 4
			pata[3].Q1 = Motores.position[0]*180/pi;
			pata[3].Q2 = Motores.position[10]*180/pi;
			pata[3].Q3 = Motores.position[16]*180/pi;
			pata[4].Npata = 4; //Pata 5
			pata[4].Q1 = Motores.position[4]*180/pi;
			pata[4].Q2 = Motores.position[8]*180/pi;
			pata[4].Q3 = Motores.position[14]*180/pi;
			pata[5].Npata = 5; //Pata 6
			pata[5].Q1 = Motores.position[5]*180/pi;
			pata[5].Q2 = Motores.position[9]*180/pi;
			pata[5].Q3 = Motores.position[15]*180/pi;

			q_calculados.publish(pata[0]);
//			q_calculados.publish(pata[1]);
//			q_calculados.publish(pata[2]);
//			q_calculados.publish(pata[3]);
//			q_calculados.publish(pata[4]);
//			q_calculados.publish(pata[5]);

    		}
	}
    ROS_INFO("Adios vrep_to_arana!");
	ros::shutdown();
	return(0);
}

