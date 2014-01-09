#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <camina/AnguloParaVrep.h>
#include <camina/commData.h>
#include "constantes.hpp"
#include "../util/comm/comm.hpp"
#include "camina/v_repConst.h"
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

//VREP
ros::ServiceClient client_GetJointStatesPublisher;
vrep_common::simRosGetJointState srv_GetJointStatesPublisher; //Servicio para obtener los joints de vrep
camina::AnguloParaVrep pierna[6];
sensor_msgs::JointState Motores;

int main(int argc, char **argv) //Inicia el Manejo de cada uno de los nodos
{
	ros::init(argc, argv, "vrep_to_arana");
	ros::NodeHandle n;
	q_calculados = n.advertise<camina::AnguloParaVrep>("arana/leg_position", 50);
    	client_GetJointStatesPublisher=n.serviceClient<vrep_common::simRosGetJointState>("/vrep/simRosGetJointState");
    	srv_GetJointStatesPublisher.request.handle=sim_handle_all;
	ros::Rate loop_rate(20);
	while (ros::ok())
  	{
		if (client_GetJointStatesPublisher.call(srv_GetJointStatesPublisher)&&(srv_GetJointStatesPublisher.response.result!=-1))
    		{
      		Motores = srv_GetJointStatesPublisher.response.state;
//			pierna[0].Npata = 0; //Pata 1
//			pierna[0].Q1 = Motores.position[2]*180/3.1416;
//			pierna[0].Q2 = Motores.position[6]*180/3.1416;
//			pierna[0].Q3 = 90+Motores.position[12]*-180/3.1416;
//			pierna[1].Npata = 1; //Pata 2
//			pierna[1].Q1 = Motores.position[1]*180/3.1416;
//			pierna[1].Q2 = Motores.position[11]*180/3.1416;
//			pierna[1].Q3 = 90+Motores.position[17]*-180/3.1416;
//			pierna[2].Npata = 2; //Pata 3
//			pierna[2].Q1 = Motores.position[3]*180/3.1416;
//			pierna[2].Q2 = Motores.position[7]*180/3.1416;
//			pierna[2].Q3 = 90+Motores.position[13]*-180/3.1416;
//			pierna[3].Npata = 3; //Pata 4
//			pierna[3].Q1 = Motores.position[0]*180/3.1416;
//			pierna[3].Q2 = Motores.position[10]*180/3.1416;
//			pierna[3].Q3 = 90+Motores.position[16]*-180/3.1416;
//			pierna[4].Npata = 4; //Pata 5
//			pierna[4].Q1 = Motores.position[4]*180/3.1416;
//			pierna[4].Q2 = Motores.position[8]*180/3.1416;
//			pierna[4].Q3 = 90+Motores.position[14]*-180/3.1416;
//			pierna[5].Npata = 5; //Pata 6
//			pierna[5].Q1 = Motores.position[5]*180/3.1416;
//			pierna[5].Q2 = Motores.position[9]*180/3.1416;
//			pierna[5].Q3 = 90+Motores.position[15]*-180/3.1416;
			pierna[0].Npata = 0; //Pata 1
			pierna[0].Q1 = Motores.position[2]*180/pi;
			pierna[0].Q2 = Motores.position[6]*180/pi;
			pierna[0].Q3 = Motores.position[12]*180/pi;
			pierna[1].Npata = 1; //Pata 2
			pierna[1].Q1 = Motores.position[1]*180/pi;
			pierna[1].Q2 = Motores.position[11]*180/pi;
			pierna[1].Q3 = Motores.position[17]*180/pi;
			pierna[2].Npata = 2; //Pata 3
			pierna[2].Q1 = Motores.position[3]*180/pi;
			pierna[2].Q2 = Motores.position[7]*180/pi;
			pierna[2].Q3 = Motores.position[13]*180/pi;
			pierna[3].Npata = 3; //Pata 4
			pierna[3].Q1 = Motores.position[0]*180/pi;
			pierna[3].Q2 = Motores.position[10]*180/pi;
			pierna[3].Q3 = Motores.position[16]*180/pi;
			pierna[4].Npata = 4; //Pata 5
			pierna[4].Q1 = Motores.position[4]*180/pi;
			pierna[4].Q2 = Motores.position[8]*180/pi;
			pierna[4].Q3 = Motores.position[14]*180/pi;
			pierna[5].Npata = 5; //Pata 6
			pierna[5].Q1 = Motores.position[5]*180/pi;
			pierna[5].Q2 = Motores.position[9]*180/pi;
			pierna[5].Q3 = Motores.position[15]*180/pi;

			q_calculados.publish(pierna[0]);
			q_calculados.publish(pierna[1]);
			q_calculados.publish(pierna[2]);
			q_calculados.publish(pierna[3]);
			q_calculados.publish(pierna[4]);
			q_calculados.publish(pierna[5]);

    		}
		loop_rate.sleep();
	}
	return 0;
}

