#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <hexacamina/AngulosParaRobot.h>
#include <hexacamina/commData.h>
#include "constantes.hpp"
#include "../util/comm/comm.hpp"
#include <sensor_msgs/JointState.h>

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

hexacamina::AngulosParaRobot pata;

int main(int argc, char **argv) //Inicia el Manejo de cada uno de los nodos
{
	float q_archivo[18];

	ros::init(argc, argv, "vrep_to_arana");
	ros::NodeHandle n;
    //Subscripciones y publicaciones
	ros::Publisher q_calculados = n.advertise<hexacamina::AngulosParaRobot>("arana/leg_position", 50);

	for(int i=0; i<Npatas; i++){
        pata.Q1.push_back(0);
        pata.Q2.push_back(0);
        pata.Q3.push_back(0);
	}

	ros::Rate loop_rate(90);
    std::fstream myfile("/home/maureen/fuerte_workspace/sandbox/TesisMaureen/ROS/hexacamina/pQ4.txt", std::ios_base::in);

    float aux;

    for(int i=0;i<800;i++){
        myfile >> q_archivo[0] >> q_archivo[1] >> q_archivo[2] >> q_archivo[3] >> q_archivo[4] >> q_archivo[5] >> q_archivo[6] >> q_archivo[7] >> q_archivo[8] >> q_archivo[9] >> q_archivo[10] >> q_archivo[11] >> q_archivo[12] >> q_archivo[13] >> q_archivo[14] >> q_archivo[15] >> q_archivo[16] >> q_archivo[17];
        for(int k=0;k<6;k++){
            pata.Q1[k]=q_archivo[k*3+0];
            pata.Q2[k]=q_archivo[k*3+1];
            pata.Q3[k]=q_archivo[k*3+2];
        }

        //-- En robot real los motores estan al reves respecto a la simulacion
        pata.Q2[1]=-pata.Q2[1];
        pata.Q2[3]=-pata.Q2[3];
        pata.Q2[5]=-pata.Q2[5];
//            ROS_INFO("pata1:%.3f, pata2:%.3f", pata.Q3[0], pata.Q3[1]);

        for(int i=0; i<6; i++){
//            ROS_INFO("envio[%d]",i);
            loop_rate.sleep();
            pata.Npata = i;
            q_calculados.publish(pata);
        }
    }
        ROS_INFO("angulos_arana!");
        ros::shutdown();
        return(0);
}
