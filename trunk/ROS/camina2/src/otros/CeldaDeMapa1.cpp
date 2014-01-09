#include "ros/ros.h"
#include "std_msgs/String.h"
#include "math.h"
#include "../msg_gen/cpp/include/camina/InfoMapa.h"
#include "vrep_common/VrepInfo.h"
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include "camina/v_repConst.h"
// Used API services:
#include "vrep_common/simRosEnableSubscriber.h"

// Global variables (modified by topic subscribers):
bool simulationRunning=true;
bool sensorTrigger=false;
float simulationTime=0.0f;

camina::InfoMapa infoMapa1;
camina::InfoMapa infoMapa2;

// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}

int main(int argc, char **argv)
{
    // (when V-REP launches this executable, V-REP will also provide the argument list)
	//numero de argumentos que mande (se excluye el fantasma que el manda solo)

	if (argc>=1)
	{
		//str=atoi(argv[1]); //N obstaculos
    }
	else
	{
		printf("Indique argumentos!\n");
		sleep(5000);
		return 0;
	}

    infoMapa1.coordenadasCelda_y = atoi(argv[1])-1;     // Reajuste de indice
    infoMapa1.coordenadasCelda_x = atoi(argv[2])-1;     // Reajuste de indice
    infoMapa2.coordenadasCelda_y = atoi(argv[3])-1;     // Reajuste de indice
    infoMapa2.coordenadasCelda_x = atoi(argv[4])-1;     // Reajuste de indice
    infoMapa1.valorCelda = atoi(argv[5]);
    infoMapa2.finalCeldas = atoi(argv[6]);

	usleep(1000);

	// Create a ROS node. The name has a random component:
	int _argc = 0;
	char** _argv = NULL;
	std::string nodeName("CeldaDeMapa");
	std::string Idy(boost::lexical_cast<std::string>(atoi(argv[1])));
	std::string Idx(boost::lexical_cast<std::string>(atoi(argv[2])));
	Idy+=Idx;
	nodeName+=Idy;
	ros::init(_argc,_argv,nodeName.c_str());
	//ROS_INFO("Hola Celdas!");

    ros::NodeHandle n;
    ros::Publisher chatter_pub1 = n.advertise<camina::InfoMapa>("InfoDeMapa1", 1000);
    ros::Publisher chatter_pub2 = n.advertise<camina::InfoMapa>("InfoDeMapa2", 1000);
    ros::Subscriber subInfo=n.subscribe("/vrep/info",1000,infoCallback);
    //ros::Rate wait_rate(atoi(argv[1])+atoi(argv[2]));
    ros::Rate wait_rate(2);

    wait_rate.sleep();      //MUY NECESARIO... para dar tiempo a mensajes
    chatter_pub1.publish(infoMapa1);
    wait_rate.sleep();      //MUY NECESARIO... para dar tiempo a mensajes
    chatter_pub2.publish(infoMapa2);

    //ROS_INFO("Adios1!");
    ros::shutdown();
    return 0;

}

//while (ros::ok() && simulationRunning)
