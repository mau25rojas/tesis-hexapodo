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

camina::InfoMapa infoMapa;

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

    infoMapa.coordenadasCelda_y1 = atoi(argv[1])-1;     // Reajuste de indice
    infoMapa.coordenadasCelda_x1 = atoi(argv[2])-1;     // Reajuste de indice
    infoMapa.coordenadasCelda_y2 = atoi(argv[3])-1;     // Reajuste de indice
    infoMapa.coordenadasCelda_x2 = atoi(argv[4])-1;     // Reajuste de indice
    infoMapa.coordenadasCelda_y3 = atoi(argv[5])-1;     // Reajuste de indice
    infoMapa.coordenadasCelda_x3 = atoi(argv[6])-1;     // Reajuste de indice
    infoMapa.coordenadasCelda_y4 = atoi(argv[7])-1;     // Reajuste de indice
    infoMapa.coordenadasCelda_x4 = atoi(argv[8])-1;     // Reajuste de indice
    infoMapa.valorCelda1 = atoi(argv[9]);
    infoMapa.valorCelda2 = atoi(argv[10]);
    infoMapa.valorCelda3 = atoi(argv[11]);
    infoMapa.valorCelda4 = atoi(argv[12]);
    infoMapa.finalCeldas = atoi(argv[13]);

	usleep(1000);

	// Create a ROS node. The name has a random component:
	int _argc = 0;
	char** _argv = NULL;
	std::string nodeName("Nodo4_CeldaDeMapa");
	std::string Idy(boost::lexical_cast<std::string>(atoi(argv[1])));
	std::string Idx(boost::lexical_cast<std::string>(atoi(argv[2])));
	Idy+=Idx;
	nodeName+=Idy;
	ros::init(_argc,_argv,nodeName.c_str());
	//ROS_INFO("Nodo4_CeldaDeMapa just started\n");

    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<camina::InfoMapa>("InfoDeMapa", 1000);
    ros::Subscriber subInfo=n.subscribe("/vrep/info",1000,infoCallback);
    //ros::Rate wait_rate(atoi(argv[1])+atoi(argv[2]));
    ros::Rate wait_rate(1);

    wait_rate.sleep();      //MUY NECESARIO... para dar tiempo a mensajes
    chatter_pub.publish(infoMapa);

    //ROS_INFO("Adios4!");
    ros::shutdown();
    return 0;

}

//while (ros::ok() && simulationRunning)
