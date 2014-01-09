#include "ros/ros.h"
#include "std_msgs/String.h"
#include "math.h"
#include "../msg_gen/cpp/include/camina/CoordenadasObstaculos.h"
#include "vrep_common/VrepInfo.h"
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include "camina/v_repConst.h"
// Used API services:
#include "vrep_common/simRosGetIntegerSignal.h"
#include "vrep_common/simRosSetIntegerSignal.h"
#include "vrep_common/simRosEnableSubscriber.h"

// Global variables (modified by topic subscribers):
bool simulationRunning=true;
bool sensorTrigger=false;
float simulationTime=0.0f;

camina::CoordenadasObstaculos infoObstaculo;

ros::ServiceClient client_GetIntegerSignal;
vrep_common::simRosGetIntegerSignal srv_GetIntegerSignal;

ros::ServiceClient client_SetIntegerSignal;
vrep_common::simRosSetIntegerSignal srv_SetIntegerSignal;

// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}

int main(int argc, char **argv)
{
    int nObstaculos=5;
    int prueba[5]={1,2,3,4,5};
	char* str;
    // (when V-REP launches this executable, V-REP will also provide the argument list)
	//numero de argumentos que mande (se excluye el fantasma que el manda solo)
	//int nObstaculo=0, coordenada_x=0, coordenada_y=0;
	if (argc>=1)
	{
		str=argv[1]; //N obstaculos
    }
	else
	{
		printf("Indique argumentos!\n");
		sleep(5000);
		return 0;
	}

	// Create a ROS node. The name has a random component:
	int _argc = 0;
	char** _argv = NULL;
	struct timeval tv;
	unsigned int timeVal=0;
	if (gettimeofday(&tv,NULL)==0)
		timeVal=(tv.tv_sec*1000+tv.tv_usec/1000)&0x00ffffff;
	std::string nodeName("iObstaculos");
	std::string randId(boost::lexical_cast<std::string>(timeVal+int(999999.0f*(rand()/(float)RAND_MAX))));
	nodeName+=randId;
	ros::init(_argc,_argv,nodeName.c_str());

  //ros::init(argc, argv, "Obstaculos");
  ros::NodeHandle n;
  //ros::Publisher chatter_pub = n.advertise<camina::DatosTemporales>("tempTrayectoria", 100);
  ros::Subscriber subInfo=n.subscribe("/vrep/info",1,infoCallback);

    client_SetIntegerSignal=n.serviceClient<vrep_common::simRosSetIntegerSignal>("/vrep/simRosSetIntegerSignal");
    srv_SetIntegerSignal.request.signalName=str;
    srv_SetIntegerSignal.request.signalValue=nObstaculos;
//    if (client_GetIntegerSignal.call(srv_GetIntegerSignal)&&(srv_GetIntegerSignal.response.result!=-1)&&(srv_GetIntegerSignal.response.result!=0))
if (client_SetIntegerSignal.call(srv_SetIntegerSignal)&&(srv_SetIntegerSignal.response.result!=-1))
        {
               //printf("servicio SetIntegerSignal funciona\n");
        } else{
               printf("Hubo problema o señal no existe\n");
        }

    ROS_INFO("Adios!");
    ros::shutdown();
  return 0;

}

//while (ros::ok() && simulationRunning)
