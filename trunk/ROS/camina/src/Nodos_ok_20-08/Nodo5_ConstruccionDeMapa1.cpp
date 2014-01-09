#include "ros/ros.h"
#include "std_msgs/String.h"
#include "math.h"
#include "../msg_gen/cpp/include/camina/InfoMapa.h"
#include "vrep_common/VrepInfo.h"
#include "camina/v_repConst.h"
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <geometry_msgs/PoseStamped.h>

// Used API services:
#include "vrep_common/simRosEnableSubscriber.h"
#include "vrep_common/simRosGetObjectHandle.h"
#include "vrep_common/simRosGetObjectPose.h"

// Global variables (modified by topic subscribers):
bool simulationRunning=true;
bool sensorTrigger=false;
float simulationTime=0.0f;
//camina::InfoMapa infoMapa;
int matrizMapa[100][100];
geometry_msgs::PoseStamped CeldaPose;

ros::ServiceClient client_simRosGetObjectHandle;
ros::ServiceClient client_simRosGetObjectPose;
vrep_common::simRosGetObjectHandle srv_simRosGetObjectHandle; //Servicio para obtener posicion del robot
vrep_common::simRosGetObjectPose srv_simRosGetObjectPose; //Servicio para obtener posicion del robot

// Funciones
void print_matrizMapa(int nFilas, int nColumnas);

// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}

int main(int argc, char **argv)
{
	int _argc = 0;
	char** _argv = NULL;
	int objectHandle=0;
	std::string _objectName;
	std::string objectName;
	int nCeldas_y=0, nCeldas_x=0, aux_x=0, aux_y=0;
	int contadorObstaculo = 0;
//    int cuenta=0;
    int finalCeldas=0;
    float Lx=0.0, Ly=0.0, divX=0.0, divY=0.0;

	Ly = atof(argv[1]);
    Lx = atof(argv[2]);
	nCeldas_y = atoi(argv[3]);
    nCeldas_x = atoi(argv[4]);
    _objectName = argv[5];
    contadorObstaculo = atoi(argv[6]);

    divX = Lx/nCeldas_x;
    divY = Ly/nCeldas_y;

    printf ("%s \n", _objectName.c_str());

	ros::init(_argc,_argv,"Nodo5_ConstruccionDeMapa");
	ROS_INFO("Nodo5_ConstruccionDeMapa just started\n");

    ros::NodeHandle node;
    ros::Subscriber subInfo=node.subscribe("/vrep/info",1000,infoCallback);

    client_simRosGetObjectHandle=node.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
    client_simRosGetObjectPose=node.serviceClient<vrep_common::simRosGetObjectPose>("/vrep/simRosGetObjectPose");

    srv_simRosGetObjectPose.request.relativeToObjectHandle=-1;

    ros::Rate loop_rate(100);  //Frecuencia [Hz]

    // Inicializamos la matriz con cero obstaculos
    for(int i=0;i<nCeldas_y;i++){
        for(int j=0;j<nCeldas_x;j++){
            matrizMapa[i][j]=0;
        }
    }

    objectName = _objectName;

    for(int i=0;i<contadorObstaculo;i++)
    {
            ros::spinOnce();
            loop_rate.sleep();
            if (ros::ok() && simulationRunning){
            srv_simRosGetObjectHandle.request.objectName=objectName.c_str();
            if (client_simRosGetObjectHandle.call(srv_simRosGetObjectHandle)&&(srv_simRosGetObjectHandle.response.handle!=-1))
            {
                objectHandle = srv_simRosGetObjectHandle.response.handle;

                srv_simRosGetObjectPose.request.handle=objectHandle;
                if (client_simRosGetObjectPose.call(srv_simRosGetObjectPose)&&(srv_simRosGetObjectPose.response.result!=-1))
                {
                    CeldaPose = srv_simRosGetObjectPose.response.pose;
//                    ROS_INFO("posicion: y=%.3f; x=%.3f; z=%.3f\n", CeldaPose.pose.position.y,CeldaPose.pose.position.x,CeldaPose.pose.position.z);

                    if (CeldaPose.pose.position.y>=0){
                        aux_y = (int) (nCeldas_y/2 - floor(CeldaPose.pose.position.y/divY));
                    }else{
                        aux_y = (int) (nCeldas_y/2 - ceil(CeldaPose.pose.position.y/divY) + 1);
                    }

                    if (CeldaPose.pose.position.x>=0){
                        aux_x = (int) (nCeldas_x/2 + floor(CeldaPose.pose.position.x/divX) + 1);
                    }else{
                        aux_x = (int) (nCeldas_x/2 + ceil(CeldaPose.pose.position.x/divX));
                    }
//                    ROS_INFO("i: %d\t j: %d\n",aux_y,aux_x);

//                    if (CeldaPose.pose.position.z<0.01)
//                    {   //El objeto NO es obstaculo
//                        matrizMapa[aux_y-1][aux_x-1]=0;
//                    } else{
                        //El objeto SI es obstaculo
                        matrizMapa[aux_y-1][aux_x-1]=1;
//                    }
//                    if (aux_x==nCeldas_x && aux_y==nCeldas_y) finalCeldas = 1;

                } else {
                        ROS_ERROR("Servicio de POSE no funciona");
                  }
                } else {
                    ROS_ERROR("Servicio de HANDLE no funciona");
                  }

                objectName = _objectName;
                std::string Id(boost::lexical_cast<std::string>(i));
                objectName+=Id;

                if (i==(contadorObstaculo-1)){
                    print_matrizMapa(nCeldas_y,nCeldas_x);
                    ROS_INFO("Adios51!");
                    ros::shutdown();
                    return 0;
                }
        } else {
            ROS_INFO("Adios52!");
            ros::shutdown();
            return 0;
        }
    }
    ROS_INFO("Adios53!");
    ros::shutdown();
    return 0;
}

void print_matrizMapa(int nFilas, int nColumnas)
{
     printf("\n");
     for(int i=0;i<nFilas;i++)
     {
         for(int j=0;j<nColumnas;j++)
         {
            printf(" %d",matrizMapa[i][j]);
         }
        printf("\n");
     }
}
