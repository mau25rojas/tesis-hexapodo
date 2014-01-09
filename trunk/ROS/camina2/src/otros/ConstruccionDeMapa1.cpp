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
int matrizMapa[10][10];
int aux_x1, aux_y1,aux_x2, aux_y2;
int cuenta=0;

// Funciones
void print_matrizMapa(int nFilas, int nColumnas);

// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}

void MapaCallback1(const camina::InfoMapa msgInfoMapa)
{
	cuenta++;
	aux_y1=msgInfoMapa.coordenadasCelda_y;
	aux_x1=msgInfoMapa.coordenadasCelda_x;
	matrizMapa[aux_y1][aux_x1]=msgInfoMapa.valorCelda;
	//ROS_INFO("\t Coor_y: %d\t Coor_x: %d\t Celda: %d", msgInfoMapa.coordenadasCelda_y,msgInfoMapa.coordenadasCelda_x, msgInfoMapa.valorCelda);
	infoMapa.finalCeldas = msgInfoMapa.finalCeldas;
}

void MapaCallback2(const camina::InfoMapa msgInfoMapa)
{
	cuenta++;
	aux_y2=msgInfoMapa.coordenadasCelda_y;
	aux_x2=msgInfoMapa.coordenadasCelda_x;
	matrizMapa[aux_y2][aux_x2]=msgInfoMapa.valorCelda;
	//ROS_INFO("\t Coor_y: %d\t Coor_x: %d\t Celda: %d", msgInfoMapa.coordenadasCelda_y,msgInfoMapa.coordenadasCelda_x, msgInfoMapa.valorCelda);
	infoMapa.finalCeldas = msgInfoMapa.finalCeldas;
}

int main(int argc, char **argv)
{
	int _argc = 0;
	char** _argv = NULL;

	ros::init(_argc,_argv,"ConstruccionDeMapa");
	ROS_INFO("Hola ConstruccionDeMapa!");

    ros::NodeHandle n;
    ros::Subscriber sub1=n.subscribe("InfoDeMapa1",1000,MapaCallback1);
    ros::Subscriber sub2=n.subscribe("InfoDeMapa2",1000,MapaCallback2);
    ros::Subscriber subInfo=n.subscribe("/vrep/info",1000,infoCallback);

    infoMapa.nCeldas_y = atoi(argv[1]);
    infoMapa.nCeldas_x = atoi(argv[2]);

    for(int i=0;i<infoMapa.nCeldas_y;i++){
        for(int j=0;j<infoMapa.nCeldas_x;j++){
            matrizMapa[i][j]=-1;
        }
    }

    while (ros::ok() && simulationRunning){
        ros::spinOnce();
        if (infoMapa.finalCeldas==1){
            print_matrizMapa(infoMapa.nCeldas_y,infoMapa.nCeldas_x);
            ROS_INFO("numero de mensajes: %d\n",cuenta);
            ROS_INFO("Adios21!");
            ros::shutdown();
            return 0;
        }
    }
    ROS_INFO("Adios22!");
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
