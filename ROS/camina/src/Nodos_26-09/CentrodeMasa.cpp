#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <ros/ros.h>
//Librerias propias usadas
#include "constantes.hpp"
#include "vector3d.hpp"
#include "camina/v_repConst.h"
// Used data structures:
#include "../msg_gen/cpp/include/camina/UbicacionRobot.h"
// Used API services:
#include "vrep_common/VrepInfo.h"
#include "vrep_common/simRosEnableSubscriber.h"
#include "vrep_common/simRosGetJointState.h"

// Global variables:
bool simulationRunning=true;
bool sensorTrigger=false;
float simulationTime=0.0f;
ros::Publisher chatter_pub;
sensor_msgs::JointState RobJoints[Nmotores];
int JointHandle[Nmotores];
float PosicionPata_x=0.0, PosicionPata_y=0.0, PosicionPata_x2=0.0, PosicionPata_y2=0.0, anguloPatas_rad=0.0;
float teta_CuerpoRobot=0.0, PosicionCuerpo_x=0.0, PosicionCuerpo_y=0.0;
int k=0;
punto3d CM1, CM2, CM3, CM_Pata[Npatas], CM_Hexapodo;
camina::UbicacionRobot cm_Robot;
FILE *fp;
//Clientes y Servicios
ros::ServiceClient client_GetJointStates;
vrep_common::simRosGetJointState srv_GetJointStates; //Servicio para obtener los joints de vrep
//Funciones
void CentroDeMasa_Eslabon(punto3d* CentroDeMasa, float q1, float q2, float q3, int articulacion);
void CentroDeMasa_Pata(punto3d* CentroDeMasa, punto3d cm_eslabon1, punto3d cm_eslabon2, punto3d cm_eslabon3, int Npata);
void CentroDeMasa_Hexapodo(punto3d* CentroDeMasa, punto3d cm_pata1, punto3d cm_pata2, punto3d cm_pata3, punto3d cm_pata4, punto3d cm_pata5, punto3d cm_pata6);


// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}

void ubicacionRobCallback(camina::UbicacionRobot msgUbicacionRobot)
{
    float q1=0.0, q2=0.0, q3=0.0;

    PosicionCuerpo_x = msgUbicacionRobot.coordenadaCuerpo_x;
    PosicionCuerpo_y = msgUbicacionRobot.coordenadaCuerpo_y;
    teta_CuerpoRobot = msgUbicacionRobot.orientacionCuerpo_yaw;

    for(k=0;k<18;k++){
        srv_GetJointStates.request.handle=JointHandle[k];
        if (client_GetJointStates.call(srv_GetJointStates)&&(srv_GetJointStates.response.result!=-1))
        {
            RobJoints[k] = srv_GetJointStates.response.state;
//                ROS_INFO("Motor1: %.3f, Motor2: %.3f, Motor3: %.3f", RobJoints.position[0]*180.0/pi,RobJoints.position[1]*180.0/pi,RobJoints.position[2]*180.0/pi);
        } else{
               ROS_ERROR("Nodo centrodemasa: servicio getjoints no funciona\n");
        }
    }
    for(k=0;k<Npatas;k++){
            q1 = RobJoints[0+k*Neslabones].position[0];
            q2 = RobJoints[1+k*Neslabones].position[0];
            q3 = RobJoints[2+k*Neslabones].position[0];
//                printf("k:%d\tq1:%.3f\tq2:%.3f\tq3:%.3f\n",k,q1,q2,q3);
            CentroDeMasa_Eslabon(&CM1,q1,q2,q3,1);
            CentroDeMasa_Eslabon(&CM2,q1,q2,q3,2);
            CentroDeMasa_Eslabon(&CM3,q1,q2,q3,3);
            CentroDeMasa_Pata(&(CM_Pata[k]),CM1,CM2,CM3,k);
    }
    CentroDeMasa_Hexapodo(&CM_Hexapodo,CM_Pata[0],CM_Pata[1],CM_Pata[2],CM_Pata[3],CM_Pata[4],CM_Pata[5]);
        // Tranformada de coordenadas para ver posicion en el mundo
        for(k=0;k<Npatas;k++){
            cm_Robot.centroMasaPata_x[k] = cos(teta_CuerpoRobot)*CM_Pata[k].x-sin(teta_CuerpoRobot)*CM_Pata[k].y+PosicionCuerpo_x;
            cm_Robot.centroMasaPata_y[k] = sin(teta_CuerpoRobot)*CM_Pata[k].x+cos(teta_CuerpoRobot)*CM_Pata[k].y+PosicionCuerpo_y;
        }
        cm_Robot.centroMasaCuerpo_x = cos(teta_CuerpoRobot)*CM_Hexapodo.x-sin(teta_CuerpoRobot)*CM_Hexapodo.y+PosicionCuerpo_x;
        cm_Robot.centroMasaCuerpo_y = sin(teta_CuerpoRobot)*CM_Hexapodo.x+cos(teta_CuerpoRobot)*CM_Hexapodo.y+PosicionCuerpo_y;
    printf("cm_robot: %.3f\t%.3f\n",cm_Robot.centroMasaCuerpo_x,cm_Robot.centroMasaCuerpo_y);
//        printf("\n");
//        for(k=0;k<Npatas;k++){
//            printf("pata %d: %.3f\t%.3f\n",k+1,cm_Robot.centroMasaPata_x[k],cm_Robot.centroMasaPata_y[k]);
//        }
//        printf("\n");
    chatter_pub.publish(cm_Robot);
        //---------------------------------
        // Creamos archivo para guardar posiciones de patas
//        fprintf(fp,"\n");
//        fprintf(fp,"%.3f\t%.3f\t",cm_Robot.centroMasaCuerpo_x,cm_Robot.centroMasaCuerpo_y);
//        for(k=0;k<Npatas;k++){
//            fprintf(fp,"%d\t",msgUbicacionRobot.pataApoyo[k]);
//        }
//        for(k=0;k<Npatas;k++){
//            fprintf(fp,"%.3f\t%.3f\t",msgUbicacionRobot.coordenadaPata_x[k],msgUbicacionRobot.coordenadaPata_y[k]);
//        }
//        fprintf(fp,"\n");
}

// Main code:
int main(int argc,char* argv[])
{
	// (when V-REP launches this executable, V-REP will also provide the argument list)
	//numero de argumentos que mande (se excluye el fantasma que el manda solo)
	int Narg=0;
//	float periodo=0.0, f=0.0;
	Narg=Nmotores;

	if (argc>=Narg)
	{
        for (k=0;k<Nmotores;k++) JointHandle[k] = atoi(argv[1+k]);
	}
	else
	{
		ROS_ERROR("centroMasa: Indique argumentos!\n");
		return (0);
	}

	for(k=0;k<Npatas;k++){
        cm_Robot.centroMasaPata_x.push_back(0);
        cm_Robot.centroMasaPata_y.push_back(0);
	}
	// Create a ROS node:
	int _argc = 0;
	char** _argv = NULL;
	ros::init(_argc,_argv,"CentrodeMasa");

	if(!ros::master::check())
		return(0);

	ros::NodeHandle node;
	ROS_INFO("CentrodeMasa just started\n");

    //Subscripciones y publicaciones
    ros::Subscriber subInfo1=node.subscribe("/vrep/info",1,infoCallback);
    ros::Subscriber subInfo2=node.subscribe("UbicacionRobot",100,ubicacionRobCallback);
    chatter_pub = node.advertise<camina::UbicacionRobot>("CentrodeMasa", 100);
    //Clientes y servicios
    client_GetJointStates=node.serviceClient<vrep_common::simRosGetJointState>("/vrep/simRosGetJointState");
//    srv_GetJointStates.request.handle=sim_handle_all;

//    periodo=1;
//    f=1/periodo;
//    ros::Rate loop_rate(f);  //Frecuencia [Hz]

    anguloPatas_rad = anguloPatas*pi/180;
    PosicionPata_x = radioCuerpo*cos(anguloPatas_rad);
    PosicionPata_y = radioCuerpo*sin(anguloPatas_rad);
    PosicionPata_x2 = radioCuerpo;
    PosicionPata_y2 = 0.0;

    fp = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina/XYPatas.txt","w+");

	while (ros::ok() && simulationRunning)
	{
		ros::spinOnce();
//		loop_rate.sleep();
	}
	fclose(fp);
	ROS_INFO("Adios CentrodeMasa!");
	ros::shutdown();
	return(0);
}

/*Funcion que calcula el centro de masa de cada eslabon
.. es necesario calcular solo las posiciones del "medio"
.. de los eslabones
*/
void CentroDeMasa_Eslabon(punto3d* CentroDeMasa, float q1, float q2, float q3, int articulacion){

    if (articulacion == 3){
        CentroDeMasa->x = cos(q1)*(L1 + L2*cos(q2) + cm_L3*cos(q2+q3));
        CentroDeMasa->y = sin(q1)*(L1 + L2*cos(q2) + cm_L3*cos(q2+q3));
        CentroDeMasa->z = L2*sin(q2) + cm_L3*sin(q2+q3);
    }else if (articulacion == 2){
        CentroDeMasa->x = cos(q1)*(L1 + cm_L2*cos(q2));
        CentroDeMasa->y = sin(q1)*(L1 + cm_L2*cos(q2));
        CentroDeMasa->z = L2*sin(q2);
    }else if (articulacion == 1){
        CentroDeMasa->x = cos(q1)*cm_L1;
        CentroDeMasa->y = sin(q1)*cm_L1;
        CentroDeMasa->z = 0;
    }
    else{
        ROS_ERROR("No es posible sacar el calculo, articulacion invalida!");
    }
}

/*FUNCION QUE CALCULA EL CENTRO DE MASA DE UN PATA EN FUNCIÓN DE SUS ESLABONES*/
void CentroDeMasa_Pata(punto3d* CentroDeMasa, punto3d cm_eslabon1, punto3d cm_eslabon2, punto3d cm_eslabon3, int Npata){

    float masaPata = masa_L1 + masa_L2 + masa_L3;
//    punto3d cm_aux;
//        CentroDeMasa->x = (cm_eslabon1.x*(masa_L1) + cm_eslabon2.x*masa_L2 + cm_eslabon3.x*(masa_L3))/masaPata;
//        CentroDeMasa->y = (cm_eslabon1.y*(masa_L1) + cm_eslabon2.y*masa_L2 + cm_eslabon3.y*(masa_L3))/masaPata;
//        CentroDeMasa->z = (cm_eslabon1.z*(masa_L1) + cm_eslabon2.z*masa_L2 + cm_eslabon3.z*(masa_L3))/masaPata;

    if(Npata==0){
        CentroDeMasa->x = (-(cm_eslabon1.x+PosicionPata_x)*(masa_L1) - (cm_eslabon2.x+PosicionPata_x)*masa_L2 - (cm_eslabon3.x+PosicionPata_x)*(masa_L3))/masaPata;
        CentroDeMasa->y = ((cm_eslabon1.y+PosicionPata_y)*(masa_L1) + (cm_eslabon2.y+PosicionPata_y)*masa_L2 + (cm_eslabon3.y+PosicionPata_y)*(masa_L3))/masaPata;
        CentroDeMasa->z = (cm_eslabon1.z*(masa_L1) + cm_eslabon2.z*masa_L2 + cm_eslabon3.z*(masa_L3))/masaPata;
    }
    if(Npata==1){
        CentroDeMasa->x = ((cm_eslabon1.x+PosicionPata_x)*(masa_L1) + (cm_eslabon2.x+PosicionPata_x)*masa_L2 + (cm_eslabon3.x+PosicionPata_x)*(masa_L3))/masaPata;
        CentroDeMasa->y = ((cm_eslabon1.y+PosicionPata_y)*(masa_L1) + (cm_eslabon2.y+PosicionPata_y)*masa_L2 + (cm_eslabon3.y+PosicionPata_y)*(masa_L3))/masaPata;
        CentroDeMasa->z = (cm_eslabon1.z*(masa_L1) + cm_eslabon2.z*masa_L2 + cm_eslabon3.z*(masa_L3))/masaPata;
    }
    if(Npata==2){
        CentroDeMasa->x = (-(cm_eslabon1.x+PosicionPata_x2)*(masa_L1) - (cm_eslabon2.x+PosicionPata_x2)*masa_L2 - (cm_eslabon3.x+PosicionPata_x2)*(masa_L3))/masaPata;
        CentroDeMasa->y = ((cm_eslabon1.y+PosicionPata_y2)*(masa_L1) + (cm_eslabon2.y+PosicionPata_y2)*masa_L2 + (cm_eslabon3.y+PosicionPata_y2)*(masa_L3))/masaPata;
        CentroDeMasa->z = (cm_eslabon1.z*(masa_L1) + cm_eslabon2.z*masa_L2 + cm_eslabon3.z*(masa_L3))/masaPata;
    }
    if(Npata==3){
        CentroDeMasa->x = ((cm_eslabon1.x+PosicionPata_x2)*(masa_L1) + (cm_eslabon2.x+PosicionPata_x2)*masa_L2 + (cm_eslabon3.x+PosicionPata_x2)*(masa_L3))/masaPata;
        CentroDeMasa->y = ((cm_eslabon1.y+PosicionPata_y2)*(masa_L1) + (cm_eslabon2.y+PosicionPata_y2)*masa_L2 + (cm_eslabon3.y+PosicionPata_y2)*(masa_L3))/masaPata;
        CentroDeMasa->z = (cm_eslabon1.z*(masa_L1) + cm_eslabon2.z*masa_L2 + cm_eslabon3.z*(masa_L3))/masaPata;
    }
    if(Npata==4){
        CentroDeMasa->x = (-(cm_eslabon1.x+PosicionPata_x)*(masa_L1) - (cm_eslabon2.x+PosicionPata_x)*masa_L2 - (cm_eslabon3.x+PosicionPata_x)*(masa_L3))/masaPata;
        CentroDeMasa->y = (-(cm_eslabon1.y+PosicionPata_y)*(masa_L1) - (cm_eslabon2.y+PosicionPata_y)*masa_L2 - (cm_eslabon3.y+PosicionPata_y)*(masa_L3))/masaPata;
        CentroDeMasa->z = (cm_eslabon1.z*(masa_L1) + cm_eslabon2.z*masa_L2 + cm_eslabon3.z*(masa_L3))/masaPata;
    }
    if(Npata==5){
        CentroDeMasa->x = ((cm_eslabon1.x+PosicionPata_x)*(masa_L1) + (cm_eslabon2.x+PosicionPata_x)*masa_L2 + (cm_eslabon3.x+PosicionPata_x)*(masa_L3))/masaPata;
        CentroDeMasa->y = (-(cm_eslabon1.y+PosicionPata_y)*(masa_L1) - (cm_eslabon2.y+PosicionPata_y)*masa_L2 - (cm_eslabon3.y+PosicionPata_y)*(masa_L3))/masaPata;
        CentroDeMasa->z = (cm_eslabon1.z*(masa_L1) + cm_eslabon2.z*masa_L2 + cm_eslabon3.z*(masa_L3))/masaPata;
    }
}

/*Funcion que calcula el centro de masa del hexapodo, tomando en cuenta sus 6 patas*/
void CentroDeMasa_Hexapodo(punto3d* CentroDeMasa, punto3d cm_pata1, punto3d cm_pata2, punto3d cm_pata3, punto3d cm_pata4, punto3d cm_pata5, punto3d cm_pata6){

    float masaPata = masa_L1 + masa_L2 + (masa_L3);

//    CentroDeMasa->x = masaPata*(-(PosicionPata_x+cm_pata1.x)+(PosicionPata_x+cm_pata2.x)-(PosicionPata_x2+cm_pata3.x)+(PosicionPata_x2+cm_pata4.x)-(PosicionPata_x+cm_pata5.x)+(PosicionPata_x+cm_pata6.x))/(masa_Base+Npatas*masaPata);
//    CentroDeMasa->y = masaPata*((PosicionPata_y+cm_pata1.y)+(PosicionPata_y+cm_pata2.y)+(PosicionPata_y2+cm_pata3.y)+(PosicionPata_y2+cm_pata4.y)-(PosicionPata_y+cm_pata5.y)-(PosicionPata_y+cm_pata6.y))/(masa_Base+Npatas*masaPata);
//    CentroDeMasa->z = ((cm_pata1.z+cm_pata2.z+cm_pata3.z+cm_pata4.z+cm_pata5.z+cm_pata6.z) + 0*masa_base)/(masa+masa_base);
    CentroDeMasa->x = masaPata*(cm_pata1.x+cm_pata2.x+cm_pata3.x+cm_pata4.x+cm_pata5.x+cm_pata6.x)/(masa_Base+Npatas*masaPata);
    CentroDeMasa->y = masaPata*(cm_pata1.y+cm_pata2.y+cm_pata3.y+cm_pata4.y+cm_pata5.y+cm_pata6.y)/(masa_Base+Npatas*masaPata);
}



