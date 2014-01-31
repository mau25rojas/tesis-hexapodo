#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <ros/ros.h>
//Librerias propias usadas
#include "constantes.hpp"
#include "Convexhull/vector3d.hpp"
#include "Convexhull/convexhull.cpp"
#include "Convexhull/analisis.cpp"
#include "camina6/v_repConst.h"
#include <allegro.h>
// Used data structures:
#include "camina6/UbicacionRobot.h"
// Used API services:
#include "vrep_common/VrepInfo.h"
#include "vrep_common/simRosEnableSubscriber.h"
#include "vrep_common/simRosGetJointState.h"
//Definiciones
#define VentanaY 700
#define VentanaX 200
#define LongitudY 3
#define LongitudX 1

// Global variables:
bool simulationRunning=true;
bool sensorTrigger=false;
float simulationTime=0.0f;
ros::Publisher chatter_pub;
sensor_msgs::JointState RobJoints;
int JointHandle[Nmotores];
float PosicionPata_x=0.0, PosicionPata_y=0.0, PosicionPata_x2=0.0, PosicionPata_y2=0.0, anguloPatas_rad=0.0;
float teta_CuerpoRobot=0.0, PosicionCuerpo_x=0.0, PosicionCuerpo_y=0.0;
punto3d CM1, CM2, CM3, CM_Pata[Npatas], CM_Hexapodo, COG;
recta3d recta_ME[2];
camina6::UbicacionRobot cm_Robot;
BITMAP *buffer;
FILE *fp, *fp1;
int NPuntos_hull=0;
float ME=0.0;
punto3d posicionesPatas[Npatas], Salida_hull[Npatas];
//Clientes y Servicios
ros::ServiceClient client_GetJointStates;
vrep_common::simRosGetJointState srv_GetJointStates; //Servicio para obtener los joints de vrep
//Funciones
void CentroDeMasa_Eslabon(punto3d* CentroDeMasa, float q1, float q2, float q3, int articulacion);
void CentroDeMasa_Pata(punto3d* CentroDeMasa, punto3d cm_eslabon1, punto3d cm_eslabon2, punto3d cm_eslabon3, int Npata);
void CentroDeMasa_Hexapodo(punto3d* CentroDeMasa, punto3d cm_pata1, punto3d cm_pata2, punto3d cm_pata3, punto3d cm_pata4, punto3d cm_pata5, punto3d cm_pata6);
void IniciaGrafica();
void FuncionGrafica_CM();
punto3d RectaME();

// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}

void ubicacionCuerpoCallback(camina6::UbicacionRobot msgUbicacionCuerpo)
{
//    ROS_INFO("recibe mensaje ubicacion");

    float q1=0.0, q2=0.0, q3=0.0;

    PosicionCuerpo_x = msgUbicacionCuerpo.coordenadaCuerpo_x;
    PosicionCuerpo_y = msgUbicacionCuerpo.coordenadaCuerpo_y;
    teta_CuerpoRobot = msgUbicacionCuerpo.orientacionCuerpo_yaw;

    if (client_GetJointStates.call(srv_GetJointStates)&&(srv_GetJointStates.response.result!=-1))
    {

        RobJoints = srv_GetJointStates.response.state;
//                ROS_INFO("Motor1: %.3f, Motor2: %.3f, Motor3: %.3f", RobJoints.position[0]*180.0/pi,RobJoints.position[1]*180.0/pi,RobJoints.position[2]*180.0/pi);
    } else{
           ROS_ERROR("Nodo centrodemasa: servicio getjoints no funciona\n");
    }

    for(int k=0;k<Npatas;k++){
        q1 = RobJoints.position[0+k*Neslabones];
        q2 = RobJoints.position[1+k*Neslabones];
        q3 = RobJoints.position[2+k*Neslabones];
//                printf("k:%d\tq1:%.3f\tq2:%.3f\tq3:%.3f\n",k,q1,q2,q3);
        CentroDeMasa_Eslabon(&CM1,q1,q2,q3,1);
        CentroDeMasa_Eslabon(&CM2,q1,q2,q3,2);
        CentroDeMasa_Eslabon(&CM3,q1,q2,q3,3);
        CentroDeMasa_Pata(&(CM_Pata[k]),CM1,CM2,CM3,k);
    }
    CentroDeMasa_Hexapodo(&CM_Hexapodo,CM_Pata[0],CM_Pata[1],CM_Pata[2],CM_Pata[3],CM_Pata[4],CM_Pata[5]);

//-- Tranformada de coordenadas para ver posicion en el mundo
    for(int k=0;k<Npatas;k++){
        cm_Robot.centroMasaPata_x[k] = cos(teta_CuerpoRobot)*CM_Pata[k].x-sin(teta_CuerpoRobot)*CM_Pata[k].y+PosicionCuerpo_x;
        cm_Robot.centroMasaPata_y[k] = sin(teta_CuerpoRobot)*CM_Pata[k].x+cos(teta_CuerpoRobot)*CM_Pata[k].y+PosicionCuerpo_y;
    }
    cm_Robot.centroMasaCuerpo_x = cos(teta_CuerpoRobot)*CM_Hexapodo.x-sin(teta_CuerpoRobot)*CM_Hexapodo.y+PosicionCuerpo_x;
    cm_Robot.centroMasaCuerpo_y = sin(teta_CuerpoRobot)*CM_Hexapodo.x+cos(teta_CuerpoRobot)*CM_Hexapodo.y+PosicionCuerpo_y;
    fprintf(fp,"%.3f\t%.3f\t",cm_Robot.centroMasaCuerpo_x,cm_Robot.centroMasaCuerpo_y);
    chatter_pub.publish(cm_Robot);

    punto3d *P, *Q;

    int cuentaPataApoyo=0;
    for(int k=0;k<Npatas;k++){
        if(msgUbicacionCuerpo.pataApoyo[k]==1){
            posicionesPatas[cuentaPataApoyo].x = msgUbicacionCuerpo.coordenadaPata_x[k];
            posicionesPatas[cuentaPataApoyo].y = msgUbicacionCuerpo.coordenadaPata_y[k];
//            ROS_INFO("pata[%d] en apoyo, x=%.3f, y=%.3f",k,posicionesPatas[cuentaPataApoyo].x,posicionesPatas[cuentaPataApoyo].y);
            cuentaPataApoyo++;
        }
    }
    if (cuentaPataApoyo>2){
//        fprintf("%d\t",cuentaPataApoyo);
        P = posicionesPatas;
        Q = Salida_hull;
        NPuntos_hull = convexhull_graham(P,Q,cuentaPataApoyo);
        fprintf(fp1,"%.3f\t",simulationTime);
        fprintf(fp1,"%d\t",NPuntos_hull);
        for(int k=0;k<NPuntos_hull;k++) fprintf(fp1,"%.3f\t%.3f\t",Salida_hull[k].x,Salida_hull[k].y);
        fprintf(fp1,"\n");

        recta3d *S;
        S = recta_ME;
        COG.x=cm_Robot.centroMasaCuerpo_x; COG.y=cm_Robot.centroMasaCuerpo_y;
        ME = margen_est (COG, Q, NPuntos_hull, S);
        fprintf(fp,"%.3f\n",ME);

    //-- Funcion para grafica de cuerpo
//        FuncionGrafica_CM();
    } else {
        ROS_ERROR("Hay SOLO 2 o menos patas en apoyo: no se forma poligono de apoyo");
    }
}

// Main code:
int main(int argc,char* argv[])
{
	int Narg=0;
	float periodo=0.0, f=0.0;

	for(int k=0;k<Npatas;k++){
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
    ros::Subscriber subInfo2=node.subscribe("UbicacionRobot",100,ubicacionCuerpoCallback);
    chatter_pub = node.advertise<camina6::UbicacionRobot>("CentrodeMasa", 100);
    //Clientes y servicios
    client_GetJointStates=node.serviceClient<vrep_common::simRosGetJointState>("/vrep/simRosGetJointState");
    srv_GetJointStates.request.handle=sim_handle_all;

    periodo=1;
    f=1/periodo;
    ros::Rate loop_rate(f);  //Frecuencia [Hz]

    anguloPatas_rad = anguloPatas*pi/180;
    PosicionPata_x = radioCuerpo*cos(anguloPatas_rad);
    PosicionPata_y = radioCuerpo*sin(anguloPatas_rad);
    PosicionPata_x2 = radioCuerpo;
    PosicionPata_y2 = 0.0;
//--- Inicializacion de grafica
//    IniciaGrafica();

    fp = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina6/datos/CM_ME.txt","w+");
    fp1 = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina6/datos/convexhull.txt","w+");

	while (ros::ok() && simulationRunning)
	{
        loop_rate.sleep();
		ros::spinOnce();
	}
	fclose(fp);
	fclose(fp1);
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

    CentroDeMasa->x = masaPata*(cm_pata1.x+cm_pata2.x+cm_pata3.x+cm_pata4.x+cm_pata5.x+cm_pata6.x)/(masa_Base+Npatas*masaPata);
    CentroDeMasa->y = masaPata*(cm_pata1.y+cm_pata2.y+cm_pata3.y+cm_pata4.y+cm_pata5.y+cm_pata6.y)/(masa_Base+Npatas*masaPata);
}



void IniciaGrafica(){
    if (allegro_init() != 0) return;
    install_keyboard();
    install_timer();

   if (set_gfx_mode(GFX_AUTODETECT_WINDOWED, VentanaX, VentanaY, 0, 0) != 0) {
      if (set_gfx_mode(GFX_SAFE, VentanaX, VentanaY, 0, 0) != 0) {
     set_gfx_mode(GFX_TEXT, 0, 0, 0, 0);
     allegro_message("Unable to set any graphic mode\n%s\n", allegro_error);
     return;
      }
   }
    set_palette(desktop_palette);
    /* allocate the memory buffer */
    buffer = create_bitmap(SCREEN_W, SCREEN_H);
    clear_keybuf();
}

void FuncionGrafica_CM(){

    float posicionY=0.0, posicionX=0.0;
    float P1_x=0.0,P1_y=0.0,P2_x=0.0,P2_y=0.0;
    punto3d Punto_ME;

    clear_to_color(buffer, makecol(255, 255, 255));

    posicionX = VentanaX/2 - cm_Robot.centroMasaCuerpo_x*VentanaX/LongitudX;
    posicionY = VentanaY/2 - cm_Robot.centroMasaCuerpo_y*VentanaY/LongitudY;
    circlefill(buffer, posicionX, posicionY, 2, makecol(0,0,255));

    for(int k=0;k<NPuntos_hull;k++){
        posicionX = VentanaX/2 - Salida_hull[k].x*VentanaX/LongitudX;
        posicionY = VentanaY/2 - Salida_hull[k].y*VentanaY/LongitudY;
        circlefill(buffer, posicionX, posicionY, 2, makecol(0,0,0));
    }
    for(int k=0;k<NPuntos_hull;k++){
        P1_x = VentanaX/2 - Salida_hull[k].x*VentanaX/LongitudX;
        P1_y = VentanaY/2 - Salida_hull[k].y*VentanaY/LongitudY;
        if (k==NPuntos_hull-1) {
            P2_x = VentanaX/2 - Salida_hull[0].x*VentanaX/LongitudX;
            P2_y = VentanaY/2 - Salida_hull[0].y*VentanaY/LongitudY;
        } else {
            P2_x = VentanaX/2 - Salida_hull[k+1].x*VentanaX/LongitudX;
            P2_y = VentanaY/2 - Salida_hull[k+1].y*VentanaY/LongitudY;
        }
        line(buffer,P1_x,P1_y,P2_x,P2_y,makecol(0,0,0));
    }

    Punto_ME = RectaME();
    P1_x = VentanaX/2 - COG.x*VentanaX/LongitudX;
    P1_y = VentanaY/2 - COG.y*VentanaY/LongitudY;
    P2_x = VentanaX/2 - Punto_ME.x*VentanaX/LongitudX;
    P2_y = VentanaY/2 - Punto_ME.y*VentanaY/LongitudY;
    line(buffer,P1_x,P1_y,P2_x,P2_y,makecol(255,0,0));

    textprintf_ex(buffer, font, 10, 10, makecol(0,0,0), -1, "ME=%.3f",ME);
    blit(buffer, screen, 0, 0, 0, 0, SCREEN_W, SCREEN_H);
}

punto3d RectaME(){

    vector3d OD;

    OD = recta_ME[0].proyeccion(COG);
    return (OD);
}
