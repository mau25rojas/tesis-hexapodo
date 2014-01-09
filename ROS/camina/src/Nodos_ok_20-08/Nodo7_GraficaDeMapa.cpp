#include "ros/ros.h"
#include "math.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <allegro.h>
#include <time.h>
//Librerias propias usadas
#include "constantes.hpp"
#include "camina/v_repConst.h"
// Used data structures:
#include "std_msgs/String.h"
#include "../msg_gen/cpp/include/camina/InfoMapa.h"
#include "../msg_gen/cpp/include/camina/UbicacionRobot.h"
// Used API services:
#include "vrep_common/VrepInfo.h"
#include "vrep_common/simRosEnableSubscriber.h"
//Definiciones
#define VentanaY 640
#define VentanaX 640

// Global variables (modified by topic subscribers):
bool simulationRunning=true;
bool sensorTrigger=false;
float simulationTime=0.0f;
int coordenadaPata_i[6]={0,0,0,0,0,0}, coordenadaPata_j[6]={0,0,0,0,0,0};
std::vector<int> coordenadaObstaculo_i, coordenadaObstaculo_j;
int pataApoyo[6]={0,0,0,0,0,0};
//int cm_Pata_i[6]={0,0,0,0,0,0}, cm_Pata_j[6]={0,0,0,0,0,0}, cm_Cuerpo_i=0, cm_Cuerpo_j=0;
int k=0, cantidadObstaculos=0;
int nCeldas_i=0, nCeldas_j=0;
float LongitudCeldaY=0.0, LongitudCeldaX=0.0;
int ij[2], *p_ij;     //Apuntadores a arreglos de coordenadas e indices

// Funciones
//void transformacion_ijTOyx(float *ptr_yx, int i, int j, float divisionY, float divisionX);
void transformacion_yxTOij(int *ptr_ij, float y, float x);

// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}
void graficaCallback(camina::InfoMapa msgInfoMapa)
{
	for(k=0; k<cantidadObstaculos; k++){
	    coordenadaObstaculo_i[k]=msgInfoMapa.coordenadaObstaculo_i[k];
        coordenadaObstaculo_j[k]=msgInfoMapa.coordenadaObstaculo_j[k];
	}
}
void ubicacionRobCallback(camina::UbicacionRobot msgUbicacionRobot)
{
    for(k=0; k<Npatas;k++) {
        transformacion_yxTOij(p_ij, msgUbicacionRobot.coordenadaPata_y[k], msgUbicacionRobot.coordenadaPata_x[k]);
        coordenadaPata_i[k]=ij[0];
        coordenadaPata_j[k]=ij[1];
        pataApoyo[k] = msgUbicacionRobot.pataApoyo[k];
//        printf("Pata apoyo[%d]=%d\n",k,pataApoyo[k]);
    }
}

//void cmCallback(camina::UbicacionRobot msgUbicacionRobot)
//{
//	transformacion_yxTOij(p_ij, msgUbicacionRobot.centroMasaCuerpo_y, msgUbicacionRobot.centroMasaCuerpo_x);
//    cm_Cuerpo_i=ij[0];
//    cm_Cuerpo_j=ij[1];
//
//    for(k=0;k<Npatas;k++){
//        transformacion_yxTOij(p_ij, msgUbicacionRobot.centroMasaPata_y[k], msgUbicacionRobot.centroMasaPata_x[k]);
//        cm_Pata_i[k]=ij[0];
//        cm_Pata_j[k]=ij[1];
//    }
//}

int main(int argc, char **argv){

    char** _argv = NULL;
	int _argc = 0;
    BITMAP *buffer;
    int divisionX=0, divisionY=0, posicionY=0, posicionX=0;
    int Narg=0;
    float Lx=0.0,Ly=0.0;

    Narg=4;
    if (argc>=Narg)
	{
        Ly = atof(argv[1]);
        Lx = atof(argv[2]);
        nCeldas_i = atoi(argv[3]);
        nCeldas_j = atoi(argv[4]);
        cantidadObstaculos = atoi(argv[5]);
	} else
	{
		ROS_ERROR("Nodo7: Indique argumentos!\n");
		return (0);
	}
	//Inicializo apuntador
	p_ij=ij;

	LongitudCeldaY = Ly/(float) nCeldas_i;
    LongitudCeldaX = Lx/(float) nCeldas_j;

    divisionY = VentanaY/nCeldas_i;
    divisionX = VentanaX/nCeldas_j;
    for (k=0; k<cantidadObstaculos;k++) {
        coordenadaObstaculo_i.push_back(0);
        coordenadaObstaculo_j.push_back(0);
    }

    if (allegro_init() != 0) return 1;
        install_keyboard();
        install_timer();

       if (set_gfx_mode(GFX_AUTODETECT_WINDOWED, VentanaX, VentanaY, 0, 0) != 0) {
          if (set_gfx_mode(GFX_SAFE, VentanaX, VentanaY, 0, 0) != 0) {
         set_gfx_mode(GFX_TEXT, 0, 0, 0, 0);
         allegro_message("Unable to set any graphic mode\n%s\n", allegro_error);
         return 1;
          }
       }
        set_palette(desktop_palette);
        /* allocate the memory buffer */
        buffer = create_bitmap(SCREEN_W, SCREEN_H);
        /* and now with a double buffer... */
        clear_keybuf();

        ros::init(_argc,_argv,"Nodo_GraficaDeMapa");
        ROS_INFO("Nodo7_GraficaDeMapa just started\n");
        ros::NodeHandle node;
        //Topicos susbcritos y publicados
        ros::Subscriber subInfo=node.subscribe("/vrep/info",100,infoCallback);
        ros::Subscriber sub=node.subscribe("GraficaMapa",100,graficaCallback);
        ros::Subscriber sub2=node.subscribe("UbicacionRobot",100,ubicacionRobCallback);
//        ros::Subscriber sub=node.subscribe("CentrodeMasa",1000,cmCallback);

        while(ros::ok() && simulationRunning && !keypressed()){
//            while(!keypressed()){
            ros::spinOnce();

            clear_to_color(buffer, makecol(255, 255, 255));
            //Dibuja matriz de celdas
            for(k=0; k<nCeldas_i;k++){
                hline(buffer, 0, divisionY*k, (divisionY*nCeldas_i)-divisionY, makecol(0,0,0));
            }
            for(k=0; k<nCeldas_j;k++){
                vline(buffer, divisionX*k, 0, (divisionX*nCeldas_j)-divisionX, makecol(0,0,0));
            }
            //Dibuja obstaculos
            for (k=0; k<cantidadObstaculos;k++){
                posicionY = divisionY*(coordenadaObstaculo_i[k]) - divisionY/2;
                posicionX = divisionX*(coordenadaObstaculo_j[k]) - divisionX/2;
                circlefill(buffer, posicionX, posicionY, divisionX/2, makecol(0,0,0));
    //            textprintf_ex(buffer, font, posicionPataX, posicionPataY, makecol(255,255,255), -1, "pata1");
            }
            //Dibuja posicion de patas
            for (k=0; k<Npatas;k++){
                posicionY = divisionY*(coordenadaPata_i[k]) - divisionY/2;
                posicionX = divisionX*(coordenadaPata_j[k]) - divisionX/2;

                if (pataApoyo[k]==1) {
                    //La pata esta en apoyo
                    circlefill(buffer, posicionX, posicionY, divisionX/2, makecol(0,0,255));
                    textprintf_ex(buffer, font, posicionX, posicionY, makecol(0,0,0), -1, "pata%d:A",k+1);
                } else {
                    //La pata esta en transferencia
                    circlefill(buffer, posicionX, posicionY, divisionX/2, makecol(0,255,0));
                    textprintf_ex(buffer, font, posicionX, posicionY, makecol(255,0,0), -1, "pata%d:T",k+1);
                }
            }
//            // Dibuja centro de masa de cuerpo
//            posicionY = divisionY*(cm_Cuerpo_i) - divisionY/2;
//            posicionX = divisionX*(cm_Cuerpo_j) - divisionX/2;
//            circlefill(buffer, posicionX, posicionY, divisionX/2, makecol(255,0,0));
//            textprintf_ex(buffer, font, posicionX, posicionY, makecol(0,0,0), -1, "cuerpo%d",k+1);
////            //Dibuja centro de masa de patas
//            for (k=0; k<Npatas;k++){
//                posicionY = divisionY*(cm_Pata_i[k]) - divisionY/2;
//                posicionX = divisionX*(cm_Pata_j[k]) - divisionX/2;
//                circlefill(buffer, posicionX, posicionY, divisionX/2, makecol(0,0,255));
//                textprintf_ex(buffer, font, posicionX, posicionY, makecol(0,0,0), -1, "pata%d",k+1);
//            }
            blit(buffer, screen, 0, 0, 0, 0, SCREEN_W, SCREEN_H);
       }
    ROS_INFO("AdiosNodo7Grafica!");
    ros::shutdown();
    return 0;
}
END_OF_MAIN()

//void transformacion_ijTOyx(float *ptr_yx, int i, int j, float divisionY, float divisionX){
//
//    ptr_yx[0] = divisionY*i - divisionY/2;
//    ptr_yx[1] = divisionX*j - divisionX/2;
//
//}

void transformacion_yxTOij(int *ptr_ij, float y, float x){
    if (y>=0){
        ptr_ij[0] = (int) (nCeldas_i/2 - floor(y/LongitudCeldaY));
    }else{
        ptr_ij[0] = (int) (nCeldas_i/2 - floor(y/LongitudCeldaY)-1);
    }

    if (x>=0){
        ptr_ij[1] = (int) (nCeldas_j/2 + floor(x/LongitudCeldaX));
    }else{
        ptr_ij[1] = (int) (nCeldas_j/2 + floor(x/LongitudCeldaX)-1);
    }
}
