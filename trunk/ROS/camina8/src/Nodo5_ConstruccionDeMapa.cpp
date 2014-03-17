#include "ros/ros.h"
#include "std_msgs/String.h"
#include "math.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <allegro.h>
//Librerias propias usadas
#include "constantes.hpp"
#include "camina7/v_repConst.h"
// Used data structures:
#include "std_msgs/String.h"
#include "camina7/InfoMapa.h"
#include "camina7/UbicacionRobot.h"
#include "camina7/EspacioTrabajoParametros.h"
// Used API services:
#include "vrep_common/VrepInfo.h"
#include "vrep_common/simRosEnableSubscriber.h"
//Definiciones
#define VentanaY 640
#define VentanaX 640
//Clientes y Servicios
ros::ServiceClient client_EspacioTrabajo;
camina7::EspacioTrabajoParametros srv_EspacioTrabajo;

// Global variables (modified by topic subscribers):
bool simulationRunning=true;
bool sensorTrigger=false;
float simulationTime=0.0f;
camina7::InfoMapa infoMapa;
int matrizMapa[100][100];
bool bool_matrizMapa[100][100];
int nCeldas_i=0, nCeldas_j=0;
int cantidadObstaculos=0;
int coordenadaAjuste_i[6]={0,0,0,0,0,0}, coordenadaAjuste_j[6]={0,0,0,0,0,0}, coordenadaPreAjuste_i[6]={0,0,0,0,0,0}, coordenadaPreAjuste_j[6]={0,0,0,0,0,0};
float LongitudCeldaY=0.0, LongitudCeldaX=0.0;
int ij[2], *p_ij;     //Apuntadores a arreglos de coordenadas e indices
ros::Publisher chatter_pub;
FILE *fp1,*fp2;
BITMAP *buffer;
int divisionX=0, divisionY=0;
int cuentaPasos=0;
Obstaculo obstaculo[1000];
//float PosicionPata_x=0.0, PosicionPata_y=0.0, PosicionPata_x2=0.0, PosicionPata_y2=0.0, anguloPatas_rad=0.0;
//float origenPata_x[6], origenPata_y[6], rotacionPata[6];
//float EspacioTrabajoPata_x[6][4],EspacioTrabajoPata_y[6][4];
//float EspacioTrabajoPatas_ij[6][8];

// Funciones
int Construye_matrizMapa(std::string fileName, int nCeldas_i, int nCeldas_j);
void Limpiar_matrizMapa(int nFilas, int nColumnas, int cantidadObstaculos);
void print_matrizMapa(int nFilas, int nColumnas);
void FilePrint_matrizMapa(FILE *fp, int nFilas, int nColumnas);
void transformacion_yxTOij(int *ptr_ij, float y, float x);
void IniciaGrafica();
void FuncionGrafica_Cuerpo();
//void FuncionGrafica_EspacioTrabajo();
void Info_Obstaculos(std::string fileName, int N_Obstaculos);

// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}

void ubicacionRobCallback(camina7::UbicacionRobot msgUbicacionRobot)
{
//    srv_EspacioTrabajo.request.PosicionCuerpo_y = msgUbicacionRobot.coordenadaCuerpo_y;
//    srv_EspacioTrabajo.request.PosicionCuerpo_x = msgUbicacionRobot.coordenadaCuerpo_x;
//    srv_EspacioTrabajo.request.theta_CuerpoRobot = msgUbicacionRobot.orientacionCuerpo_yaw;
//    if (client_EspacioTrabajo.call(srv_EspacioTrabajo))
//    {
//        for (int k=0; k<Npuntos;k++) {
//            EspacioTrabajoPata_x[0][k] = srv_EspacioTrabajo.response.EspacioTrabajoPata1_x[k];
//            EspacioTrabajoPata_y[0][k] = srv_EspacioTrabajo.response.EspacioTrabajoPata1_y[k];
//            EspacioTrabajoPata_x[1][k] = srv_EspacioTrabajo.response.EspacioTrabajoPata2_x[k];
//            EspacioTrabajoPata_y[1][k] = srv_EspacioTrabajo.response.EspacioTrabajoPata2_y[k];
//            EspacioTrabajoPata_x[2][k] = srv_EspacioTrabajo.response.EspacioTrabajoPata3_x[k];
//            EspacioTrabajoPata_y[2][k] = srv_EspacioTrabajo.response.EspacioTrabajoPata3_y[k];
//            EspacioTrabajoPata_x[3][k] = srv_EspacioTrabajo.response.EspacioTrabajoPata4_x[k];
//            EspacioTrabajoPata_y[3][k] = srv_EspacioTrabajo.response.EspacioTrabajoPata4_y[k];
//            EspacioTrabajoPata_x[4][k] = srv_EspacioTrabajo.response.EspacioTrabajoPata5_x[k];
//            EspacioTrabajoPata_y[4][k] = srv_EspacioTrabajo.response.EspacioTrabajoPata5_y[k];
//            EspacioTrabajoPata_x[5][k] = srv_EspacioTrabajo.response.EspacioTrabajoPata6_x[k];
//            EspacioTrabajoPata_y[5][k] = srv_EspacioTrabajo.response.EspacioTrabajoPata6_y[k];
//        }
//    } else {
//        ROS_ERROR("Nodo 5: servicio de EspacioTrabajo no funciona\n");
////        return;
//    }

    for (int k=0; k<Npatas;k++) {
        transformacion_yxTOij(p_ij, msgUbicacionRobot.coordenadaPata_y[k], msgUbicacionRobot.coordenadaPata_x[k]);
        infoMapa.coordenadaPata_i[k]=ij[0];
        infoMapa.coordenadaPata_j[k]=ij[1];
        infoMapa.pataApoyo[k] = msgUbicacionRobot.pataApoyo[k];
    }
//    chatter_pub.publish(infoMapa);
//    print_matrizMapa(nCeldas_y,nCeldas_x);
}

void ajusteCallback(camina7::InfoMapa msgInfoMapa)
{
    cuentaPasos++;
	for(int k=0; k<Npatas; k++){
	    coordenadaPreAjuste_i[k]=msgInfoMapa.coordenadaPreAjuste_i[k];
        coordenadaPreAjuste_j[k]=msgInfoMapa.coordenadaPreAjuste_j[k];
	    coordenadaAjuste_i[k]=msgInfoMapa.coordenadaAjuste_i[k];
        coordenadaAjuste_j[k]=msgInfoMapa.coordenadaAjuste_j[k];
	}

    fprintf(fp2,"\ntiempo de simulacion: %.3f",simulationTime);
    fprintf(fp2,"\nPaso:%d",cuentaPasos);
    if (msgInfoMapa.correccion){
        fprintf(fp2,"\nCorreccion:Si");
    } else {
        fprintf(fp2,"\nCorreccion:No");
    }
    Limpiar_matrizMapa(nCeldas_i,nCeldas_j,cantidadObstaculos);
    for(int k=0; k<Npatas; k++) {
        matrizMapa[coordenadaPreAjuste_i[k]][coordenadaPreAjuste_j[k]]=k+1;
        matrizMapa[coordenadaAjuste_i[k]][coordenadaAjuste_j[k]]='a';
        fprintf(fp2,"\n[%d]Preajuste:i=%d,j=%d - ajuste:i=%d,j=%d",k+1,coordenadaPreAjuste_i[k],coordenadaPreAjuste_j[k],coordenadaAjuste_i[k],coordenadaAjuste_j[k]);
//            ROS_INFO("Nodo5: Posicion actual:[%d] i:%d\t j:%d",k+1,coordenadaPreAjuste_i[k],coordenadaPreAjuste_j[k]);
    }
    for(int k=0; k<Npatas; k++) {
        if(bool_matrizMapa[coordenadaPreAjuste_i[k]][coordenadaPreAjuste_j[k]]){
            fprintf(fp2,"\nPata[%d] Coincide Obstaculo: [%d][%d]",k+1,coordenadaPreAjuste_i[k],coordenadaPreAjuste_j[k]);
            ROS_WARN("Pata[%d] coincide con obstaculo",k+1);
        }
    }
    FilePrint_matrizMapa(fp2, nCeldas_i,nCeldas_j);

    fprintf(fp1,"\ntiempo de simulacion: %.3f",simulationTime);
    Limpiar_matrizMapa(nCeldas_i,nCeldas_j,cantidadObstaculos);
    for(int k=0; k<Npatas; k++) {
        matrizMapa[infoMapa.coordenadaPata_i[k]][infoMapa.coordenadaPata_j[k]]=k+1;
//            ROS_INFO("Nodo5: Posicion actual:[%d] i:%d\t j:%d",k+1,coordenadaPreAjuste_i[k],coordenadaPreAjuste_j[k]);
    }
    FilePrint_matrizMapa(fp1, nCeldas_i,nCeldas_j);
}

int main(int argc, char **argv)
{
	char** _argv = NULL;
	int _argc = 0;
	int Narg=0;
	int cuentaObs=0;
	float Lx=0.0, Ly=0.0;
    float periodo, f;
	int k=0;
	std::string Name, M_fileName,O_fileName;

    Narg=6;

    if (argc>Narg)
	{
        Ly = atof(argv[1]);
        Lx = atof(argv[2]);
        nCeldas_i = atoi(argv[3]);
        nCeldas_j = atoi(argv[4]);
        Name = argv[5];
        cantidadObstaculos = atoi(argv[6]);
	} else{
        ROS_ERROR("Nodo 5: Indique argumentos completos!\n");
        return (0);
     }

//--- Inicializacion de nodo de ROS
    ros::init(_argc,_argv,"Nodo5_ConstruccionDeMapa");
	ROS_INFO("Nodo5_ConstruccionDeMapa just started");

    ros::NodeHandle node;
//-- Topicos susbcritos y publicados
    ros::Subscriber subInfo1=node.subscribe("/vrep/info",100,infoCallback);
    ros::Subscriber subInfo2=node.subscribe("UbicacionRobot",100,ubicacionRobCallback);
    ros::Subscriber subInfo3=node.subscribe("Plan",100,ajusteCallback);
    chatter_pub = node.advertise<camina7::InfoMapa>("GraficaMapa", 100);
    //Clientes y Servicios
//    client_EspacioTrabajo=node.serviceClient<camina7::EspacioTrabajoParametros>("EspacioTrabajo");

//--- Inicializacion de variables
    LongitudCeldaY = infoMapa.tamanoCelda_i = Ly/(float) nCeldas_i;
    LongitudCeldaX = infoMapa.tamanoCelda_j = Lx/(float) nCeldas_j;
//    infoMapa.tamanoCelda_i = Ly/(float) nCeldas_i;
//    infoMapa.tamanoCelda_j = Lx/(float) nCeldas_j;
//-- Inicializacion de mensaje para graficar
    infoMapa.nCeldas_i=nCeldas_i;
    infoMapa.nCeldas_j=nCeldas_j;
    infoMapa.cantidadObstaculos = cantidadObstaculos;
    for (k=0; k<cantidadObstaculos;k++) {
        infoMapa.coordenadaObstaculo_i.push_back(0);
        infoMapa.coordenadaObstaculo_j.push_back(0);
    }
    for (k=0; k<Npatas;k++) {
        infoMapa.coordenadaPata_i.push_back(0);
        infoMapa.coordenadaPata_j.push_back(0);
        infoMapa.pataApoyo.push_back(0);
    }
    // Inicializacion de apuntadores
    p_ij = ij;
    // Inicializacion de matriz nula
    for(int i=0;i<nCeldas_i;i++){
        for(int j=0;j<nCeldas_j;j++){
            matrizMapa[i][j]=-1;
            bool_matrizMapa[i][j]=false;
        }
    }
    M_fileName = O_fileName = Name;
    std::string obs("_o");
    std::string txt(".txt");
    M_fileName+=txt;
    cuentaObs = Construye_matrizMapa(M_fileName, nCeldas_i, nCeldas_j);
    printf("obstaculos recibidos: %d, obstaculos contados: %d\n",cantidadObstaculos,cuentaObs);
    O_fileName+=obs;
    O_fileName+=txt;
    Info_Obstaculos(O_fileName,cuentaObs);
//--- Inicializacion de grafica
//    IniciaGrafica();
//--- Inicializacion de archivo de salida
    fp1 = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina7/datos/SalidaMapa_robot.txt","w+");
    fp2 = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina7/datos/SalidaMapa_ajuste.txt","w+");
//--- Ciclo de ROS
//	periodo=1.5;
//    f=1/periodo;
//    ros::Rate loop_rate(f);  //Frecuencia [Hz]
//    for (int i=0; i<3; i++) loop_rate.sleep();
    chatter_pub.publish(infoMapa);

	while (ros::ok() && simulationRunning)
	{
//        for (int k=0; k<Npatas;k++) {
//            //-- Punto 1
//            transformacion_yxTOij(p_ij,EspacioTrabajoPata_y[k][0],EspacioTrabajoPata_x[k][0]);
//            EspacioTrabajoPatas_ij[k][0]=ij[0];
//            EspacioTrabajoPatas_ij[k][1]=ij[1];
//            //-- Punto 2
//            transformacion_yxTOij(p_ij,EspacioTrabajoPata_y[k][1],EspacioTrabajoPata_x[k][1]);
//            EspacioTrabajoPatas_ij[k][2]=ij[0];
//            EspacioTrabajoPatas_ij[k][3]=ij[1];
//            //-- Punto 3
//            transformacion_yxTOij(p_ij,EspacioTrabajoPata_y[k][2],EspacioTrabajoPata_x[k][2]);
//            EspacioTrabajoPatas_ij[k][4]=ij[0];
//            EspacioTrabajoPatas_ij[k][5]=ij[1];
//            //-- Punto 4
//            transformacion_yxTOij(p_ij,EspacioTrabajoPata_y[k][3],EspacioTrabajoPata_x[k][3]);
//            EspacioTrabajoPatas_ij[k][6]=ij[0];
//            EspacioTrabajoPatas_ij[k][7]=ij[1];
//        }
//    //-- Funcion para grafica de cuerpo
//        FuncionGrafica_Cuerpo();

//        chatter_pub.publish(infoMapa);
        ros::spinOnce();
//        loop_rate.sleep();
	}
	fclose(fp1); fclose(fp2);
    ROS_INFO("AdiosGrafica!");
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
            if (matrizMapa[i][j] == '-'){
                printf ("-.");
            } else if (matrizMapa[i][j] == 'o'){
                printf ("o.");
            } else {
                printf(" %d.",matrizMapa[i][j]);
            }
         }
        printf("\n");
    }
}

int Construye_matrizMapa(std::string fileName, int nCeldas_i, int nCeldas_j){
    //--- Construccion de mapa mediante lectura de archivo
//    printf ("%s \n", fileName.c_str());
    FILE *fp;
    int int_aux=0, i=0, j=0, cuentaObs=0;

    fp = fopen(fileName.c_str(), "r");
    if(fp!=NULL){
        printf("\n");
        for(i=0;i<nCeldas_i;i++){
            for(j=0;j<nCeldas_j;j++){
                fscanf(fp, "%d", &int_aux);
                if (int_aux==0) {
                    matrizMapa[i][j]='-';
                    bool_matrizMapa[i][j]=false;
                }
                if (int_aux==1) {
                    matrizMapa[i][j]='o';
                    bool_matrizMapa[i][j]=true;
                    infoMapa.coordenadaObstaculo_i[cuentaObs]=i;
                    infoMapa.coordenadaObstaculo_j[cuentaObs]=j;
                    cuentaObs++;
//                    ROS_INFO("obstaculo_i: %d",i);
                }
            }
        }
//        printf("Mapa inicial\n");
//        print_matrizMapa(nCeldas_i,nCeldas_j);
    }
    fclose(fp);
    return (cuentaObs);
}

void Info_Obstaculos(std::string fileName, int N_Obstaculos){
    //--- Construccion de mapa mediante lectura de archivo
//    printf ("%s \n", fileName.c_str());
    FILE *fp;
    int int_aux=0, i=0;
    float f_aux=0.0;

    fp = fopen(fileName.c_str(), "r");
    if(fp!=NULL){
        for(i=0;i<N_Obstaculos;i++){
            fscanf(fp, "%d", &int_aux);
            obstaculo[i].i=int_aux;
            fscanf(fp, "%d", &int_aux);
            obstaculo[i].j=int_aux;
            fscanf(fp, "%f", &f_aux);
            obstaculo[i].O_x=f_aux;
            fscanf(fp, "%f", &f_aux);
            obstaculo[i].O_y=f_aux;
            fscanf(fp, "%f", &f_aux);
            obstaculo[i].P1_x=f_aux;
            fscanf(fp, "%f", &f_aux);
            obstaculo[i].P1_y=f_aux;
            fscanf(fp, "%f", &f_aux);
            obstaculo[i].P2_x=f_aux;
            fscanf(fp, "%f", &f_aux);
            obstaculo[i].P2_y=f_aux;
            fscanf(fp, "%f", &f_aux);
            obstaculo[i].P3_x=f_aux;
            fscanf(fp, "%f", &f_aux);
            obstaculo[i].P3_y=f_aux;
            fscanf(fp, "%f", &f_aux);
            obstaculo[i].P4_x=f_aux;
            fscanf(fp, "%f", &f_aux);
            obstaculo[i].P4_y=f_aux;
        }
    }
    fclose(fp);
}

void Limpiar_matrizMapa(int nFilas, int nColumnas, int cantidadObstaculos){

    int i=0, j=0, k=0;

    for(i=0;i<nFilas;i++){
        for(j=0;j<nColumnas;j++){
            matrizMapa[i][j]='-';
            bool_matrizMapa[i][j]=false;
        }
    }
    for(k=0;k<cantidadObstaculos;k++){
        matrizMapa[infoMapa.coordenadaObstaculo_i[k]][infoMapa.coordenadaObstaculo_j[k]]='o';
        bool_matrizMapa[infoMapa.coordenadaObstaculo_i[k]][infoMapa.coordenadaObstaculo_j[k]]=true;
    }
}

void FilePrint_matrizMapa(FILE *fp, int nFilas, int nColumnas)
{
     int i=0, j=0;

     fprintf(fp,"\n");
     for(i=0;i<nFilas;i++)
     {
         for(j=0;j<nColumnas;j++)
         {
            if (matrizMapa[i][j] == '-'){
                fprintf(fp,"-.");
            }

            else if (matrizMapa[i][j] == 'o'){
                    fprintf(fp,"o.");
            }
            else if (matrizMapa[i][j] == '*'){
                    fprintf(fp,"*.");
            }
            else if (matrizMapa[i][j] == 'a'){
                    fprintf(fp,"a.");
            }
            else {

            fprintf(fp,"%d.",matrizMapa[i][j]);
            }
         }
        fprintf(fp,"\n");
    }
}

/*En matriz de mapa las coordenadas van de i=[0,99], j=[0,19] */
void transformacion_yxTOij(int *ptr_ij, float y, float x){
    ptr_ij[0] = (int) (nCeldas_i/2 - floor(y/LongitudCeldaY)-1);
    ptr_ij[1] = (int) (nCeldas_j/2 + floor(x/LongitudCeldaX));
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

    divisionY = VentanaY/nCeldas_i;
    divisionX = VentanaX/nCeldas_j;
}

void FuncionGrafica_Cuerpo(){

    int posicionY=0, posicionX=0, k=0;

    clear_to_color(buffer, makecol(255, 255, 255));
    //Dibuja matriz de celdas
    for(k=0; k<nCeldas_i;k++){
        hline(buffer, 0, divisionY*k, (divisionY*nCeldas_i)-divisionY, makecol(0,0,0));
    }
    for(k=0; k<nCeldas_j;k++){
        vline(buffer, divisionX*k, 0, (divisionX*nCeldas_j)-divisionX, makecol(0,0,0));
    }
    //Dibuja obstaculos
    for (k=0; k<infoMapa.cantidadObstaculos;k++){
        posicionY = divisionY*(infoMapa.coordenadaObstaculo_i[k]) - divisionY/2;
        posicionX = divisionX*(infoMapa.coordenadaObstaculo_j[k]) - divisionX/2;
        circlefill(buffer, posicionX, posicionY, divisionX/2, makecol(0,0,0));
//            textprintf_ex(buffer, font, posicionPataX, posicionPataY, makecol(255,255,255), -1, "pata1");
    }
    //Dibuja posicion de patas
    for (k=0; k<Npatas;k++){
        posicionY = divisionY*(infoMapa.coordenadaPata_i[k]) - divisionY/2;
        posicionX = divisionX*(infoMapa.coordenadaPata_j[k]) - divisionX/2;

        if (infoMapa.pataApoyo[k]==1) {
            //La pata esta en apoyo
            circlefill(buffer, posicionX, posicionY, divisionX/2, makecol(150,150,150));
            textprintf_ex(buffer, font, posicionX, posicionY, makecol(0,0,0), -1, "pata%d:A",k+1);
        } else {
            //La pata esta en transferencia
            circlefill(buffer, posicionX, posicionY, divisionX/2, makecol(150,150,150));
            textprintf_ex(buffer, font, posicionX, posicionY, makecol(0,0,0), -1, "pata%d:T",k+1);
        }
    }
//    FuncionGrafica_EspacioTrabajo();

    blit(buffer, screen, 0, 0, 0, 0, SCREEN_W, SCREEN_H);
}

//void FuncionGrafica_EspacioTrabajo(){
//    int R,G,B;
//    int posicionY=0, posicionX=0, k=0;
//    //Dibuja posicion Espacio de trabajo de patas
//    for (k=0; k<Npatas;k++){
//        switch (k){
//            case 0:// Pata 1: rojo
//                R=255;
//                G=0;
//                B=0;
//            break;
//            case 1:// Pata 2: verde
//                R=0;
//                G=255;
//                B=0;
//            break;
//            case 2:// Pata 3: azul
//                R=0;
//                G=0;
//                B=255;
//            break;
//            case 3:// Pata 4: amarillo
//                R=255;
//                G=255;
//                B=0;
//            break;
//            case 4:// Pata 5: rosado
//                R=255;
//                G=0;
//                B=255;
//            break;
//            case 5:// Pata 6: azul claro
//                R=0;
//                G=255;
//                B=255;
//            break;
//        }
//        posicionY = divisionY*(EspacioTrabajoPatas_ij[k][0]) - divisionY/2;
//        posicionX = divisionX*(EspacioTrabajoPatas_ij[k][1]) - divisionX/2;
//        circlefill(buffer, posicionX, posicionY, divisionX/2, makecol(R,G,B));
////        textprintf_ex(buffer, font, posicionX, posicionY, makecol(0,0,0), -1, "pata%d:A",k+1);
//        posicionY = divisionY*(EspacioTrabajoPatas_ij[k][2]) - divisionY/2;
//        posicionX = divisionX*(EspacioTrabajoPatas_ij[k][3]) - divisionX/2;
//        circlefill(buffer, posicionX, posicionY, divisionX/2, makecol(R,G,B));
////        textprintf_ex(buffer, font, posicionX, posicionY, makecol(0,0,0), -1, "pata%d:A",k+1);
//        posicionY = divisionY*(EspacioTrabajoPatas_ij[k][4]) - divisionY/2;
//        posicionX = divisionX*(EspacioTrabajoPatas_ij[k][5]) - divisionX/2;
//        circlefill(buffer, posicionX, posicionY, divisionX/2, makecol(R,G,B));
////        textprintf_ex(buffer, font, posicionX, posicionY, makecol(0,0,0), -1, "pata%d:A",k+1);
//        posicionY = divisionY*(EspacioTrabajoPatas_ij[k][6]) - divisionY/2;
//        posicionX = divisionX*(EspacioTrabajoPatas_ij[k][7]) - divisionX/2;
//        circlefill(buffer, posicionX, posicionY, divisionX/2, makecol(R,G,B));
////        textprintf_ex(buffer, font, posicionX, posicionY, makecol(0,0,0), -1, "pata%d:A",k+1);
//    }
//}
