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
#include "camina11/v_repConst.h"
#include "camina11/vector3d.hpp"
// Used data structures:
#include "std_msgs/String.h"
#include "camina11/InfoMapa.h"
#include "camina11/UbicacionRobot.h"
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
//-- Mapa
camina11::InfoMapa infoMapa;
bool bool_matrizMapa[100][20];
int cuentaPasos=0, matrizMapa[100][20];
int cantidadObstaculos=0, nCeldas_i=0, nCeldas_j=0;
int coordenadaAjuste_i[Npatas]={0,0,0,0,0,0}, coordenadaAjuste_j[Npatas]={0,0,0,0,0,0}, coordenadaPata_i[Npatas]={0,0,0,0,0,0}, coordenadaPata_j[Npatas]={0,0,0,0,0,0};
float coordenadaPata_x[Npatas]={0,0,0,0,0,0}, coordenadaPata_y[Npatas]={0,0,0,0,0,0};
//-- Transformaciones
int ij[2], *p_ij;     //Apuntadores a arreglos de coordenadas e indices
int divisionX=0, divisionY=0;
float LongitudCeldaY=0.0, LongitudCeldaX=0.0;
//-- Publicaciones
ros::Publisher chatter_pub;
//-- Salida
FILE *fp2;
//-- Allegro
BITMAP *buffer;
//-- Obstaculos
Obstaculo obstaculo[100][20];
float di=0.0;
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

void ajusteCallback(camina11::InfoMapa msgInfoMapa)
{
    cuentaPasos++;
    int nPata = msgInfoMapa.nPata;
    coordenadaPata_x[nPata]=msgInfoMapa.coordenadaPata_x;
    coordenadaPata_y[nPata]=msgInfoMapa.coordenadaPata_y;
    coordenadaPata_i[nPata]=msgInfoMapa.coordenadaPata_i;
    coordenadaPata_j[nPata]=msgInfoMapa.coordenadaPata_j;
    coordenadaAjuste_i[nPata]=msgInfoMapa.coordenadaAjuste_i;
    coordenadaAjuste_j[nPata]=msgInfoMapa.coordenadaAjuste_j;

    fprintf(fp2,"\ntiempo de simulacion: %.3f",simulationTime);
//    fprintf(fp2,"\nPaso:%d",cuentaPasos);
//    if (msgInfoMapa.correccion){
//        fprintf(fp2,"\nCorreccion:Si");
//    } else {
//        fprintf(fp2,"\nCorreccion:No");
//    }
    Limpiar_matrizMapa(nCeldas_i,nCeldas_j,cantidadObstaculos);
    matrizMapa[coordenadaPata_i[nPata]][coordenadaPata_j[nPata]]=nPata+1;
    matrizMapa[coordenadaAjuste_i[nPata]][coordenadaAjuste_j[nPata]]='a';
    fprintf(fp2,"\n[%d]Preajuste:i=%d,j=%d - ajuste:i=%d,j=%d",nPata+1,coordenadaPata_i[nPata],coordenadaPata_j[nPata],coordenadaAjuste_i[nPata],coordenadaAjuste_j[nPata]);
//            ROS_INFO("Nodo5: Posicion actual:[%d] i:%d\t j:%d",k+1,coordenadaPata_i[k],coordenadaPata_j[k]);

//    punto3d Pata, puntosObstaculo[4], *Q;
//    recta3d recta_di[2], *S;
//
//    for(int k=0; k<Npatas; k++) {
//        if(bool_matrizMapa[coordenadaPata_i[k]][coordenadaPata_j[k]]){
//            puntosObstaculo[0].x=obstaculo[coordenadaPata_i[k]][coordenadaPata_j[k]].P1.x;
//            puntosObstaculo[0].y=obstaculo[coordenadaPata_i[k]][coordenadaPata_j[k]].P1.y;
//            puntosObstaculo[1].x=obstaculo[coordenadaPata_i[k]][coordenadaPata_j[k]].P2.x;
//            puntosObstaculo[1].y=obstaculo[coordenadaPata_i[k]][coordenadaPata_j[k]].P2.y;
//            puntosObstaculo[2].x=obstaculo[coordenadaPata_i[k]][coordenadaPata_j[k]].P3.x;
//            puntosObstaculo[2].y=obstaculo[coordenadaPata_i[k]][coordenadaPata_j[k]].P3.y;
//            puntosObstaculo[3].x=obstaculo[coordenadaPata_i[k]][coordenadaPata_j[k]].P4.x;
//            puntosObstaculo[3].y=obstaculo[coordenadaPata_i[k]][coordenadaPata_j[k]].P4.y;
//            Q = puntosObstaculo;
//
//            S = recta_di;
//            Pata.x=coordenadaPata_x[k]; Pata.y=coordenadaPata_y[k];
//            di = margen_est (Pata,Q,4,S);
////            ROS_WARN("Pata[%d]:%.3f,%.3f; obstaculo:%.3f,%.3f",k+1,Pata.x,Pata.y,obstaculo[coordenadaPata_i[k]][coordenadaPata_j[k]].O_x,obstaculo[coordenadaPata_i[k]][coordenadaPata_j[k]].O_y);
////            ROS_WARN("P1:%.3f,%.3f;P2:%.3f,%.3f,P3:%.3f,%.3f,P4:%.3f,%.3f",puntosObstaculo[0].x,puntosObstaculo[0].y,puntosObstaculo[1].x,puntosObstaculo[1].y,puntosObstaculo[2].x,puntosObstaculo[2].y,puntosObstaculo[3].x,puntosObstaculo[3].y);
////            fprintf(fp2,"\nPata[%d]:%.3f,%.3f->Coincide Obstaculo [%d][%d]; distancia_min:%.3f",k+1,Pata.x,Pata.y,coordenadaPata_i[k],coordenadaPata_j[k],di);
//            ROS_WARN("Mapa::Pata[%d]:%.3f,%.3f->coincide con obstaculo [%d][%d]; di=%.4f",k+1,Pata.x,Pata.y,coordenadaPata_i[k],coordenadaPata_j[k],di);
////            fprintf(fp1,"%d\t%.5f\n",k+1,di);
//        }
//    }
    FilePrint_matrizMapa(fp2,nCeldas_i,nCeldas_j);
}

int main(int argc, char **argv)
{
	char** _argv = NULL;
	int _argc = 0;
	int Narg=0;
	int cuentaObs=0;
	float Lx=0.0, Ly=0.0;
	int k=0;
	std::string fileName, M_fileName,O_fileName;

    Narg=6;

    if (argc>Narg){
        Ly = atof(argv[1]);
        Lx = atof(argv[2]);
        nCeldas_i = atoi(argv[3]);
        nCeldas_j = atoi(argv[4]);
        fileName = argv[5];
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
//    ros::Subscriber subInfo2=node.subscribe("UbicacionRobot",100,ubicacionRobCallback);
    ros::Subscriber subInfo3=node.subscribe("Plan",100,ajusteCallback);
    chatter_pub = node.advertise<camina11::InfoMapa>("GraficaMapa", 100);

//--- Inicializacion de variables
    LongitudCeldaY = Ly/(float) nCeldas_i;
    LongitudCeldaX = Lx/(float) nCeldas_j;
    infoMapa.cantidadObstaculos = cantidadObstaculos;
    for (k=0; k<cantidadObstaculos;k++) {
        infoMapa.coordenadaObstaculo_i.push_back(0);
        infoMapa.coordenadaObstaculo_j.push_back(0);
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
    for(int i=0;i<nCeldas_i;i++){
        for(int j=0;j<nCeldas_j;j++){
            obstaculo[i][j].P1.x=-100;
            obstaculo[i][j].P1.y=-100;
            obstaculo[i][j].P2.x=-100;
            obstaculo[i][j].P2.y=-100;
            obstaculo[i][j].P3.x=-100;
            obstaculo[i][j].P3.y=-100;
            obstaculo[i][j].P4.x=-100;
            obstaculo[i][j].P4.y=-100;
        }
    }
    M_fileName = O_fileName = fileName;
    std::string txt(".txt");
    M_fileName+=txt;
    cuentaObs = Construye_matrizMapa(M_fileName, nCeldas_i, nCeldas_j);
    printf("obstaculos recibidos: %d, obstaculos contados: %d\n",cantidadObstaculos,cuentaObs);
    std::string obs("_o");
    O_fileName+=obs;
    O_fileName+=txt;
    Info_Obstaculos(O_fileName,cuentaObs);
//--- Inicializacion de grafica
//    IniciaGrafica();
//--- Inicializacion de archivo de salida
//    fp1 = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina11/datos/Errores.txt","a+");
    fp2 = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina11/datos/SalidaMapa_ajuste.txt","w+");

//    chatter_pub.publish(infoMapa);
	while (ros::ok() && simulationRunning)
	{
//    //-- Funcion para grafica de cuerpo
//        FuncionGrafica_Cuerpo();

//        chatter_pub.publish(infoMapa);
        ros::spinOnce();
//        loop_rate.sleep();
	}
	fclose(fp2);
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
    int int_aux=0, i=0, j=0, cuentaObs=0, aux=0;

    fp = fopen(fileName.c_str(), "r");
    if(fp!=NULL){
        printf("\n");
        for(i=0;i<nCeldas_i;i++){
            for(j=0;j<nCeldas_j;j++){
                aux = fscanf(fp, "%d", &int_aux);
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
    int int_aux=0, i=0, j=0, aux=0;
    float f_aux=0.0;

    fp = fopen(fileName.c_str(), "r");
    if(fp!=NULL){
        for(int k=0;k<N_Obstaculos;k++){
            aux=fscanf(fp, "%d", &int_aux);
            i=int_aux;
            aux=fscanf(fp, "%d", &int_aux);
            j=int_aux;
            aux=fscanf(fp, "%f", &f_aux);
            obstaculo[i][j].O.x=f_aux;
            aux=fscanf(fp, "%f", &f_aux);
            obstaculo[i][j].O.y=f_aux;
            aux=fscanf(fp, "%f", &f_aux);
            obstaculo[i][j].P1.x=f_aux;
            aux=fscanf(fp, "%f", &f_aux);
            obstaculo[i][j].P1.y=f_aux;
            aux=fscanf(fp, "%f", &f_aux);
            obstaculo[i][j].P2.x=f_aux;
            aux=fscanf(fp, "%f", &f_aux);
            obstaculo[i][j].P2.y=f_aux;
            aux=fscanf(fp, "%f", &f_aux);
            obstaculo[i][j].P3.x=f_aux;
            aux=fscanf(fp, "%f", &f_aux);
            obstaculo[i][j].P3.y=f_aux;
            aux=fscanf(fp, "%f", &f_aux);
            obstaculo[i][j].P4.x=f_aux;
            aux=fscanf(fp, "%f", &f_aux);
            obstaculo[i][j].P4.y=f_aux;
        }
    } else{
        ROS_ERROR("Error al leer archivo %s",fileName.c_str());
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
        posicionY = divisionY*coordenadaPata_i[k] - divisionY/2;
        posicionX = divisionX*coordenadaPata_j[k] - divisionX/2;

//        if (infoMapa.pataApoyo[k]==1) {
            //La pata esta en apoyo
            circlefill(buffer, posicionX, posicionY, divisionX/2, makecol(150,150,150));
            textprintf_ex(buffer, font, posicionX, posicionY, makecol(0,0,0), -1, "pata%d:A",k+1);
//        } else {
//            //La pata esta en transferencia
//            circlefill(buffer, posicionX, posicionY, divisionX/2, makecol(150,150,150));
//            textprintf_ex(buffer, font, posicionX, posicionY, makecol(0,0,0), -1, "pata%d:T",k+1);
//        }
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
