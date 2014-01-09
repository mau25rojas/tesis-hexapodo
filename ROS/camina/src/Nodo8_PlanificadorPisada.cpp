#include "ros/ros.h"
#include "std_msgs/String.h"
#include "math.h"
#include <algorithm>    // std::sort
//Librerias propias usadas
#include "constantes.hpp"
#include "camina/v_repConst.h"
// Used data structures:
#include "camina/DatosTrayectoriaPata.h"
#include "camina/InfoMapa.h"
#include "camina/CinversaParametros.h"
#include "camina/UbicacionRobot.h"
// Used API services:
#include "vrep_common/VrepInfo.h"
// Definiciones
#define MAX_CICLOS 5
//Clientes y Servicios
ros::ServiceClient client_Cinversa;
camina::CinversaParametros srv_Cinversa;

//-- Variables Globales
bool simulationRunning=true;
bool sensorTrigger=false;
float simulationTime=0.0f;
//-- Log de planificador
FILE *fp2;
//-- Variables de mapa
camina::InfoMapa infoMapa;
std::vector<int> coordenadaObstaculo_i(1000,0), coordenadaObstaculo_j(1000,0);
bool matrizMapa[100][100];
int nCeldas_i=0, nCeldas_j=0;
float LongitudCeldaY=0, LongitudCeldaX=0;
//-- Variables de ubicacion robot
float posicionActualPata_y[Npatas], posicionActualPata_x[Npatas], posicionActualPata_z[Npatas];
float posicionActualPataSistemaPata_y[Npatas],posicionActualPataSistemaPata_x[Npatas],posicionActualPataSistemaPata_z[Npatas];
int pataApoyo[Npatas]={0,0,0,0,0,0};
float teta_CuerpoRobot=0.0;
//-- Generales
//int k=0;

//-- Funciones
void transformacion_yxTOij(int *ptr_ij, float y, float x);
void FilePrint_matrizMapa(int nFilas, int nColumnas);
void Limpiar_matrizMapa(int nFilas, int nColumnas, int cantidadObstaculos);
void BubbleSort(float *num, int N);

//-- Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}

void infoMapaCallback(camina::InfoMapa msgInfoMapa)
{
    int cantidadObstaculos=0;
	nCeldas_i = msgInfoMapa.nCeldas_i;
	nCeldas_j = msgInfoMapa.nCeldas_j;
	LongitudCeldaY=msgInfoMapa.tamanoCelda_i;
	LongitudCeldaX=msgInfoMapa.tamanoCelda_j;
    cantidadObstaculos = msgInfoMapa.cantidadObstaculos;
	for(int k=0; k<cantidadObstaculos; k++){
	    coordenadaObstaculo_i[k]=msgInfoMapa.coordenadaObstaculo_i[k];
        coordenadaObstaculo_j[k]=msgInfoMapa.coordenadaObstaculo_j[k];
	}
	Limpiar_matrizMapa(nCeldas_i,nCeldas_j,cantidadObstaculos);
}

void ubicacionRobCallback(camina::UbicacionRobot msgUbicacionRobot)
{
    teta_CuerpoRobot = msgUbicacionRobot.orientacionCuerpo_yaw;
    for(int k=0; k<Npatas;k++) {
        posicionActualPata_x[k] = msgUbicacionRobot.coordenadaPata_x[k];
        posicionActualPata_y[k] = msgUbicacionRobot.coordenadaPata_y[k];
        posicionActualPata_z[k] = msgUbicacionRobot.coordenadaPata_z[k];
        posicionActualPataSistemaPata_x[k] = msgUbicacionRobot.coordenadaPataSistemaPata_x[k];
        posicionActualPataSistemaPata_y[k] = msgUbicacionRobot.coordenadaPataSistemaPata_y[k];
        posicionActualPataSistemaPata_z[k] = msgUbicacionRobot.coordenadaPataSistemaPata_z[k];
        pataApoyo[k] = msgUbicacionRobot.pataApoyo[k];
    }
}

int main(int argc, char **argv)
{
    int Narg=0, tripode[Npatas], Tripode1[Npatas/2], Tripode2[Npatas/2], cuenta_T1=0, cuenta_T2=0, Tripode_Apoyo[Npatas/2];
    int ij[2]={0,0}, *p_ij;     //Apuntadores a arreglos de coordenadas e indices
    int PisadaProxima_i=0, PisadaProxima_j=0;
    bool PisadaInvalida[Npatas/2], TodasPisadasOk, cinversaOK;
    float velocidad_Apoyo=0.0, beta=0.0, T=0.0;
    float Periodo=0.0, f=0.0, phi[Npatas], alfa=0.0;
    float PisadaProxima_x=0.0, PisadaProxima_y=0.0, PisadaProxima_z=0.0;
    float delta_x_S0=0.0, delta_y_S0=0.0, delta_x=0.0, delta_y=0.0;
    float modificacion_T[Npatas/2], modificacion_lambda[Npatas/2];
    camina::DatosTrayectoriaPata modificacion_datosTrayectoria;

    Narg=16;
    if (argc>=Narg)
	{
        T = atof(argv[1]);
        beta = atof(argv[2]);
        velocidad_Apoyo = atof(argv[3]);
        alfa = atof(argv[4])*pi/180.0;
        for(int k=0;k<Npatas;k++) phi[k] = atof(argv[5+k])*pi/180.0;
        for(int k=0;k<Npatas;k++) tripode[k] = atoi(argv[5+Npatas+k]);
	} else{
        ROS_ERROR("Nodo 8: Indique argumentos completos!\n");
        return (0);
    }

    /*Inicio nodo de ROS*/
    ros::init(argc, argv, "Nodo8_PlanificacionPisada");
    ros::NodeHandle node;
    ROS_INFO("Nodo8_PlanificacionPisada just started\n");

//-- Topicos susbcritos y publicados
    ros::Publisher chatter_pub1=node.advertise<camina::DatosTrayectoriaPata>("PlanificacionPisada", 100);
    ros::Publisher chatter_pub2=node.advertise<camina::InfoMapa>("Plan", 100);
    ros::Subscriber sub1=node.subscribe("/vrep/info",100,infoCallback);
    ros::Subscriber sub2=node.subscribe("UbicacionRobot",100,ubicacionRobCallback);
    ros::Subscriber sub3=node.subscribe("GraficaMapa",100,infoMapaCallback);
//-- Clientes y Servicios
    client_Cinversa=node.serviceClient<camina::CinversaParametros>("Cinversa");

    /* Log de planificador */
    fp2 = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina/datos/LogPlanificador.txt","w+");

    //-- Patas de [0-5]
    for(int k=0;k<Npatas;k++) {
        if(tripode[k]==1){
            Tripode1[cuenta_T1]=k;
            cuenta_T1++;
        } else {
            Tripode2[cuenta_T2]=k;
            cuenta_T2++;
        }
    }
//-- Datos para envio de mensajes
    modificacion_datosTrayectoria.T.push_back(0);
    modificacion_datosTrayectoria.lambda_Transferencia.push_back(0);

    for(int k=0;k<Npatas;k++) {
        infoMapa.coordenadaPreAjuste_i.push_back(0);
        infoMapa.coordenadaPreAjuste_j.push_back(0);
        infoMapa.coordenadaAjuste_i.push_back(0);
        infoMapa.coordenadaAjuste_j.push_back(0);
    }
    p_ij = ij;    // Inicialización de apuntador

    /* Velocidad de transmision */
    Periodo = 0.5;
    f=1/Periodo;
    ros::Rate loop_rate(f);  //Frecuencia [Hz]
    //-- Delay inicial para esperar inicio de todos los nodos
    for(int k=0;k<20;k++) loop_rate.sleep();

//    modificacion_datosTrayectoria.Tripode = 1;
//    int cuenta=0;
//    while (ros::ok() && simulationRunning){
//        ros::spinOnce();
//        loop_rate.sleep();
//        cuenta++;
//        if (cuenta==30){
//            cuenta = 0;
//            modificacion_datosTrayectoria.lambda_Transferencia[0]=0.03;
//            modificacion_datosTrayectoria.T[0] = modificacion_datosTrayectoria.lambda_Transferencia[0]/(beta*velocidad_Apoyo);
//            ROS_INFO("nodo8: publica cambio: lamda=%.3f, T=%.3f",modificacion_datosTrayectoria.lambda_Transferencia[0],modificacion_datosTrayectoria.T[0]);
//            chatter_pub1.publish(modificacion_datosTrayectoria);
//        }
//    }
//    fclose(fp2);
//    ROS_INFO("Adios8!");
//    ros::shutdown();
//    return 0;

while (ros::ok() && simulationRunning){
    ros::spinOnce();
    loop_rate.sleep();
//        ROS_INFO("Revision pisadas");
//-- Busqueda ¿que tripode esta en apoyo?
    cuenta_T1=0;
    cuenta_T2=0;
    for(int k=0;k<Npatas;k++){
        if (pataApoyo[k]==1){
//            ROS_INFO("Nodo8: Pata apoyo[%d]=%d",k,pataApoyo[k]);
            if (tripode[k]==1){
                cuenta_T1++;
//                ROS_INFO("Nodo8: cuenta_T1: %d",cuenta_T1);
            } else {
                cuenta_T2++;
//                ROS_INFO("Nodo8: cuenta_T2: %d",cuenta_T2);
            }
        }
    }
    if (cuenta_T1>cuenta_T2){
        for(int k=0;k<Npatas/2;k++) Tripode_Apoyo[k] = Tripode1[k];
        modificacion_datosTrayectoria.Tripode = 1;
//        ROS_INFO("Nodo8: Tripode en apoyo = 1");
//        ROS_INFO("Nodo8: [%d,%d,%d]",Tripode_Apoyo[0],Tripode_Apoyo[1],Tripode_Apoyo[2]);
    } else {
        for(int k=0;k<Npatas/2;k++) Tripode_Apoyo[k] = Tripode2[k];
        modificacion_datosTrayectoria.Tripode = 2;
//        ROS_INFO("Nodo8: Tripode en apoyo = 2");
//        ROS_INFO("Nodo8: [%d,%d,%d]",Tripode_Apoyo[0],Tripode_Apoyo[1],Tripode_Apoyo[2]);
    }

    TodasPisadasOk = true;    // Todas las pisadas se asumen bien a la primera
    for(int k=0;k<Npatas/2;k++){
//        ROS_INFO("Nodo8: revisando pata [%d]", Tripode_Apoyo[k]);
        ros::spinOnce();
        //-- Calculamos proximo movimiento en el sistema de pata
            delta_x_S0 = -lambda_maximo*cos(alfa);
            delta_y_S0 = lambda_maximo*sin(alfa);
            delta_x = delta_x_S0*cos(phi[Tripode_Apoyo[k]]+alfa)-delta_y_S0*sin(phi[Tripode_Apoyo[k]]+alfa);
            delta_y = delta_x_S0*sin(phi[Tripode_Apoyo[k]]+alfa)+delta_y_S0*cos(phi[Tripode_Apoyo[k]]+alfa);
            PisadaProxima_x=posicionActualPataSistemaPata_x[Tripode_Apoyo[k]] + delta_x;
            PisadaProxima_y=posicionActualPataSistemaPata_y[Tripode_Apoyo[k]] + delta_y;
            PisadaProxima_z=posicionActualPataSistemaPata_z[Tripode_Apoyo[k]];
        //-- Verificamos que pisada sea factible
            srv_Cinversa.request.x = PisadaProxima_x;
            srv_Cinversa.request.y = PisadaProxima_y;
            srv_Cinversa.request.z = PisadaProxima_z;
            if (client_Cinversa.call(srv_Cinversa)){
            //-- Funciona servicio
                cinversaOK=true;
            } else {
                ROS_ERROR("Nodo 8: servicio de Cinversa no funciona\n");
                cinversaOK=false;
            }
            if (cinversaOK){
                ros::spinOnce();
                // Calculamos proximo movimiento en el sistema mundo
                    PisadaProxima_y=posicionActualPata_y[Tripode_Apoyo[k]] + (lambda_maximo+velocidad_Apoyo*(1-beta)*T)*cos((teta_CuerpoRobot-teta_Offset)+alfa);
                    PisadaProxima_x=posicionActualPata_x[Tripode_Apoyo[k]] + (lambda_maximo+velocidad_Apoyo*(1-beta)*T)*sin((teta_CuerpoRobot-teta_Offset)+alfa);
                    transformacion_yxTOij(p_ij, PisadaProxima_y, PisadaProxima_x);
                    PisadaProxima_i=ij[0];
                    PisadaProxima_j=ij[1];
                    PisadaInvalida[k] = false;
                    if(matrizMapa[PisadaProxima_i][PisadaProxima_j]){
                    // La pisada COINCIDE con obstaculo
                        ROS_WARN("Nodo8: pata [%d] coincide con obstaculo",Tripode_Apoyo[k]+1);
                        fprintf(fp2,"\ntiempo de simulacion: %.3f\n",simulationTime);
                        fprintf(fp2,"pata [%d] coincide con obstaculo\n",Tripode_Apoyo[k]+1);
                        PisadaInvalida[k] = true;
                        TodasPisadasOk = false;
                        break;
                    }
                if(!PisadaInvalida[k]){
                //-- Todo salio bien! Esta proxima_pisada es valida! :D
                    modificacion_lambda[k] = lambda_maximo;
                //-- Pisada maxima
                    infoMapa.coordenadaAjuste_i[Tripode_Apoyo[k]] = PisadaProxima_i;
                    infoMapa.coordenadaAjuste_j[Tripode_Apoyo[k]] = PisadaProxima_j;
                    transformacion_yxTOij(p_ij, posicionActualPata_y[Tripode_Apoyo[k]], posicionActualPata_x[Tripode_Apoyo[k]]);
                    infoMapa.coordenadaPreAjuste_i[Tripode_Apoyo[k]] = ij[0];
                    infoMapa.coordenadaPreAjuste_j[Tripode_Apoyo[k]] = ij[1];
                }
            } else {
                ROS_WARN("Nodo8: pata [%d] error cinversa",k+1);
                fprintf(fp2,"\ntiempo de simulacion: %.3f\n",simulationTime);
                fprintf(fp2,"pata [%d] error cinversa\n",Tripode_Apoyo[k]+1);
                PisadaInvalida[k] = true;
                TodasPisadasOk = false;
            }
    } // Fin de revision de pisadas

//---> Aqui va codigo para arreglar pisadas invalidas
    float lambda_Correccion = 0.0;
    int ciclosContraccion = 0;
    while (ciclosContraccion<MAX_CICLOS && !TodasPisadasOk){
        fprintf(fp2,"\ntiempo de simulacion: %.3f\n",simulationTime);
        fprintf(fp2,"Correccion de pisada ciclo: %d\n",ciclosContraccion);
        ROS_WARN("Nodo8: Correccion de pisada ciclo: %d",ciclosContraccion);
        ciclosContraccion++;
        TodasPisadasOk = true;
    //-- Se corrige en tamaños de celda
        lambda_Correccion = lambda_Correccion+LongitudCeldaY/2;
        for (int k=0;k<Npatas/2;k++){
            ros::spinOnce();
            if(PisadaInvalida[k]){
            //-- Calculamos proximo movimiento en el sistema de pata
                delta_x_S0 = -(lambda_maximo-lambda_Correccion)*cos(alfa);
                delta_y_S0 = (lambda_maximo-lambda_Correccion)*sin(alfa);
                delta_x = delta_x_S0*cos(phi[Tripode_Apoyo[k]]+alfa)-delta_y_S0*sin(phi[Tripode_Apoyo[k]]+alfa);
                delta_y = delta_x_S0*sin(phi[Tripode_Apoyo[k]]+alfa)+delta_y_S0*cos(phi[Tripode_Apoyo[k]]+alfa);
                PisadaProxima_x=posicionActualPataSistemaPata_x[Tripode_Apoyo[k]] + delta_x;
                PisadaProxima_y=posicionActualPataSistemaPata_y[Tripode_Apoyo[k]] + delta_y;
                PisadaProxima_z=posicionActualPataSistemaPata_z[Tripode_Apoyo[k]];
            //-- Verificamos que pisada sea factible
                srv_Cinversa.request.x = PisadaProxima_x;
                srv_Cinversa.request.y = PisadaProxima_y;
                srv_Cinversa.request.z = PisadaProxima_z;
                if (client_Cinversa.call(srv_Cinversa)){
                //-- Funciona servicio
                    cinversaOK=true;
                } else {
                    ROS_ERROR("Nodo 8: servicio de Cinversa no funciona\n");
                    cinversaOK=false;
                }
                if (cinversaOK){
                    ros::spinOnce();
                //-- Calculamos proximo movimiento en el sistema mundo
                    PisadaProxima_y=posicionActualPata_y[Tripode_Apoyo[k]] + ((lambda_maximo-lambda_Correccion)+velocidad_Apoyo*(1-beta)*T)*cos((teta_CuerpoRobot-teta_Offset)+alfa);
                    PisadaProxima_x=posicionActualPata_x[Tripode_Apoyo[k]] + ((lambda_maximo-lambda_Correccion)+velocidad_Apoyo*(1-beta)*T)*sin((teta_CuerpoRobot-teta_Offset)+alfa);
                    transformacion_yxTOij(p_ij, PisadaProxima_y, PisadaProxima_x);
                    PisadaProxima_i=ij[0];
                    PisadaProxima_j=ij[1];
                    PisadaInvalida[k] = false;
                    if(matrizMapa[PisadaProxima_i][PisadaProxima_j]){
                    //-- La pisada COINCIDE con obstaculo
                        ROS_WARN("Nodo8: pata [%d] coincide con obstaculo",Tripode_Apoyo[k]+1);
                        fprintf(fp2,"\ntiempo de simulacion: %.3f\n",simulationTime);
                        fprintf(fp2,"pata [%d] coincide con obstaculo\n",Tripode_Apoyo[k]+1);
                        PisadaInvalida[k] = true;
                        TodasPisadasOk = false;
                        break;
                    }
                    if(!PisadaInvalida[k]){
                    //-- Todo salio bien! Esta proxima_pisada es valida! :D
                        modificacion_lambda[k] = (lambda_maximo-lambda_Correccion);
                    //-- Pisada corregida
                        infoMapa.coordenadaAjuste_i[Tripode_Apoyo[k]] = PisadaProxima_i;
                        infoMapa.coordenadaAjuste_j[Tripode_Apoyo[k]] = PisadaProxima_j;
                        transformacion_yxTOij(p_ij, posicionActualPata_y[Tripode_Apoyo[k]], posicionActualPata_x[Tripode_Apoyo[k]]);
                        infoMapa.coordenadaPreAjuste_i[Tripode_Apoyo[k]] = ij[0];
                        infoMapa.coordenadaPreAjuste_j[Tripode_Apoyo[k]] = ij[1];
                    }
                } else {
                //-- La pisada no es factible
                    ROS_WARN("Nodo8: pata [%d] error cinversa",Tripode_Apoyo[k]+1);
                    fprintf(fp2,"\ntiempo de simulacion: %.3f\n",simulationTime);
                    fprintf(fp2,"pata [%d] error cinversa\n",Tripode_Apoyo[k]+1);
                    PisadaInvalida[k] = true;
                    TodasPisadasOk = false;
                }
            }// Fin if(pisadaInvalida)
        }   // Fin de for
    }   // Fin while ciclos && !TodasPisadasOk
//-- Si hay alguna pisada invalida detengo la planificacion
    for(int k=0;k<Npatas/2;k++) {
        if(PisadaInvalida[k]){
            ROS_ERROR("Nodo8: No se pudo corregir pisada pata[%d]",Tripode_Apoyo[k]+1);
            fclose(fp2);
            ROS_INFO("Adios8!");
            ros::shutdown();
            return 0;
        }
    }
//-- Escojo el largo de pisada mas corto y lo impongo a todas las patas del tripode
    std::sort (modificacion_lambda, modificacion_lambda+3);
    modificacion_datosTrayectoria.lambda_Transferencia[0] = modificacion_lambda[0];
    modificacion_datosTrayectoria.T[0] = modificacion_datosTrayectoria.lambda_Transferencia[0]/(beta*velocidad_Apoyo);
//-- Envio trayectoria planificada D: chanchanchaaaaaan
    ROS_INFO("Nodo8: Tripode=%d, landa_correccion=%.3f, T_correccion=%.3f",modificacion_datosTrayectoria.Tripode,modificacion_datosTrayectoria.lambda_Transferencia[0],modificacion_datosTrayectoria.T[0]);
    chatter_pub1.publish(modificacion_datosTrayectoria);
//-- Envio datos de planificacion al mapa
    fprintf(fp2,"\ntiempo de simulacion: %.3f\t",simulationTime);
    fprintf(fp2,"Envio datos de mapa");
    chatter_pub2.publish(infoMapa);
} // Fin while de nodo
    fclose(fp2);
    ROS_INFO("Adios8!");
    ros::shutdown();
    return 0;
}


void transformacion_yxTOij(int *ptr_ij, float y, float x){
    if (y>=0){
        ptr_ij[0] = (int) (nCeldas_i/2 - floor(y/LongitudCeldaY)-1);
    }else{
        ptr_ij[0] = (int) (nCeldas_i/2 - floor(y/LongitudCeldaY)-1);
    }

    if (x>=0){
        ptr_ij[1] = (int) (nCeldas_j/2 + floor(x/LongitudCeldaX));
    }else{
        ptr_ij[1] = (int) (nCeldas_j/2 + floor(x/LongitudCeldaX));
    }
}

// Bubble Sort Function for Descending Order
void BubbleSort(float *num, int N){
      int i, j, flag = 1;    // set flag to 1 to start first pass
      int temp;             // holding variable
      for(i=1;(i<=N) && flag; i++){
          flag = 0;
          for (j=0; j<(N-1); j++){
               if (num[j+1]<num[j])      // ascending order simply changes to <
              {
                    temp = num[j];             // swap elements
                    num[j] = num[j+1];
                    num[j+1] = temp;
                    flag = 1;               // indicates that a swap occurred.
               }
          }
     }
     return;   //arrays are passed to functions by address; nothing is returned
}

void Limpiar_matrizMapa(int nFilas, int nColumnas, int cantidadObstaculos){

    int i=0, j=0, k=0;

    for(i=0;i<nFilas;i++){
        for(j=0;j<nColumnas;j++){
            matrizMapa[i][j]=false;
        }
    }
    for(k=0;k<cantidadObstaculos;k++){
        matrizMapa[coordenadaObstaculo_i[k]][coordenadaObstaculo_j[k]]=true;
    }
}
