#include <ros/ros.h>
#include <std_msgs/String.h>
#include <math.h>
#include <time.h>
#include "boost/date_time/posix_time/posix_time.hpp"
//Librerias propias usadas
#include "constantes.hpp"
#include "camina9/v_repConst.h"
// Used data structures:
#include "camina9/DatosTrayectoriaPata.h"
#include "camina9/PlanificadorParametros.h"
#include "camina9/SenalesCambios.h"
#include "camina9/UbicacionRobot.h"
// Used API services:
#include "vrep_common/VrepInfo.h"
//Clientes y Servicios
ros::ServiceClient client_Planificador;
camina9::PlanificadorParametros srv_Planificador;

// variables Globales
bool simulationRunning=true;
bool sensorTrigger=false;
bool Inicio=true, InicioApoyo_T1=false, FinApoyo_T1=false, InicioApoyo_T2=false, FinApoyo_T2=false;
bool InicioTransf_T1=false, FinTransf_T1=false, InicioTransf_T2=false, FinTransf_T2=false;
float simulationTime=0.0f;
float divisionTrayectoriaPata=0.0, T=0.0, lambda_Transferencia=0.0, divisionTiempo=0.0, desfasaje_t_T1=0.0,desfasaje_t_T2=0.0, beta=0.0, lambda_Apoyo_actual=0.0;
float modificacion_T = 0.0, modificacion_lambda =0.0;
float delta_t=0.0, t_aux_T1=0.0,t_aux_T2=0.0;
float coordenadaCuerpo_y=0.0,yCuerpo_T1_1=0.0, yCuerpo_T1_2=0.0,yCuerpo_T2_1=0.0, yCuerpo_T2_2=0.0, delta_y=0.0, tiempo_ahora=0.0, velocidadCuerpo_y=0.0;
int pataApoyo[Npatas], tripode[Npatas], Tripode1[Npatas/2], Tripode2[Npatas/2];
int Tripode=0, cuenta=0, PasosIni=0;
FILE *fp1;
boost::posix_time::ptime timerT1_1,timerT1_2,timerT2_1,timerT2_2;
boost::posix_time::time_duration diff_t;
camina9::DatosTrayectoriaPata datosTrayectoriaPata;
ros::Publisher chatter_pub1,chatter_pub2;

// Funciones
bool CambioDeEstado();
// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}

void ubicacionRobCallback(camina9::UbicacionRobot msgUbicacionRobot)
{
     coordenadaCuerpo_y = msgUbicacionRobot.coordenadaCuerpo_y;
     for(int k=0; k<Npatas;k++) {
        pataApoyo[k] = msgUbicacionRobot.pataApoyo[k];
    }
}

void relojCallback(camina9::SenalesCambios msgSenal)
{
    bool llamadaPlan=false;
    datosTrayectoriaPata.cambio_estado[T1-1]=0;
    datosTrayectoriaPata.cambio_estado[T2-1]=0;

    if (!msgSenal.Stop){

        llamadaPlan = CambioDeEstado();

        if (Inicio){
            cuenta++;
            if (cuenta==PasosIni*divisionTrayectoriaPata){
                Inicio=false;
                delta_t=T;
            } else {
                datosTrayectoriaPata.t_Trayectoria[T1-1]=datosTrayectoriaPata.t_Trayectoria[T2-1]=delta_t;
                chatter_pub1.publish(datosTrayectoriaPata);

                delta_t = delta_t + T/divisionTrayectoriaPata;
                if (fabs(delta_t-T)<=(T/divisionTrayectoriaPata)) {
                    delta_t = 0.0;
                    if (datosTrayectoriaPata.vector_estados[T1-1]==0){ datosTrayectoriaPata.vector_estados[T1-1]=1;
                    } else { datosTrayectoriaPata.vector_estados[T1-1]=0;}
                    if (datosTrayectoriaPata.vector_estados[T2-1]==0){ datosTrayectoriaPata.vector_estados[T2-1]=1;
                    } else { datosTrayectoriaPata.vector_estados[T2-1]=0;}
                }
            }
        } else {
            //-------------------------------
            if(llamadaPlan){
//                ROS_INFO("Llama a plan tripode[%d]",Tripode);
                llamadaPlan = false;
            //-- reinicio cuenta para iniciar apoyo
                delta_t = 0.0;
                for(int k=0;k<Npatas;k++) datosTrayectoriaPata.correccion_di[k]=0.0;
            //-- la distancia en apoyo se mantiene según la distancia recorrida en transferencia
                if (Tripode==T1){
                    datosTrayectoriaPata.lambda_Apoyo[T1-1]=datosTrayectoriaPata.lambda_Transferencia[T1-1];
                    lambda_Apoyo_actual=datosTrayectoriaPata.lambda_Apoyo[T1-1];
                //-- calculo de velocidad
//////////////////////////////////////////////////////////////////////////////////////////////
/* //////////////////FALTA CALCULO DE MODULO DE VELOCIDAD///////////////////////////////////////////////
*///////////////////////////////////////////////////////////////////////////////////////////////
                    delta_y = fabs(yCuerpo_T1_1-yCuerpo_T1_2);
                    diff_t = timerT1_1 - timerT1_2;
                    tiempo_ahora = (float) fabs(diff_t.total_milliseconds())/1000;
                    velocidadCuerpo_y = delta_y/tiempo_ahora;
                }else{
                    datosTrayectoriaPata.lambda_Apoyo[T2-1]=datosTrayectoriaPata.lambda_Transferencia[T2-1];
                    lambda_Apoyo_actual=datosTrayectoriaPata.lambda_Apoyo[T2-1];
                //-- calculo de velocidad
                    diff_t = timerT2_1 - timerT2_2;
                    tiempo_ahora = (float) fabs(diff_t.total_milliseconds())/1000;
                    velocidadCuerpo_y = delta_y/tiempo_ahora;
                }
                ROS_INFO("Nodo1::T[%d]: delta_y=%.3f, diff_t=%.3f, velocidad=%.3f",Tripode,delta_y,tiempo_ahora,velocidadCuerpo_y);
                fprintf(fp1,"%.3f,%.3f,%.3f\n",delta_y,tiempo_ahora,velocidadCuerpo_y);

                srv_Planificador.request.Tripode = Tripode;
                srv_Planificador.request.T = T;
                srv_Planificador.request.lambda = lambda_Apoyo_actual;
                srv_Planificador.request.velApoyo_y = velocidadCuerpo_y;
                if (client_Planificador.call(srv_Planificador)){
                    modificacion_lambda = srv_Planificador.response.modificacion_lambda;
                    modificacion_T = srv_Planificador.response.modificacion_T;
                    datosTrayectoriaPata.correccion_di = srv_Planificador.response.correccion_di;
                    datosTrayectoriaPata.posicionActualSistemaPata_x = srv_Planificador.response.posicionActualSistemaPata_x;
                    datosTrayectoriaPata.posicionActualSistemaPata_y = srv_Planificador.response.posicionActualSistemaPata_y;
                    datosTrayectoriaPata.posicionActualSistemaPata_z = srv_Planificador.response.posicionActualSistemaPata_z;
                    ROS_INFO("Nodo1::T[%d]: t_sim=%.3f, lambda_c=%.3f,t_c=%.3f",Tripode,simulationTime,modificacion_lambda,modificacion_T);

                } else {
                    ROS_ERROR("Nodo1::T[%d] servicio de Planificacion no funciona",Tripode);
                    ROS_ERROR("result=%d", srv_Planificador.response.result);
                }

                T = modificacion_T;
                divisionTrayectoriaPata = T/divisionTiempo;
                //-- datos a enviar
                if (Tripode==T1){
                //-- Correccion en T1 significa inicio de apoyo en T1 y transferencia en T2
                    datosTrayectoriaPata.vector_estados[T1-1]=0;
                    datosTrayectoriaPata.vector_estados[T2-1]=1;
                    datosTrayectoriaPata.cambio_estado[T1-1]=1;
                    datosTrayectoriaPata.cambio_estado[T2-1]=1;
                //-- El tiempo de apoyo de T1 es tiempo de trans de T2
                //....La correccion obtenida se aplica en lambda_trans T2
                    datosTrayectoriaPata.T_apoyo[T1-1]=datosTrayectoriaPata.T_apoyo[T2-1]=modificacion_T;
                    datosTrayectoriaPata.lambda_Transferencia[T2-1]=modificacion_lambda;
                } else {
                //-- Correccion en T2 significa inicio de apoyo en T2 y transferencia en T1
                    datosTrayectoriaPata.vector_estados[T1-1]=1;
                    datosTrayectoriaPata.vector_estados[T2-1]=0;
                    datosTrayectoriaPata.cambio_estado[T1-1]=1;
                    datosTrayectoriaPata.cambio_estado[T2-1]=1;
                //-- El tiempo de apoyo de T2 es tiempo de trans de T1
                //....La correccion obtenida se aplica en lambda_trans T1
                    datosTrayectoriaPata.T_apoyo[T1-1]=datosTrayectoriaPata.T_apoyo[T2-1]=modificacion_T;
                    datosTrayectoriaPata.lambda_Transferencia[T1-1]=modificacion_lambda;
                }
            }// Fin llamada plan

            datosTrayectoriaPata.t_Trayectoria[T1-1]=datosTrayectoriaPata.t_Trayectoria[T2-1]=delta_t;
//            fprintf(fp1,"%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",modificacion_T,datosTrayectoriaPata.t_Trayectoria[T1-1],datosTrayectoriaPata.lambda_Apoyo[T1-1],datosTrayectoriaPata.lambda_Transferencia[T1-1],datosTrayectoriaPata.lambda_Apoyo[T2-1], datosTrayectoriaPata.lambda_Transferencia[T2-1]);
            chatter_pub1.publish(datosTrayectoriaPata);
    //        delta_t = delta_t + T/divisionTrayectoriaPata;
            if (fabs(delta_t-T)<=(T/divisionTrayectoriaPata)) {
                delta_t = delta_t;
//                ROS_INFO("Esperando apoyo, delta_t=%.4f",delta_t);
            } else {
                delta_t = delta_t + T/divisionTrayectoriaPata;
            }
        }
    }//-- fin is !Stop
} //-- fin de callback

int main(int argc, char **argv)
{
  float lambda_Apoyo=0.0, alfa=0.0, f=0.0;
  int Narg=0;

  Narg=14;
	if (argc>=Narg)
	{
        PasosIni=atoi(argv[1]);
        T=atof(argv[2]); // Periodo de trayectoria [seg]
		divisionTrayectoriaPata=atof(argv[3]);  //N puntos
		beta=atof(argv[4]);
		lambda_Apoyo=atof(argv[5]);
		lambda_Transferencia=atof(argv[6]);
		alfa=atof(argv[7])*pi/180;
        desfasaje_t_T1=atof(argv[8]);
        desfasaje_t_T2=atof(argv[9]);
        for(int k=0;k<Npatas;k++) tripode[k] = atoi(argv[10+k]);
    } else {
		ROS_ERROR("Nodo1: Indique argumentos!\n");
		return 0;
	}

	/*Inicio nodo de ROS*/
    std::string nodeName("Nodo1_datosTrayectoriaPata");
	ros::init(argc,argv,nodeName.c_str());
    ros::NodeHandle node;

//-- Topicos susbcritos y publicados
    ros::Subscriber subInfo1=node.subscribe("/vrep/info",100,infoCallback);
    ros::Subscriber subInfo2=node.subscribe("Reloj",100,relojCallback);
    ros::Subscriber subInfo3=node.subscribe("UbicacionRobot",100,ubicacionRobCallback);
//-- Manda topico especifico para cada pata
    chatter_pub1=node.advertise<camina9::DatosTrayectoriaPata>("datosTrayectoria", 100);
//-- Clientes y Servicios
    client_Planificador = node.serviceClient<camina9::PlanificadorParametros>("PlanificadorPisada");
//-- Log de datos
    std::string fileName("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina9/datos/SalidaDatos");
    std::string texto(".txt");
    fileName+=texto;
    fp1 = fopen(fileName.c_str(),"w+");

//-- Patas de [0-5]
    int cuenta_T1=0, cuenta_T2=0;
    for(int k=0;k<Npatas;k++) {
        if(tripode[k]==1){
            Tripode1[cuenta_T1]=k;
            cuenta_T1++;
        } else {
            Tripode2[cuenta_T2]=k;
            cuenta_T2++;
        }
    }
//    ROS_INFO("Nodo1: Tripode1[%d,%d,%d] - Tripode2[%d,%d,%d]",Tripode1[0]+1,Tripode1[1]+1,Tripode1[2]+1,Tripode2[0]+1,Tripode2[1]+1,Tripode2[2]+1);
//-- Datos de envio
    for(int i=0;i<2;i++){
        datosTrayectoriaPata.T_apoyo.push_back(0);
        datosTrayectoriaPata.t_Trayectoria.push_back(0);
        datosTrayectoriaPata.lambda_Apoyo.push_back(0);
        datosTrayectoriaPata.lambda_Transferencia.push_back(0);
        datosTrayectoriaPata.alfa.push_back(0);
        datosTrayectoriaPata.desfasaje_t.push_back(0);
        datosTrayectoriaPata.vector_estados.push_back(0);
        datosTrayectoriaPata.cambio_estado.push_back(0);
    }
    for(int k=0;k<Npatas;k++) {
        datosTrayectoriaPata.correccion_di.push_back(0);
        datosTrayectoriaPata.correccion_ID.push_back(0);
        datosTrayectoriaPata.posicionActualSistemaPata_x.push_back(0);
        datosTrayectoriaPata.posicionActualSistemaPata_y.push_back(0);
        datosTrayectoriaPata.posicionActualSistemaPata_z.push_back(0);
    }
//-- Tripode 1
    datosTrayectoriaPata.T_apoyo[T1-1]=T;
    datosTrayectoriaPata.lambda_Apoyo[T1-1]=lambda_Apoyo;
    datosTrayectoriaPata.lambda_Transferencia[T1-1]=lambda_Transferencia;
    datosTrayectoriaPata.alfa[T1-1]=alfa;
    datosTrayectoriaPata.desfasaje_t[T1-1]=desfasaje_t_T1;
    datosTrayectoriaPata.vector_estados[T1-1]=0;
    datosTrayectoriaPata.cambio_estado[T1-1]=0;
//-- Tripode 2
    datosTrayectoriaPata.T_apoyo[T2-1]=T;
    datosTrayectoriaPata.lambda_Apoyo[T2-1]=lambda_Apoyo;
    datosTrayectoriaPata.lambda_Transferencia[T2-1]=lambda_Transferencia;
    datosTrayectoriaPata.alfa[T2-1]=alfa;
    datosTrayectoriaPata.desfasaje_t[T2-1]=desfasaje_t_T2;
    datosTrayectoriaPata.vector_estados[T2-1]=1;
    datosTrayectoriaPata.cambio_estado[T2-1]=0;

//-- Prepara variables para calculos de trayectoria de PATA
    //delta_t = 1/divisionTrayectoriaPata;
    divisionTiempo = T/divisionTrayectoriaPata;
    f=1/divisionTiempo;
    /* La velocidad de envío de los datos se encarga de darme el tiempo total de trayectoria deseado */
    /* Velocidad de transmision */
    ros::Rate loop_rate(f);  //Frecuencia [Hz]
//-- Delay inicial para esperar inicio de todos los nodos
//    for(i=0;i<10;i++) loop_rate.sleep();

    modificacion_T = T;
    modificacion_lambda = lambda_Transferencia;

    while (ros::ok() && simulationRunning){
        ros::spinOnce();
//        loop_rate.sleep();
    }
        ROS_INFO("Adios1!");
//        fclose(fp1);
        ros::shutdown();
        return 0;
}


/* Funciones */

bool CambioDeEstado(){
    bool cambio = false;
//--- Apoyo de Tripode 1
    if ((pataApoyo[Tripode1[0]]==1 and pataApoyo[Tripode1[1]]==1 and pataApoyo[Tripode1[2]]==1) and FinApoyo_T1) {
//            ROS_INFO("Nodo1: apoyo Tripode1[%d,%d,%d]",pataApoyo[Tripode1[0]],pataApoyo[Tripode1[1]],pataApoyo[Tripode1[2]]);
        InicioApoyo_T1=true;
        FinApoyo_T1=false;
    }
    if (pataApoyo[Tripode1[0]]==0 and pataApoyo[Tripode1[1]]==0 and pataApoyo[Tripode1[2]]==0) {
        FinApoyo_T1=true;
    }

    if (InicioApoyo_T1){
        InicioApoyo_T1 = false;
        Tripode = T1;
        yCuerpo_T1_1 = coordenadaCuerpo_y;
        timerT1_1 = boost::posix_time::microsec_clock::local_time();
        cambio = true;
    }
//--- Transferencia de Tripode 1
    if ((pataApoyo[Tripode1[0]]==0 and pataApoyo[Tripode1[1]]==0 and pataApoyo[Tripode1[2]]==0) and FinTransf_T1) {
        InicioTransf_T1=true;
        FinTransf_T1=false;
    }
    if (pataApoyo[Tripode1[0]]==1 and pataApoyo[Tripode1[1]]==1 and pataApoyo[Tripode1[2]]==1) {
        FinTransf_T1=true;
    }

    if (InicioTransf_T1){
        InicioTransf_T1 = false;
        yCuerpo_T1_2 = coordenadaCuerpo_y;
        timerT1_2 = boost::posix_time::microsec_clock::local_time();
    }
//--- Apoyo de Tripode 2
    if ((pataApoyo[Tripode2[0]]==1 and pataApoyo[Tripode2[1]]==1 and pataApoyo[Tripode2[2]]==1) and FinApoyo_T2) {
//            ROS_INFO("Nodo1: apoyo Tripode2[%d,%d,%d]",pataApoyo[Tripode2[0]],pataApoyo[Tripode2[1]],pataApoyo[Tripode2[2]]);
        InicioApoyo_T2=true;
        FinApoyo_T2=false;
    }
    if (pataApoyo[Tripode2[0]]==0 and pataApoyo[Tripode2[1]]==0 and pataApoyo[Tripode2[2]]==0) {
        FinApoyo_T2=true;
    }

    if (InicioApoyo_T2){
        InicioApoyo_T2 = false;
        Tripode = T2;
        yCuerpo_T2_1 = coordenadaCuerpo_y;
        timerT2_1 = boost::posix_time::microsec_clock::local_time();
        cambio = true;
    }
//--- Transferencia de Tripode 2
    if ((pataApoyo[Tripode2[0]]==0 and pataApoyo[Tripode2[1]]==0 and pataApoyo[Tripode2[2]]==0) and FinTransf_T2) {
        InicioTransf_T2=true;
        FinTransf_T2=false;
    }
    if (pataApoyo[Tripode2[0]]==1 and pataApoyo[Tripode2[1]]==1 and pataApoyo[Tripode2[2]]==1) {
        FinTransf_T2=true;
    }

    if (InicioTransf_T2){
        InicioTransf_T2 = false;
        yCuerpo_T2_2 = coordenadaCuerpo_y;
        timerT2_2 = boost::posix_time::microsec_clock::local_time();
    }

    return cambio;
}
