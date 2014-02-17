#include "ros/ros.h"
#include "std_msgs/String.h"
#include "math.h"
//Librerias propias usadas
#include "constantes.hpp"
#include "camina7/v_repConst.h"
// Used data structures:
#include "camina7/DatosTrayectoriaPata.h"
#include "camina7/PlanificadorParametros.h"
#include "camina7/SenalesCambios.h"
#include "camina7/UbicacionRobot.h"
// Used API services:
#include "vrep_common/VrepInfo.h"
//Clientes y Servicios
ros::ServiceClient client_Planificador;
camina7::PlanificadorParametros srv_Planificador;

// variables Globales
bool simulationRunning=true;
bool sensorTrigger=false;
bool InicioApoyo_T1=false, FinApoyo_T1=false, InicioApoyo_T2=false, FinApoyo_T2=false;
camina7::DatosTrayectoriaPata datosTrayectoriaPata;
float simulationTime=0.0f;
float divisionTrayectoriaPata=0.0, T=0.0, lambda_Transferencia=0.0, divisionTiempo=0.0, desfasaje_t_T1=0.0,desfasaje_t_T2=0.0, beta=0.0;
float modificacion_T = 0.0, modificacion_lambda =0.0;
float delta_t=0.0, t_aux_T1=0.0,t_aux_T2=0.0;
int pataApoyo[Npatas], tripode[Npatas], Tripode1[Npatas/2], Tripode2[Npatas/2];
int Tripode=0;
FILE *fp1;
ros::Publisher chatter_pub1,chatter_pub2;

// Funciones
bool Apoyo_T1();
bool Apoyo_T2();
// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}

void ubicacionRobCallback(camina7::UbicacionRobot msgUbicacionRobot)
{
     for(int k=0; k<Npatas;k++) {
        pataApoyo[k] = msgUbicacionRobot.pataApoyo[k];
    }
}

void relojCallback(camina7::SenalesCambios msgSenal)
{
    bool llamadaPlan=false;

    if (!msgSenal.Stop){

        //------- T1 --------------------
        t_aux_T1=delta_t+desfasaje_t_T1*T;
        t_aux_T1=fmod(t_aux_T1,T);
        llamadaPlan = Apoyo_T1();
        //-------------------------------
        //------- T2 --------------------
        t_aux_T2=delta_t+desfasaje_t_T2*T;
        t_aux_T2=fmod(t_aux_T2,T);
        llamadaPlan = Apoyo_T2();
        //-------------------------------
        if(llamadaPlan){
            ROS_INFO("Llama a plan tripode[%d]",Tripode);
            llamadaPlan = false;
            srv_Planificador.request.Tripode = Tripode;
            srv_Planificador.request.T = T;
            if (client_Planificador.call(srv_Planificador)){
                modificacion_lambda = srv_Planificador.response.modificacion_lambda;
                modificacion_T = srv_Planificador.response.modificacion_T;
                ROS_INFO("Nodo1::T[%d]: t_sim=%.3f, t_T1=%.3f, t_T2=%.3f, lambda_c=%.3f,t_c=%.3f",Tripode,simulationTime,t_aux_T1,t_aux_T2,modificacion_lambda,modificacion_T);

            } else {
                ROS_ERROR("Nodo1::T[%d] servicio de Planificacion no funciona",Tripode);
                ROS_ERROR("result=%d", srv_Planificador.response.result);
            }

            T = modificacion_T;
            divisionTrayectoriaPata = T/divisionTiempo;
            //-- datos a enviar
            datosTrayectoriaPata.T = modificacion_T;
            datosTrayectoriaPata.lambda_Transferencia[0]=datosTrayectoriaPata.lambda_Transferencia[1]=modificacion_lambda;
            datosTrayectoriaPata.lambda_Apoyo[0]=datosTrayectoriaPata.lambda_Apoyo[1]=modificacion_lambda;
        }

        datosTrayectoriaPata.t_Trayectoria[0] = t_aux_T1;
        datosTrayectoriaPata.t_Trayectoria[1] = t_aux_T2;

        if (0<=t_aux_T1 and t_aux_T1<beta*T){
            datosTrayectoriaPata.vector_estados[0]=0;
        }else{
            datosTrayectoriaPata.vector_estados[0]=1;
        }
        if (0<=t_aux_T2 and t_aux_T2<beta*T){
            datosTrayectoriaPata.vector_estados[1]=0;
        }else{
            datosTrayectoriaPata.vector_estados[1]=1;
        }

        chatter_pub1.publish(datosTrayectoriaPata);
//        delta_t = delta_t + T/divisionTrayectoriaPata;
        if (fabs(delta_t-T)<=(T/divisionTrayectoriaPata)) {
//            ROS_INFO("reinicio trayectoria[%d]",Tripode);
            delta_t = 0.0;
//            t_aux = 0.0;
        }

        delta_t = delta_t + T/divisionTrayectoriaPata;
    }
}

int main(int argc, char **argv)
{
  float lambda_Apoyo=0.0, alfa=0.0, f=0.0;
  int i=0, Narg=0;

  Narg=14;
	if (argc>=Narg)
	{
        T=atof(argv[1]); // Periodo de trayectoria [seg]
		divisionTrayectoriaPata=atof(argv[2]);  //N puntos
		beta=atof(argv[3]);
		lambda_Apoyo=atof(argv[4]);
		lambda_Transferencia=atof(argv[5]);
		alfa=atof(argv[6])*pi/180;
        desfasaje_t_T1=atof(argv[7]);
        desfasaje_t_T2=atof(argv[8]);
        for(int k=0;k<Npatas;k++) tripode[k] = atoi(argv[9+k]);
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
    chatter_pub1=node.advertise<camina7::DatosTrayectoriaPata>("datosTrayectoria", 100);
//-- Clientes y Servicios
    client_Planificador = node.serviceClient<camina7::PlanificadorParametros>("PlanificadorPisada");
//-- Log de datos
//    std::string fileName("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina7/datos/SalidaDatos");
//    std::string texto(".txt");
//    fileName+=Id;
//    fileName+=texto;
//    fp1 = fopen(fileName.c_str(),"w+");

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
    ROS_INFO("Nodo1: Tripode1[%d,%d,%d] - Tripode2[%d,%d,%d]",Tripode1[0]+1,Tripode1[1]+1,Tripode1[2]+1,Tripode2[0]+1,Tripode2[1]+1,Tripode2[2]+1);
//-- Datos de envio
    for(int i=0;i<2;i++){
        datosTrayectoriaPata.t_Trayectoria.push_back(0);
        datosTrayectoriaPata.lambda_Apoyo.push_back(0);
        datosTrayectoriaPata.lambda_Transferencia.push_back(0);
        datosTrayectoriaPata.alfa.push_back(0);
        datosTrayectoriaPata.desfasaje_t.push_back(0);
        datosTrayectoriaPata.vector_estados.push_back(0);
    }
    datosTrayectoriaPata.T = T;
//-- Tripode 1
    datosTrayectoriaPata.lambda_Apoyo[0]=lambda_Apoyo;
    datosTrayectoriaPata.lambda_Transferencia[0]=lambda_Transferencia;
    datosTrayectoriaPata.alfa[0]=alfa;
    datosTrayectoriaPata.desfasaje_t[0]=desfasaje_t_T1;
    datosTrayectoriaPata.vector_estados[0]=0;
//-- Tripode 2
    datosTrayectoriaPata.lambda_Apoyo[1]=lambda_Apoyo;
    datosTrayectoriaPata.lambda_Transferencia[1]=lambda_Transferencia;
    datosTrayectoriaPata.alfa[1]=alfa;
    datosTrayectoriaPata.desfasaje_t[1]=desfasaje_t_T2;
    datosTrayectoriaPata.vector_estados[1]=1;

//-- Prepara variables para calculos de trayectoria de PATA
    //delta_t = 1/divisionTrayectoriaPata;
    divisionTiempo = T/divisionTrayectoriaPata;
    f=1/divisionTiempo;
    /* La velocidad de envío de los datos se encarga de darme el tiempo total de trayectoria deseado */
    /* Velocidad de transmision */
    ros::Rate loop_rate(f);  //Frecuencia [Hz]
//-- Delay inicial para esperar inicio de todos los nodos
    for(i=0;i<10;i++) loop_rate.sleep();

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

bool Apoyo_T1(){
    bool apoyo = false;
    if ((pataApoyo[Tripode1[0]]==1 and pataApoyo[Tripode1[1]]==1 and pataApoyo[Tripode1[2]]==1) and FinApoyo_T1) {
            ROS_INFO("Nodo1: apoyo Tripode1[%d,%d,%d]",pataApoyo[Tripode1[0]],pataApoyo[Tripode1[1]],pataApoyo[Tripode1[2]]);
        InicioApoyo_T1=true;
        FinApoyo_T1=false;
    }
    if (pataApoyo[Tripode1[0]]==0 and pataApoyo[Tripode1[1]]==0 and pataApoyo[Tripode1[2]]==0) {
        FinApoyo_T1=true;
    }

    if (InicioApoyo_T1){
        InicioApoyo_T1 = false;
        Tripode = T1;
        apoyo = true;
    }
    return apoyo;
}

bool Apoyo_T2(){
    bool apoyo = false;
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
        apoyo = true;
    }
    return apoyo;
}
