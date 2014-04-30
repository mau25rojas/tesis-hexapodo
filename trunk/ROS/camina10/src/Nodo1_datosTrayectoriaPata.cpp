#include <ros/ros.h>
#include <std_msgs/String.h>
#include <math.h>
#include <time.h>
#include "boost/date_time/posix_time/posix_time.hpp"
//Librerias propias usadas
#include "constantes.hpp"
#include "camina10/v_repConst.h"
// Used data structures:
#include "camina10/DatosTrayectoriaPata.h"
#include "camina10/PlanificadorParametros.h"
#include "camina10/SenalesCambios.h"
#include "camina10/UbicacionRobot.h"
// Used API services:
#include "vrep_common/VrepInfo.h"
//Clientes y Servicios
ros::ServiceClient client_Planificador;
camina10::PlanificadorParametros srv_Planificador;

// variables Globales
bool simulationRunning=true;
bool sensorTrigger=false, Inicio=true;
bool InicioApoyo[Npatas]=[false,false,false,false,false,false], FinApoyo[Npatas]=[false,false,false,false,false,false];
bool InicioTransf[Npatas]=[false,false,false,false,false,false], FinTransf[Npatas]=[false,false,false,false,false,false];
float simulationTime=0.0f;
float divisionTrayectoriaPata[Npatas], T[Npatas], divisionTiempo=0.0, desfasaje_t[Npatas]=0.0, beta=0.0;
float T[Npatas], delta_t[Npatas], modificacion_T_apoyo = 0.0, modificacion_lambda =0.0;
float xCuerpo_1=0.0, xCuerpo_2=0.0, yCuerpo_1=0.0, yCuerpo_2=0.0;
float coordenadaCuerpo_y=0.0, coordenadaCuerpo_x=0.0, velocidadCuerpo_y=0.0, velocidadCuerpo_x=0.0, mod_velocidadCuerpo=0.0;
int pataApoyo[Npatas];
int cuenta=0, PasosIni=0;
FILE *fp1;
boost::posix_time::ptime timer_1,timer_2;
camina10::DatosTrayectoriaPata datosTrayectoriaPata;
ros::Publisher chatter_pub1,chatter_pub2;

// Funciones
int CambioDeEstado();
float VelocidadCuerpo(boost::posix_time::ptime t1, boost::posix_time::ptime t2, float x1, float x2, float y1, float y2);
// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}

void ubicacionRobCallback(camina10::UbicacionRobot msgUbicacionRobot)
{
     coordenadaCuerpo_x = msgUbicacionRobot.coordenadaCuerpo_x;
     coordenadaCuerpo_y = msgUbicacionRobot.coordenadaCuerpo_y;
     for(int k=0; k<Npatas;k++) {
        pataApoyo[k] = msgUbicacionRobot.pataApoyo[k];
    }
}

void relojCallback(camina10::SenalesCambios msgSenal)
{
    int nPata=0;

    if (!msgSenal.Stop){

        //-- Hay llamada al planificador?
        nPata = CambioDeEstado();
        ParametrosVelocidad();

        if (Inicio){
            cuenta++;
            if (cuenta==PasosIni*divisionTrayectoriaPata){
                Inicio=false;
            } else {

                for(int k=0;k<Npatas;k++) datosTrayectoriaPata.t_Trayectoria[k]=delta_t[k];
                chatter_pub1.publish(datosTrayectoriaPata);

                for(int k=0;k<Npatas;k++){
                    delta_t[k] = delta_t[k] + T[k]/divisionTrayectoriaPata[k];

                    if (fabs(delta_t[k]-T[k])<=(T[k]/divisionTrayectoriaPata[k])) {
                        delta_t[k] = 0.0;
                        datosTrayectoriaPata.vector_estados[k]=0;
                    }

                    if (fabs(delta_t[k]-T[k])<=(T[k]/divisionTrayectoriaPata[k])) {
                        delta_t[k] = 0.0;
                        datosTrayectoriaPata.vector_estados[k]=0;
                    }
                }
            }
        } else {
            //-------------------------------
            if(nPata!=0){
                nPata = 0;
            //-- reinicio cuenta para iniciar apoyo
                delta_t[nPata] = 0.0;
                for(int k=0;k<Npatas;k++){
                    datosTrayectoriaPata.correccion_ID[k]=-1;
                    datosTrayectoriaPata.correccion_x[k]=0.0;
                    datosTrayectoriaPata.correccion_y[k]=0.0;
                }
                mod_velocidadCuerpo = VelocidadCuerpo(timer_1,timer_2,xCuerpo_1,xCuerpo_2,yCuerpo_1,yCuerpo_2);

                srv_Planificador.request.T = T[nPata];
                srv_Planificador.request.lambda = lambda[nPata];
                srv_Planificador.request.mod_velApoyo = mod_velocidadCuerpo;
                if (client_Planificador.call(srv_Planificador)){
                    modificacion_lambda = srv_Planificador.response.modificacion_lambda;
                    modificacion_T_apoyo = srv_Planificador.response.modificacion_T_apoyo_apoyo;
                    datosTrayectoriaPata.correccion_ID[nPata] = srv_Planificador.response.correccion_ID;
                    datosTrayectoriaPata.correccion_x[nPata] = srv_Planificador.response.correccion_x;
                    datosTrayectoriaPata.correccion_y[nPata] = srv_Planificador.response.correccion_y;
                    ROS_INFO("Nodo1::t_sim=%.3f, lambda_c=%.3f,t_c=%.3f",simulationTime,modificacion_lambda,modificacion_T_apoyo);
                } else {
                    ROS_ERROR("Nodo1::servicio de Planificacion no funciona");
                    ROS_ERROR("result=%d", srv_Planificador.response.result);
                }

//                /////////////////////PRUEBAS///////////////////////////////////
//                for(int k=0;k<Npatas;k++) {
//                    datosTrayectoriaPata.correccion_ID[k]=-1;
//                    datosTrayectoriaPata.correccion_x[k]=0;
//                    datosTrayectoriaPata.correccion_y[k]=0;
//                }
//                ///////////////////////////////////////////////////////////////
//                datosTrayectoriaPata.correccion_ID[2]=1;
//                datosTrayectoriaPata.correccion_x[2]=0.01;
//                datosTrayectoriaPata.correccion_y[2]=0.01;


                T[nPata] = modificacion_T_apoyo/beta;
                divisionTrayectoriaPata[nPata] = T[nPata]/divisionTiempo;
                //-- datos a enviar
                if (Tripode==T1){
                //-- Correccion en T1 significa inicio de apoyo en T1 y transferencia en T2
                    datosTrayectoriaPata.vector_estados[T1-1]=0;
                    datosTrayectoriaPata.vector_estados[T2-1]=1;
                    datosTrayectoriaPata.cambio_estado[T1-1]=1;
                    datosTrayectoriaPata.cambio_estado[T2-1]=1;
                //-- El tiempo de apoyo de T1 es tiempo de trans de T2
                //....La correccion obtenida se aplica en lambda_trans T2
                    datosTrayectoriaPata.t_apoyo[T1-1]=datosTrayectoriaPata.t_apoyo[T2-1]=modificacion_T_apoyo;
                    datosTrayectoriaPata.lambda[T2-1]=modificacion_lambda;
                } else {
                //-- Correccion en T2 significa inicio de apoyo en T2 y transferencia en T1
                    datosTrayectoriaPata.vector_estados[T1-1]=1;
                    datosTrayectoriaPata.vector_estados[T2-1]=0;
                    datosTrayectoriaPata.cambio_estado[T1-1]=1;
                    datosTrayectoriaPata.cambio_estado[T2-1]=1;
                //-- El tiempo de apoyo de T2 es tiempo de trans de T1
                //....La correccion obtenida se aplica en lambda_trans T1
                    datosTrayectoriaPata.t_apoyo[T1-1]=datosTrayectoriaPata.t_apoyo[T2-1]=modificacion_T_apoyo;
                    datosTrayectoriaPata.lambda[T1-1]=modificacion_lambda;
                }
            }// Fin llamada plan

            datosTrayectoriaPata.t_Trayectoria[T1-1]=datosTrayectoriaPata.t_Trayectoria[T2-1]=delta_t;
//            fprintf(fp1,"%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",modificacion_T_apoyo,datosTrayectoriaPata.t_Trayectoria[T1-1],datosTrayectoriaPata.lambda[T1-1],datosTrayectoriaPata.lambda[T1-1],datosTrayectoriaPata.lambda[T2-1], datosTrayectoriaPata.lambda[T2-1]);
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
  float lambda=0.0, alfa=0.0, f=0.0, vector_estados, T_ini;
  int Narg=0;

  Narg=14;
	if (argc>=Narg)
	{
        PasosIni=atoi(argv[1]);
        T_ini=atof(argv[2]); // Periodo de trayectoria [seg]
		divisionTrayectoriaPata=atof(argv[3]);  //N puntos
		beta=atof(argv[4]);
		lambda=atof(argv[5]);
		alfa=atof(argv[6])*pi/180;
        for(int k=0;k<Npatas;k++) desfasaje_t[k]=atof(argv[7]);
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
    chatter_pub1=node.advertise<camina10::DatosTrayectoriaPata>("datosTrayectoria", 100);
//-- Clientes y Servicios
    client_Planificador = node.serviceClient<camina10::PlanificadorParametros>("PlanificadorPisada");
//-- Log de datos
    std::string fileName("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina10/datos/SalidaDatos");
    std::string texto(".txt");
    fileName+=texto;
    fp1 = fopen(fileName.c_str(),"w+");

//-- Inicializo variables
    for(int k=0;k<Npatas;k++) {

        T[k] = T_ini;
        delta_t[k] = 0.0;
        if(desfasaje_t[k]>beta){
            vector_estados[k] = 1;
        } else {
            vector_estados[k] = 0;
        }
    }

//-- Datos de envio
    for(int k=0;k<Npatas;k++) {
        datosTrayectoriaPata.t_Trayectoria.push_back(0);
        datosTrayectoriaPata.lambda.push_back(lambda);
        datosTrayectoriaPata.alfa.push_back(alfa);
        datosTrayectoriaPata.desfasaje_t.push_back(desfasaje_t[k]);
        datosTrayectoriaPata.vector_estados.push_back(vector_estados[k]);
        datosTrayectoriaPata.correccion_x.push_back(0);
        datosTrayectoriaPata.correccion_y.push_back(0);
        datosTrayectoriaPata.correccion_ID.push_back(0);
    }

//-- Prepara variables para calculos de trayectoria de PATA
    //delta_t = 1/divisionTrayectoriaPata;
    divisionTiempo = T/divisionTrayectoriaPata;
    f=1/divisionTiempo;
    /* La velocidad de envío de los datos se encarga de darme el tiempo total de trayectoria deseado */
    /* Velocidad de transmision */
    ros::Rate loop_rate(f);  //Frecuencia [Hz]
//-- Delay inicial para esperar inicio de todos los nodos
//    for(i=0;i<10;i++) loop_rate.sleep();

    modificacion_T_apoyo = T;
    modificacion_lambda = lambda;

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

int CambioDeEstado(int nPata){
    int PataCambio = 0;
//--- Apoyo de Pata
    if (pataApoyo[nPata]==1 and FinApoyo[nPata]) {
        InicioApoyo[nPata]=true;
        FinApoyo[nPata]=false;
    }
    if (pataApoyo[nPata]==0) {
        FinApoyo[nPata]=true;
    }

    if (InicioApoyo[nPata]){
        InicioApoyo[nPata] = false;
        PataCambio = nPata+1;
    }
//--- Transferencia Pata
    if ((pataApoyo[nPata]==0 and FinTransf[nPata]) {
        InicioTransf[nPata]=true;
        FinTransf[nPata]=false;
    }
    if (pataApoyo[nPata]==1) {
        FinTransf[nPata]=true;
    }

    if (InicioTransf[nPata]){
        InicioTransf[nPata] = false;
    }

    return PataCambio;
}

/* Toma de muestras de tiempo y posicion para calculo de velocidad
.. se toma de muestra la pata1*/

void ParametrosVelocidad(){
//--- Apoyo de Pata 1
    if (InicioApoyo[0]){
        xCuerpo_1 = coordenadaCuerpo_x;
        yCuerpo_1 = coordenadaCuerpo_y;
        timer_1 = boost::posix_time::microsec_clock::local_time();
    }
//--- Transferencia de Pata 1
    if (InicioTransf[0]){
        xCuerpo_2 = coordenadaCuerpo_x;
        yCuerpo_2 = coordenadaCuerpo_y;
        timer_2 = boost::posix_time::microsec_clock::local_time();
    }
}

/* Retorna el modulo de la velocidad del cuerpo, segun el tiempo y distancia del cuerpo
.. en apoyo y transferencia*/
float VelocidadCuerpo(boost::posix_time::ptime t1, boost::posix_time::ptime t2, float x1, float x2, float y1, float y2){
    float delta_x=0.0, delta_y=0.0, tiempo_ahora=0.0;
    boost::posix_time::time_duration diff_t;

    delta_x = fabs(x1-x2);
    delta_y = fabs(y1-y2);
    diff_t = t1 - t2;
    tiempo_ahora = (float) fabs(diff_t.total_milliseconds())/1000;
    velocidadCuerpo_x = delta_x/tiempo_ahora;
    velocidadCuerpo_y = delta_y/tiempo_ahora;
    return (sqrt(velocidadCuerpo_x*velocidadCuerpo_x + velocidadCuerpo_y*velocidadCuerpo_y));
}
