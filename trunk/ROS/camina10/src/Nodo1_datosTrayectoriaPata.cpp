#include <ros/ros.h>
#include <std_msgs/String.h>
#include <math.h>
#include <time.h>
#include "boost/date_time/posix_time/posix_time.hpp"
//Librerias propias usadas
#include "constantes.hpp"
#include "camina10/v_repConst.h"
#include "../../Convexhull/vector3d.hpp"
#include "../../Convexhull/convexhull.cpp"
#include "../../Convexhull/analisis.cpp"
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
bool InicioApoyo[Npatas]={false,false,false,false,false,false}, FinApoyo[Npatas]={false,false,false,false,false,false};
bool InicioTransf[Npatas]={false,false,false,false,false,false}, FinTransf[Npatas]={false,false,false,false,false,false};
float simulationTime=0.0f;
float divisionTrayectoriaPata[Npatas], divisionTiempo=0.0, desfasaje_t[Npatas], beta=0.0, phi[Npatas],alfa=0.0;
float T[Npatas], T_contador[Npatas], T_apoyo[Npatas],T_transf[Npatas], contadores[Npatas],delta_t[Npatas], modificacion_T_apoyo = 0.0, modificacion_lambda =0.0;
float xCuerpo_1=0.0, xCuerpo_2=0.0, yCuerpo_1=0.0, yCuerpo_2=0.0, mod_velocidadCuerpo=0.0;
int pataApoyo[Npatas],divisionTrayectoriaPata_ini;
int cuenta=0, PasosIni=0;
FILE *fp1;
punto3d coordenadaCuerpo, velocidadCuerpo, posicionActualPataSistemaPata[Npatas],posCuerpo_1, posCuerpo_2;
punto3d Offset;
boost::posix_time::ptime timer_1,timer_2;
camina10::DatosTrayectoriaPata datosTrayectoriaPata;
ros::Publisher chatter_pub1,chatter_pub2;

// Funciones
bool CambioDeEstado_Apoyo(int nPata);
bool CambioDeEstado_Transf(int nPata);
void ParametrosVelocidad();
float VelocidadCuerpo(boost::posix_time::ptime t1, boost::posix_time::ptime t2, punto3d Pos1, punto3d Pos2);
bool LlegadaFinEDT(int nPata);
punto3d TransformacionHomogenea(punto3d Punto_in, punto3d L_traslacion, float ang_rotacion);

// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}

void ubicacionRobCallback(camina10::UbicacionRobot msgUbicacionRobot)
{
     coordenadaCuerpo.x = msgUbicacionRobot.coordenadaCuerpo_x;
     coordenadaCuerpo.y = msgUbicacionRobot.coordenadaCuerpo_y;
     for(int k=0; k<Npatas;k++) {
        pataApoyo[k] = msgUbicacionRobot.pataApoyo[k];
        posicionActualPataSistemaPata[k].x = msgUbicacionRobot.coordenadaPataSistemaPata_x[k];
        posicionActualPataSistemaPata[k].y = msgUbicacionRobot.coordenadaPataSistemaPata_y[k];
        posicionActualPataSistemaPata[k].z = msgUbicacionRobot.coordenadaPataSistemaPata_z[k];
    }
}

void relojCallback(camina10::SenalesCambios msgSenal)
{
    bool cambio_a_Apoyo=false,llegada_FEDT=false;
    float lambda = 0.0;

    if (!msgSenal.Stop){

        ParametrosVelocidad();
        if (Inicio){
            cuenta++;
            if (cuenta==PasosIni*divisionTrayectoriaPata_ini){
                Inicio=false;
            } else {

                for(int k=0;k<Npatas;k++) {
                    datosTrayectoriaPata.t_Trayectoria[k]=delta_t[k];
                    datosTrayectoriaPata.T[k]=T_transf[k];
                }
                chatter_pub1.publish(datosTrayectoriaPata);

                for(int k=0;k<Npatas;k++){
                    datosTrayectoriaPata.cambio_estado[k]=0;
                    fprintf(fp1,"%.3f\t",delta_t[k]);

                    contadores[k] = contadores[k] + T[k]/divisionTrayectoriaPata[k];

                    if (contadores[k]>=beta*T[k]) {
                        delta_t[k] = contadores[k]-T_apoyo[k];
                        datosTrayectoriaPata.vector_estados[k]=1;
                    } else {
                        delta_t[k] = contadores[k];
                        datosTrayectoriaPata.vector_estados[k]=0;
                    }
                    if (fabs(contadores[k]-T[k])<=(T[k]/divisionTrayectoriaPata[k])){
                        contadores[k] = 0.0;
                    }
                }
                fprintf(fp1,"\n");
            }
        } else {
            for(int k=0;k<Npatas;k++){
                datosTrayectoriaPata.cambio_estado[k]=0;
            //-- Hay llamada al planificador?
                cambio_a_Apoyo = CambioDeEstado_Apoyo(k);
                if(cambio_a_Apoyo){
                    cambio_a_Apoyo = false;
                //-- reinicio cuenta para iniciar apoyo
                    contadores[k] = 0.0;
                    datosTrayectoriaPata.cambio_estado[k]=1;
                    datosTrayectoriaPata.vector_estados[k]=0;
                }
                llegada_FEDT = LlegadaFinEDT(k);
                if(llegada_FEDT){
                    llegada_FEDT = false;
                    ROS_INFO("------Inicia Transferencia pata[%d]",k);
                    datosTrayectoriaPata.cambio_estado[k]=1;
                    datosTrayectoriaPata.vector_estados[k]=1;
                    contadores[k] = contadores[k] + 3*T[k]/divisionTrayectoriaPata[k];
                }
//                //-------------------------------
//                if(cambio_a_Apoyo){
//                    cambio_a_Apoyo = false;
//                //-- reinicio cuenta para iniciar apoyo
//                    contadores[k] = 0.0;
//                    datosTrayectoriaPata.correccion_ID[k]=-1;
//                    datosTrayectoriaPata.correccion_x[k]=0.0;
//                    datosTrayectoriaPata.correccion_y[k]=0.0;
//                    mod_velocidadCuerpo = VelocidadCuerpo(timer_1,timer_2,posCuerpo_1,posCuerpo_2);
//
//                    srv_Planificador.request.T = T[k];
//                    srv_Planificador.request.lambda = lambda;
//                    srv_Planificador.request.mod_velApoyo = mod_velocidadCuerpo;
//                    if (client_Planificador.call(srv_Planificador)){
//                        modificacion_lambda = srv_Planificador.response.modificacion_lambda;
//                        modificacion_T_apoyo = srv_Planificador.response.modificacion_T_apoyo;
//                        datosTrayectoriaPata.correccion_ID[k] = srv_Planificador.response.correccion_ID;
//                        datosTrayectoriaPata.correccion_x[k] = srv_Planificador.response.correccion_x;
//                        datosTrayectoriaPata.correccion_y[k] = srv_Planificador.response.correccion_y;
//                        ROS_INFO("Nodo1::t_sim=%.3f, lambda_c=%.3f,t_c=%.3f",simulationTime,modificacion_lambda,modificacion_T_apoyo);
//                    } else {
//                        ROS_ERROR("Nodo1::servicio de Planificacion no funciona");
//                        ROS_ERROR("result=%d", srv_Planificador.response.result);
//                    }
//
////                /////////////////////PRUEBAS///////////////////////////////////
////                for(int k=0;k<Npatas;k++) {
////                    datosTrayectoriaPata.correccion_ID[k]=-1;
////                    datosTrayectoriaPata.correccion_x[k]=0;
////                    datosTrayectoriaPata.correccion_y[k]=0;
////                }
////                ///////////////////////////////////////////////////////////////
////                datosTrayectoriaPata.correccion_ID[2]=1;
////                datosTrayectoriaPata.correccion_x[2]=0.01;
////                datosTrayectoriaPata.correccion_y[2]=0.01;
//
//
//                    T_apoyo[k] = modificacion_T_apoyo;
//                    T[k] = T_apoyo[k]/beta;
//                    divisionTrayectoriaPata[k] = T[k]/divisionTiempo;
//                    datosTrayectoriaPata.lambda[k]=modificacion_lambda;
//                }// Fin cambio de estado

//                if (fabs(contadores[k]-beta*T[k])<(T[k]/divisionTrayectoriaPata[k])) {
//                    ROS_INFO("Inicia Transferencia pata[%d]",k);
////                    contadores[k] = beta*T[k];
//                    datosTrayectoriaPata.cambio_estado[k]=1;
//                    datosTrayectoriaPata.vector_estados[k]=1;
//                }

                if (datosTrayectoriaPata.vector_estados[k]==1) {
                    delta_t[k] = contadores[k]-T_apoyo[k];
                } else {
                    delta_t[k] = contadores[k];
                }

                datosTrayectoriaPata.T[k]=T_transf[k];
                datosTrayectoriaPata.t_Trayectoria[k]=delta_t[k];

        //        contadores = contadores + T/divisionTrayectoriaPata;
                if (fabs(contadores[k]-T[k])<=(T[k]/divisionTrayectoriaPata[k])){
                    contadores[k] = contadores[k];
    //                ROS_INFO("Esperando apoyo, contadores=%.4f",contadores);
                } else if(fabs(contadores[k]-beta*T[k])<(T[k]/divisionTrayectoriaPata[k])){
                    contadores[k] = beta*T[k];
                    ROS_INFO("espera transferencia pata[%d]",k);
                } else {
                    contadores[k] = contadores[k] + T[k]/divisionTrayectoriaPata[k];
                }
            }// fin del for
                chatter_pub1.publish(datosTrayectoriaPata);
        }
    }//-- fin is !Stop
} //-- fin de callback

int main(int argc, char **argv)
{
  float lambda=0.0, f=0.0, vector_estados[Npatas], T_ini;
  int Narg=0;

  Narg=22;
	if (argc>=Narg)
	{
        PasosIni=atoi(argv[1]);
        T_ini=atof(argv[2]); // Periodo de trayectoria [seg]
		divisionTrayectoriaPata_ini=atof(argv[3]);  //N puntos
		beta=atof(argv[4]);
		lambda=atof(argv[5]);
		alfa=atof(argv[6])*pi/180;
        Offset.x=atof(argv[7]);
        Offset.y=atof(argv[8]);
        Offset.z=atof(argv[9]);
        for(int k=0;k<Npatas;k++) desfasaje_t[k]=atof(argv[10+k]);
        for(int k=0;k<Npatas;k++) phi[k]=atof(argv[10+Npatas+k])*pi/180;
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
        T_apoyo[k] = beta*T[k];
        T_transf[k] = T[k]-T_apoyo[k];
        contadores[k] = desfasaje_t[k]*T_ini;
        divisionTrayectoriaPata[k] = divisionTrayectoriaPata_ini;
        if(desfasaje_t[k]>beta){
            vector_estados[k] = 1;
            T_contador[k] = T_transf[k];
        } else {
            vector_estados[k] = 0;
            T_contador[k] = T_apoyo[k];
        }
    }

//-- Datos de envio
    for(int k=0;k<Npatas;k++) {
        datosTrayectoriaPata.T.push_back(T_transf[k]);
        datosTrayectoriaPata.t_Trayectoria.push_back(0);
        datosTrayectoriaPata.lambda.push_back(lambda);
        datosTrayectoriaPata.alfa.push_back(alfa);
        datosTrayectoriaPata.desfasaje_t.push_back(desfasaje_t[k]);
        datosTrayectoriaPata.vector_estados.push_back(vector_estados[k]);
        datosTrayectoriaPata.cambio_estado.push_back(0);
        datosTrayectoriaPata.correccion_x.push_back(0);
        datosTrayectoriaPata.correccion_y.push_back(0);
        datosTrayectoriaPata.correccion_ID.push_back(-1);
    }

//-- Prepara variables para calculos de trayectoria de PATA
    //contadores = 1/divisionTrayectoriaPata;
    divisionTiempo = T_ini/divisionTrayectoriaPata_ini;
    f=1/divisionTiempo;
    /* La velocidad de envío de los datos se encarga de darme el tiempo total de trayectoria deseado */
    /* Velocidad de transmision */
    ros::Rate loop_rate(f);  //Frecuencia [Hz]
//-- Delay inicial para esperar inicio de todos los nodos
//    for(i=0;i<10;i++) loop_rate.sleep();

    modificacion_T_apoyo = T_ini;
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

bool CambioDeEstado_Apoyo(int nPata){
    bool cambio = false;
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
        cambio = true;
    }
    return cambio;
}

bool CambioDeEstado_Transf(int nPata){
    bool cambio = false;
//--- Transferencia Pata
    if (pataApoyo[nPata]==0 and FinTransf[nPata]) {
        InicioTransf[nPata]=true;
        FinTransf[nPata]=false;
    }
    if (pataApoyo[nPata]==1) {
        FinTransf[nPata]=true;
    }
    if (InicioTransf[nPata]){
        InicioTransf[nPata] = false;
        cambio = true;
    }
    return cambio;
}

bool LlegadaFinEDT(int nPata){

    bool cambio = false; punto3d P0, P1;

    P0.x = Offset.y-FinEspacioTrabajo_y;
    //-----Transformacion de trayectoria a Sistema de Pata
    P1 = TransformacionHomogenea(P0,Offset,phi[nPata]+alfa);
    if(nPata==5) ROS_INFO("Pata.y=%.4f\tfinEDT.y=%.4f",posicionActualPataSistemaPata[nPata].y,P1.y);
    if (fabs(posicionActualPataSistemaPata[nPata].y-P1.y)<0.0015 and FinTransf[nPata]) {
        InicioTransf[nPata]=true;
        FinTransf[nPata]=false;
    }
    if (pataApoyo[nPata]==0) {
        FinTransf[nPata]=true;
    }
    if (InicioTransf[nPata]){
        InicioTransf[nPata] = false;
        cambio = true;
    }
    return cambio;
}

/* Toma de muestras de tiempo y posicion para calculo de velocidad
.. se toma de muestra la pata1*/

void ParametrosVelocidad(){
//--- Apoyo de Pata 1
    if (InicioApoyo[0]){
        posCuerpo_1.x = coordenadaCuerpo.x;
        posCuerpo_1.y = coordenadaCuerpo.y;
        timer_1 = boost::posix_time::microsec_clock::local_time();
    }
//--- Transferencia de Pata 1
    if (InicioTransf[0]){
        posCuerpo_2.x = coordenadaCuerpo.x;
        posCuerpo_2.y = coordenadaCuerpo.y;
        timer_2 = boost::posix_time::microsec_clock::local_time();
    }
}

/* Retorna el modulo de la velocidad del cuerpo, segun el tiempo y distancia del cuerpo
.. en apoyo y transferencia*/
float VelocidadCuerpo(boost::posix_time::ptime t1, boost::posix_time::ptime t2, punto3d Pos1, punto3d Pos2){
    float delta_x=0.0, delta_y=0.0, tiempo_ahora=0.0;
    boost::posix_time::time_duration diff_t;

    delta_x = fabs(Pos1.x-Pos2.x);
    delta_y = fabs(Pos1.y-Pos2.y);
    diff_t = t1 - t2;
    tiempo_ahora = (float) fabs(diff_t.total_milliseconds())/1000;
    velocidadCuerpo.x = delta_x/tiempo_ahora;
    velocidadCuerpo.y = delta_y/tiempo_ahora;
    return (sqrt(velocidadCuerpo.x*velocidadCuerpo.x + velocidadCuerpo.y*velocidadCuerpo.y));
}

punto3d TransformacionHomogenea(punto3d Punto_in, punto3d L_traslacion, float ang_rotacion){

    punto3d Punto_out;

    Punto_out.x = L_traslacion.x + Punto_in.x*cos(ang_rotacion) - Punto_in.y*sin(ang_rotacion);
    Punto_out.y = L_traslacion.y + Punto_in.x*sin(ang_rotacion) + Punto_in.y*cos(ang_rotacion);
    Punto_out.z = L_traslacion.z + Punto_in.z;

    return(Punto_out);
}
