#include <ros/ros.h>
#include <std_msgs/String.h>
#include <math.h>
#include <time.h>
#include "boost/date_time/posix_time/posix_time.hpp"
//Librerias propias usadas
#include "constantes.hpp"
#include "camina11/v_repConst.h"
#include "camina11/vector3d.hpp"
// Used data structures:
#include "camina11/DatosTrayectoriaPata.h"
#include "camina11/PlanificadorParametros.h"
#include "camina11/UbicacionRobot.h"
// Used API services:
#include "vrep_common/VrepInfo.h"
//Clientes y Servicios
ros::ServiceClient client_Planificador;
camina11::PlanificadorParametros srv_Planificador;

// variables Globales
bool simulationRunning=true;
bool sensorTrigger=false, Inicio=true;
bool InicioApoyo[Npatas]={false,false,false,false,false,false}, FinApoyo[Npatas]={false,false,false,false,false,false};
bool InicioTransf[Npatas]={false,false,false,false,false,false}, FinTransf[Npatas]={false,false,false,false,false,false},mediaTransf[Npatas]={true,true,true,true,true,true};
float simulationTime=0.0f;
float divisionTrayectoriaPata[Npatas], divisionTiempo=0.0, desfasaje_t[Npatas], beta=0.0, phi[Npatas],alfa=0.0, dh=0.0, velApoyo=0.0;
float T[Npatas], T_contador[Npatas], T_apoyo[Npatas],T_transf[Npatas], contadores[Npatas],delta_t[Npatas], modificacion_T_apoyo = 0.0, modificacion_lambda =0.0;
float xCuerpo_1=0.0, xCuerpo_2=0.0, yCuerpo_1=0.0, yCuerpo_2=0.0, mod_velocidadCuerpo=0.0;
int pataApoyo[Npatas],divisionTrayectoriaPata_ini;
int cuenta=0, PasosIni=0, PataPrint=0;
FILE *fp1;
punto3d coordenadaCuerpo, velocidadCuerpo, posicionActualPataSistemaPata[Npatas],posCuerpo_1, posCuerpo_2;
punto3d Offset;
boost::posix_time::ptime timer_1,timer_2;
camina11::DatosTrayectoriaPata datosTrayectoriaPata;
ros::Publisher chatter_pub1,chatter_pub2;

// Funciones
void Inizializacion();
bool CambioDeEstado_Apoyo(int nPata);
bool CambioDeEstado_Transf(int nPata);
int VerificacionEstadoPata(int nPata, int estadoActual);
bool LlegadaFinEDT(int nPata);
void ParametrosVelocidad();
float VelocidadCuerpo(boost::posix_time::ptime t1, boost::posix_time::ptime t2, punto3d Pos1, punto3d Pos2);

// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}

void ubicacionRobCallback(camina11::UbicacionRobot msgUbicacionRobot)
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

int main(int argc, char **argv)
{
  float lambda=0.0, f=0.0, vector_estados[Npatas], T_ini;
  int Narg=0;
  bool cambio_a_Apoyo=false,llegada_FEDT=false;

  Narg=23;
	if (argc>=Narg)
	{
        PasosIni=atoi(argv[1]);
        T_ini=atof(argv[2]); // Periodo de trayectoria [seg]
		divisionTrayectoriaPata_ini=atof(argv[3]);  //N puntos
		beta=atof(argv[4]);
		lambda=atof(argv[5]);
		velApoyo=atof(argv[6]);
		alfa=atof(argv[7])*pi/180;
		dh=atof(argv[8]);
        Offset.x=atof(argv[9]);
        Offset.y=atof(argv[10]);
        Offset.z=atof(argv[11]);
        for(int k=0;k<Npatas;k++) desfasaje_t[k]=atof(argv[12+k]);
        for(int k=0;k<Npatas;k++) phi[k]=atof(argv[12+Npatas+k])*pi/180;
    } else {
		ROS_ERROR("Nodo1: Indique argumentos!\n");
		return 0;
	}

	/*Inicio nodo de ROS*/
    std::string nodeName("Nodo1_datosTrayectoriaPata");
	ros::init(argc,argv,nodeName.c_str());
    ros::NodeHandle node;
    ROS_INFO("Nodo1_datosTrayectoriaPata just started\n");

//-- Topicos susbcritos y publicados
    ros::Subscriber subInfo1=node.subscribe("/vrep/info",100,infoCallback);
    ros::Subscriber subInfo3=node.subscribe("UbicacionRobot",100,ubicacionRobCallback);
//-- Manda topico especifico para cada pata
    chatter_pub1=node.advertise<camina11::DatosTrayectoriaPata>("datosTrayectoria", 100);
//-- Clientes y Servicios
    client_Planificador = node.serviceClient<camina11::PlanificadorParametros>("PlanificadorPisada");
//-- Log de datos
    std::string fileName("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina11/datos/SalidaDatos");
    std::string texto(".txt");
    fileName+=texto;
    fp1 = fopen(fileName.c_str(),"w+");

//-- Inicializo variables
    for(int k=0;k<Npatas;k++) {

        T[k] = T_ini;
        T_apoyo[k] = beta*T[k];
        T_transf[k] = T[k]-T_apoyo[k];
        if(k==0) ROS_INFO("T_apoyo=%.3f, T_trans=%.3f",T_apoyo[k],T_transf[k]);
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

    datosTrayectoriaPata.alfa=alfa;
//-- Datos de envio
    for(int k=0;k<Npatas;k++) {
        datosTrayectoriaPata.T.push_back(T_transf[k]);
        datosTrayectoriaPata.t_Trayectoria.push_back(0);
        datosTrayectoriaPata.lambda.push_back(lambda);
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
        loop_rate.sleep();

        ParametrosVelocidad();

        if (Inicio){
            Inizializacion();
        } else {
            chatter_pub1.publish(datosTrayectoriaPata);

            for(int k=0;k<Npatas;k++){
//                if(k==PataPrint) ROS_INFO("pata[%d] contador=%.3f",k+1,contadores[k]);
                datosTrayectoriaPata.cambio_estado[k]=0;

                //-- Verificacion por posicion actual de pata recibida
                //-- Seleccion de estado proximo de la pata
                //-- si hay cambio de estado se activa la bandera correspondiente
                datosTrayectoriaPata.vector_estados[k]=VerificacionEstadoPata(k,datosTrayectoriaPata.vector_estados[k]);
//                if(k==PataPrint) ROS_INFO("pata[%d] estado=%d",k+1,datosTrayectoriaPata.vector_estados[k]);

                if(datosTrayectoriaPata.vector_estados[k]==Apoyo and datosTrayectoriaPata.cambio_estado[k]==1) contadores[k]=0.0;

                if (datosTrayectoriaPata.vector_estados[k]==Transferencia) {
                    T_contador[k]=T[k];
                } else {
                    T_contador[k]=T_apoyo[k];
                }

                if (fabs(contadores[k]-T_contador[k])<(T[k]/divisionTrayectoriaPata[k])){
                    contadores[k] = contadores[k];
                } else {
                    contadores[k] = contadores[k] + T[k]/divisionTrayectoriaPata[k];
                }

                if (datosTrayectoriaPata.vector_estados[k]==Transferencia) {
                    delta_t[k] = contadores[k]-T_apoyo[k];
                } else {
                    delta_t[k] = contadores[k];
                }
//                if (fabs(contadores[k]-T[k])<(T[k]/divisionTrayectoriaPata[k])) {
//                    contadores[k]=0.0;
//                    datosTrayectoriaPata.vector_estados[k]==Apoyo;
//                }
                datosTrayectoriaPata.T[k]=T_transf[k];
                datosTrayectoriaPata.t_Trayectoria[k]=delta_t[k];
            }// fin del for

        }//-- Checkea por inicio

    }
        ROS_INFO("Adios1!");
        fclose(fp1);
        ros::shutdown();
        return 0;
}

/* Funciones */

void Inizializacion(){

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
//            if(k==0) ROS_INFO("cuenta=%.3f",contadores[k]);

            if (contadores[k]>=beta*T[k]) {
                delta_t[k] = contadores[k]-T_apoyo[k];
                datosTrayectoriaPata.vector_estados[k]=Transferencia;
            } else {
                delta_t[k] = contadores[k];
                datosTrayectoriaPata.vector_estados[k]=Apoyo;
            }
            if (fabs(contadores[k]-T[k])<=(T[k]/divisionTrayectoriaPata[k])){
                contadores[k] = 0.0;
            }
            CambioDeEstado_Apoyo(k);
            LlegadaFinEDT(k);
        }
        fprintf(fp1,"\n");
    }
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

//-- Verificacion por posicion actual de pata recibida
int VerificacionEstadoPata(int nPata, int estadoActual){
    bool cambio_a_Apoyo=false,llegada_FEDT=false;
    int estadoSiguiente;
    estadoSiguiente=estadoActual;

    cambio_a_Apoyo = CambioDeEstado_Apoyo(nPata);
    if(cambio_a_Apoyo){
        cambio_a_Apoyo = false;
        estadoSiguiente = Apoyo;
        datosTrayectoriaPata.cambio_estado[nPata]=1;
        if(nPata==PataPrint) ROS_WARN("***Inicia Apoyo pata[%d]",nPata+1);
    }
    llegada_FEDT = LlegadaFinEDT(nPata);
    if(llegada_FEDT){
        llegada_FEDT = false;
        estadoSiguiente = Transferencia;
        datosTrayectoriaPata.cambio_estado[nPata]=1;
        if(nPata==PataPrint) ROS_WARN("------Inicia Transferencia pata[%d]",nPata+1);
    }

    return(estadoSiguiente);
}

bool CambioDeEstado_Apoyo(int nPata){
    bool cambio = false;
//--- Apoyo de Pata
    if (pataApoyo[nPata]==Apoyo and FinApoyo[nPata]) {
        InicioApoyo[nPata]=true;
        FinApoyo[nPata]=false;
    }
//    if (fabs(contadores[nPata]-T_transf[nPata]/2)<(T[nPata]/divisionTrayectoriaPata[nPata])) {
    if (pataApoyo[nPata]==Transferencia) {
        FinApoyo[nPata]=true;
//        if(nPata==PataPrint) ROS_WARN("****Pata[%d] preApoyo",nPata+1);
    }
    if (InicioApoyo[nPata]){
        InicioApoyo[nPata]=false;
        cambio=true;
    }
    return cambio;
}


bool LlegadaFinEDT(int nPata){

    float paso_y = 0.0;
    bool cambio = false; punto3d P0, Fin_EDT;
    paso_y = velApoyo*divisionTiempo;
//    ROS_WARN("%.3f,%.3f",velApoyo,divisionTiempo);
    P0.x = Offset.y-FinEspacioTrabajo_y-paso_y;
    //-----Transformacion de trayectoria a Sistema de Pata
    Fin_EDT = TransformacionHomogenea(P0,Offset,phi[nPata]+alfa);

//    if(nPata==PataPrint) ROS_WARN("pata.y=%.4f,finEDT.y=%.4f",posicionActualPataSistemaPata[nPata].y,Fin_EDT.y);
    if (fabs(posicionActualPataSistemaPata[nPata].y-Fin_EDT.y)<=0.005 and FinTransf[nPata]) {
        InicioTransf[nPata]=true;
        FinTransf[nPata]=false;
    }
//    if(nPata==n) ROS_WARN("Pata.z=%.4f",posicionActualPataSistemaPata[nPata].z);
//    if (fabs(contadores[nPata]-T_apoyo[nPata]/2)<(T[nPata]/divisionTrayectoriaPata[nPata])) {
    if (pataApoyo[nPata]==Apoyo) {
        FinTransf[nPata]=true;
//        if(nPata==PataPrint) ROS_WARN("------Pata[%d] preTransferencia",nPata+1);
    }
    if (InicioTransf[nPata]){
        InicioTransf[nPata]=false;
        cambio=true;
    }
    return cambio;
}
