#include <ros/ros.h>
#include <std_msgs/String.h>
#include <math.h>
#include <time.h>
#include "boost/date_time/posix_time/posix_time.hpp"
//Librerias propias usadas
#include "constantes.hpp"
#include "pruebas/v_repConst.h"
#include "../../Convexhull/vector3d.hpp"
#include "../../Convexhull/convexhull.cpp"
#include "../../Convexhull/analisis.cpp"
// Used data structures:
#include "pruebas/DatosTrayectoriaPata.h"
#include "pruebas/SenalesCambios.h"
// Used API services:
#include "vrep_common/VrepInfo.h"

// variables Globales
bool simulationRunning=true;
bool sensorTrigger=false, Inicio=true;
float simulationTime=0.0f;
float divisionTrayectoriaPata[Npatas], divisionTiempo=0.0, desfasaje_t, beta=0.0, phi,alfa=0.0, dh=0.0, velApoyo=0.0;
float T[Npatas], T_contador[Npatas], T_apoyo[Npatas],T_transf[Npatas], contadores[Npatas],delta_t[Npatas], modificacion_T_apoyo = 0.0, modificacion_lambda =0.0;
int pataApoyo[Npatas],divisionTrayectoriaPata_ini;
int cuenta=0, PasosIni=0, PataPrint=2;
FILE *fp1;
punto3d Offset;
ros::Publisher chatter_pub1;
pruebas::DatosTrayectoriaPata datosTrayectoriaPata;


// Funciones
void Inizializacion();

// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}

void relojCallback(pruebas::SenalesCambios msgSenal)
{
    bool cambio_a_Apoyo=false,llegada_FEDT=false;
    float lambda = 0.0;

    if (!msgSenal.Stop){
        Inizializacion();
        ROS_INFO("contador=%0.3f",contadores[0]);
    }//-- fin is !Stop
} //-- fin de callback

int main(int argc, char **argv)
{
  float lambda=0.0, f=0.0, vector_estados[Npatas], T_ini;
  int Narg=0;

  Narg=9;
	if (argc>=Narg)
	{
        PasosIni=atoi(argv[1]);
        T_ini=atof(argv[2]); // Periodo de trayectoria [seg]
		divisionTrayectoriaPata_ini=atof(argv[3]);  //N puntos
		beta=atof(argv[4]);
		lambda=atof(argv[5]);
		velApoyo=atof(argv[6]);
		alfa=atof(argv[7])*pi/180;
        desfasaje_t=atof(argv[8]);
        phi=atof(argv[9])*pi/180;
    } else {
		ROS_ERROR("Nodo1: Indique argumentos, arg=%d!\n",argc);
		return 0;
	}

	/*Inicio nodo de ROS*/
    std::string nodeName("Nodo1_datosTrayectoriaPata");
	ros::init(argc,argv,nodeName.c_str());
    ros::NodeHandle node;
    ROS_INFO("Nodo1_datosTrayectoriaPata just started\n");

//-- Topicos susbcritos y publicados
    ros::Subscriber subInfo1=node.subscribe("/vrep/info",100,infoCallback);
    ros::Subscriber subInfo2=node.subscribe("Reloj",100,relojCallback);
//-- Manda topico especifico para cada pata
    chatter_pub1=node.advertise<pruebas::DatosTrayectoriaPata>("datosTrayectoria", 100);
//-- Clientes y Servicios
    //-- Log de datos
    std::string fileName("../fuerte_workspace/sandbox/TesisMaureen/ROS/pruebas/datos/SalidaDatos");
    std::string texto(".txt");
    fileName+=texto;
    fp1 = fopen(fileName.c_str(),"w+");

//-- Inicializo variables
    for(int k=0;k<Npatas;k++) {

        T[k] = T_ini;
        T_apoyo[k] = beta*T[k];
        T_transf[k] = T[k]-T_apoyo[k];
        if(k==0) ROS_INFO("T_apoyo=%.3f, T_trans=%.3f",T_apoyo[k],T_transf[k]);
        contadores[k] = desfasaje_t*T_ini;
        divisionTrayectoriaPata[k] = divisionTrayectoriaPata_ini;
        if(desfasaje_t>beta){
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
        datosTrayectoriaPata.desfasaje_t.push_back(desfasaje_t);
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
        }
        fprintf(fp1,"\n");
    }
}
