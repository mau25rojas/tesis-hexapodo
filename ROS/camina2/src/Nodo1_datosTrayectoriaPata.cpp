#include "ros/ros.h"
#include "std_msgs/String.h"
#include "math.h"
//Librerias propias usadas
#include "constantes.hpp"
#include "camina2/v_repConst.h"
// Used data structures:
#include "camina2/DatosTrayectoriaPata.h"
// Used API services:
#include "vrep_common/VrepInfo.h"

// variables Globales
bool simulationRunning=true;
bool sensorTrigger=false;
float simulationTime=0.0f;
camina2::DatosTrayectoriaPata datosTrayectoriaPata;
float divisionTrayectoriaPata=0.0, T=0.0, divisionTiempo=0.0, f=0.0;
//FILE *fp1;

// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}

int main(int argc, char **argv)
{
  float lambda_Transferencia=0.0, alfa=0.0, beta=0.0;
  int i=0, Narg=0, Tripode=0;

  Narg=5;
      // (when V-REP launches this executable, V-REP will also provide the argument list)
	//numero de argumentos que mande (se excluye el fantasma que el manda solo)
	if (argc>=Narg)
	{
        T=atof(argv[1]); // Periodo de trayectoria [seg]
		divisionTrayectoriaPata=atof(argv[2]);  //N puntos
		beta=atof(argv[3]);
		lambda_Transferencia=atof(argv[4]);
		alfa=atof(argv[5])*pi/180;
    } else {
		ROS_ERROR("Nodo1: Indique argumentos!\n");
		sleep(5000);
		return 0;
	}

        datosTrayectoriaPata.T.push_back(0);
        datosTrayectoriaPata.lambda_Transferencia.push_back(0);

	/*Inicio nodo de ROS*/
	ros::init(argc,argv,"Nodo1_datosTrayectoriaPata");
    ros::NodeHandle node;
    ROS_INFO("Nodo1_datosTrayectoriaPata just started\n");

//-- Topicos susbcritos y publicados
    ros::Subscriber subInfo1=node.subscribe("/vrep/info",100,infoCallback);
    ros::Publisher chatter_pub1=node.advertise<camina2::DatosTrayectoriaPata>("datosTrayectoria", 100);
//-- Datos de envio
    datosTrayectoriaPata.T[0]=T;
    datosTrayectoriaPata.divisionTrayectoriaPata=divisionTrayectoriaPata;
    datosTrayectoriaPata.lambda_Transferencia[0]=lambda_Transferencia;
    datosTrayectoriaPata.alfa=alfa;
//-- Prepara variables para calculos de trayectoria de PATA
    divisionTiempo = T/divisionTrayectoriaPata;
    f=1/divisionTiempo;
    /* La velocidad de envío de los datos se encarga de darme el tiempo total de trayectoria deseado */
    /* Velocidad de transmision */
    ros::Rate loop_rate(f);  //Frecuencia [Hz]
//-- Delay inicial para esperar inicio de todos los nodos
    for(i=0;i<10;i++) loop_rate.sleep();
    float delta_t=0.0;
    while (ros::ok() && simulationRunning){

        ros::spinOnce();
        loop_rate.sleep();

//-- Esta linea es muy importante
        chatter_pub1.publish(datosTrayectoriaPata);

        delta_t = delta_t + T/divisionTrayectoriaPata;
        datosTrayectoriaPata.t_Trayectoria = delta_t;

        if (fabs(delta_t-T)<(T/divisionTrayectoriaPata)) {
//            ROS_INFO("reinicio trayectoria[%d]",Tripode);
            delta_t = 0.0;
        }

//        fprintf(fp1,"%.3f\n",delta_t);
    }
//        ROS_INFO("Adios1!");
//        fclose(fp1);
        ros::shutdown();
        return 0;
    }
