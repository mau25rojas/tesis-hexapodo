#include "ros/ros.h"
#include "std_msgs/String.h"
#include "math.h"
//Librerias propias usadas
#include "constantes.hpp"
#include "camina10/v_repConst.h"
// Used data structures:
#include "camina10/DatosTrayectoriaPata.h"
#include "camina10/SenalesCambios.h"
// Used API services:
#include "vrep_common/VrepInfo.h"

// variables Globales
bool simulationRunning=true;
bool sensorTrigger=false;
float simulationTime=0.0f;
camina10::SenalesCambios senales;
ros::Publisher chatter_pub1;

// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}
void senalesCallback(camina10::SenalesCambios msgSenal)
{
    senales.Stop=msgSenal.Stop;
    ROS_INFO("Nodo0 recibe senal de Planificador");
    chatter_pub1.publish(senales);
}

int main(int argc, char **argv){

    int Narg=0;
    float T=0.0, divisionTiempo=0.0, divisionTrayectoriaPata=0.0, f=0.0;

    Narg=2;
    if (argc>=Narg)
	{
		T=atof(argv[1]); // Periodo de trayectoria [seg]
		divisionTrayectoriaPata=atof(argv[2]);  //N puntos
    } else {
		ROS_ERROR("Nodo0: Indique argumentos!\n");
		return 0;
	}

    /*Inicio nodo de ROS*/
    ros::init(argc, argv, "Nodo0_controlMovimientoRob");
    ros::NodeHandle node;
    ROS_INFO("Nodo0_controlMovimientoRob just started\n");

//-- Topicos susbcritos y publicados
    chatter_pub1=node.advertise<camina10::SenalesCambios>("Reloj", 100);
    ros::Subscriber subInfo1=node.subscribe("/vrep/info",100,infoCallback);
    ros::Subscriber subInfo2=node.subscribe("Senal",100,senalesCallback);

//-- Datos temporales de parametrizacion
    divisionTiempo = T/divisionTrayectoriaPata;
    f=1/divisionTiempo;
    /* La velocidad de envío de los datos se encarga de darme el tiempo total de trayectoria deseado */
    /* Velocidad de transmision */
    ros::Rate loop_rate(f);  //Frecuencia [Hz]
    //Delay inicial para esperar inicio de todos los nodos
    for(int i=0;i<10;i++) loop_rate.sleep();
    senales.Stop=false;

    while(ros::ok() && simulationRunning){
        ros::spinOnce();
        loop_rate.sleep();
        chatter_pub1.publish(senales);
    }

    ROS_INFO("Adios0!");
    ros::shutdown();
    return 0;
}
