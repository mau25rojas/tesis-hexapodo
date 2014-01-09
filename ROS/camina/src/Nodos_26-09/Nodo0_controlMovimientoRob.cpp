#include "ros/ros.h"
#include "std_msgs/String.h"
#include "math.h"
//Librerias propias usadas
#include "constantes.hpp"
#include "camina/v_repConst.h"
// Used data structures:
#include "../msg_gen/cpp/include/camina/DatosTrayectoriaPata.h"
// Used API services:
#include "vrep_common/VrepInfo.h"

// variables Globales
bool simulationRunning=true;
bool sensorTrigger=false;
float simulationTime=0.0f;
float divisionTrayectoriaPata=0.0, beta=0.0, dh=0.0, desfasaje_t[6];
float x_Tray=0.0, y_Tray=0.0, z_Tray=0.0;
float x_FinApoyo=0.0,y_FinApoyo=0.0,z_FinApoyo=0.0;
bool finTrayectoria=false;
float PosicionCuerpo_y=0.0, PosicionCuerpo_x=0.0, theta_CuerpoRobot=0.0;
float trayectoriaPata_x[Npatas][100],trayectoriaPata_y[Npatas][100],trayectoriaPata_z[Npatas][100];
float posicionActualPata_x[Npatas],posicionActualPata_y[Npatas],posicionActualPata_z[Npatas];
//float Tripode_desfasaje_t[6]={0.0,0.5,0.5,0.0,0.0,0.5};
//float Sixpode_desfasaje_t[6]={0.0,3/6,1/6,4/6,2/6,5/6};

// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}


int main(int argc, char **argv)
{
  float T=0.0, divisionTiempo=0.0, f=0.0;
  float x_Offset=0.0, y_Offset=0.0, z_Offset=0.0, phi=0.0;
  float alfa[Npatas], desfasaje_t[Npatas];
  float lambda_Apoyo_arg=0.0,lambda_Transferencia_arg=0.0;
  int i=0, Narg=0;

  Narg=22;
      // (when V-REP launches this executable, V-REP will also provide the argument list)
	//numero de argumentos que mande (se excluye el fantasma que el manda solo)
	if (argc>=Narg)
	{
		T=atof(argv[1]); // Periodo de trayectoria [seg]
		divisionTrayectoriaPata=atof(argv[2]);  //N puntos
		beta=atof(argv[3]);
		lambda_Apoyo_arg=atof(argv[4]);
		lambda_Transferencia_arg=atof(argv[5]);
		dh=atof(argv[6]);
		x_Offset=atof(argv[7]);
		y_Offset=atof(argv[8]);
		z_Offset=atof(argv[9]);
		phi=atof(argv[10])*pi/180;
		for (i=0;i<Npatas;i++) alfa[i] = atof(argv[11+i])*pi/180;
        for (i=0;i<Npatas;i++) desfasaje_t[i] = atof(argv[11+Npatas+i]);
    } else {
		ROS_ERROR("Nodo0: Indique argumentos!\n");
		sleep(5000);
		return 0;
	}

    /*Inicio nodo de ROS*/
    ros::init(argc, argv, "Nodo0_controlMovimientoRob");
    ros::NodeHandle node;
    ROS_INFO("Nodo0_controlMovimientoRob just started\n");

//Topicos susbcritos y publicados
    ros::Publisher chatter_pub1 = node.advertise<camina::DatosTrayectoriaPata>("datosTrayectoriaPata", 100);
    ros::Subscriber subInfo1=node.subscribe("/vrep/info",100,infoCallback);

    //--- Datos de envio
    camina::DatosTrayectoriaPata datosTrayectoriaPata;
    datosTrayectoriaPata.divisionTrayectoriaPata=divisionTrayectoriaPata;
    datosTrayectoriaPata.beta=beta;
    datosTrayectoriaPata.dh=dh;
    datosTrayectoriaPata.x_Offset=x_Offset;
    datosTrayectoriaPata.y_Offset=y_Offset;
    datosTrayectoriaPata.z_Offset=z_Offset;
    datosTrayectoriaPata.phi=phi;
    for(i=0;i<Npatas;i++) {
        datosTrayectoriaPata.lambda_Apoyo.push_back(0);
        datosTrayectoriaPata.lambda_Transferencia.push_back(0);
        datosTrayectoriaPata.alfa.push_back(0);
        datosTrayectoriaPata.desfasaje_t.push_back(0);
        datosTrayectoriaPata.lambda_Apoyo[i] = lambda_Apoyo_arg;
        datosTrayectoriaPata.lambda_Transferencia[i]=lambda_Transferencia_arg;
        datosTrayectoriaPata.alfa[i] = alfa[i];
        datosTrayectoriaPata.desfasaje_t[i] = desfasaje_t[i];
    }

    // Prepara variables para calculos de trayectoria de PATA
    //delta_t = 1/divisionTrayectoriaPata;
    divisionTiempo = T/divisionTrayectoriaPata;
    f=1/divisionTiempo;
    /* La velocidad de envío de los datos se encarga de darme el tiempo total de trayectoria deseado */
    /* Velocidad de transmision */
    ros::Rate loop_rate(f);  //Frecuencia [Hz]
    for(i=0;i<20;i++) loop_rate.sleep();

    float delta_t=0.0;

    while (ros::ok() && simulationRunning){

        ros::spinOnce();
        loop_rate.sleep();
        datosTrayectoriaPata.t_Trayectoria = delta_t;
        chatter_pub1.publish(datosTrayectoriaPata);

        delta_t = delta_t + 1/divisionTrayectoriaPata;
        if (fabs(delta_t-1)<=2*(1/divisionTrayectoriaPata)){
            delta_t = 0.0;
        }
    }

        ROS_INFO("Adios0!");
        ros::shutdown();
        return 0;
}
