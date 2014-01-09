#include "ros/ros.h"
#include "std_msgs/String.h"
#include "math.h"
//Librerias propias usadas
#include "constantes.hpp"
#include "camina/v_repConst.h"
// Used data structures:
#include <geometry_msgs/PoseStamped.h>
#include "../msg_gen/cpp/include/camina/DatosTrayectoriaPata.h"
//#include "../msg_gen/cpp/include/camina/DatosTrayectoriaRob.h"
#include "../msg_gen/cpp/include/camina/AngulosMotor.h"
#include "../msg_gen/cpp/include/camina/SenalesCambios.h"
// Used API services:
#include "vrep_common/VrepInfo.h"
#include "vrep_common/simRosGetObjectPose.h"

// variables Globales
bool simulationRunning=true;
bool sensorTrigger=false;
float simulationTime=0.0f;
//float Tripode_desfasaje_t[6]={0.0,0.5,0.5,0.0,0.0,0.5};
//float Sixpode_desfasaje_t[6]={0.0,3/6,1/6,4/6,2/6,5/6};
int k=1;
camina::DatosTrayectoriaPata datoTrayectoriaPata;

// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}

//void ajusteCallback(const camina::DatosTrayectoriaPata msg_DatosTrayectoriaPata)
//{
//     for(k=0;k<6;k++){
//        datoTrayectoriaPata.beta[k]=msg_DatosTrayectoriaPata.beta[k];
//        datoTrayectoriaPata.lambda2[k]=msg_DatosTrayectoriaPata.lambda2[k];
//     }
//
//}

int main(int argc, char **argv)
{
  float T=0.0, divisionTrayectoriaPata=0.0, divisionTiempo=0.0, f=0.0;
  float beta=0.0, lambda_Apoyo=0.0, lambda_Transferencia=0.0, dh=0.0, x_Offset=0.0, y_Offset=0.0, z_Offset=0.0, phi=0.0;
  float alfa[Npatas],desfasaje_t[Npatas];
  int i=0, Narg=0;
  float delta_t=0.0;
  int cuenta=0;

  Narg=22;
      // (when V-REP launches this executable, V-REP will also provide the argument list)
	//numero de argumentos que mande (se excluye el fantasma que el manda solo)
	if (argc>=Narg)
	{
		T=atof(argv[1]); // Periodo de trayectoria [seg]
		divisionTrayectoriaPata=atof(argv[2]);  //N puntos
		beta=atof(argv[3]);
		lambda_Apoyo=atof(argv[4]);
		lambda_Transferencia=atof(argv[5]);
		dh=atof(argv[6]);
		x_Offset=atof(argv[7]);
		y_Offset=atof(argv[8]);
		z_Offset=atof(argv[9]);
		phi=atof(argv[10]);
		for (i=0;i<Npatas;i++) alfa[i] = atof(argv[11+i]);
        for (i=0;i<Npatas;i++) desfasaje_t[i] = atof(argv[11+Npatas+i]);
    }
	else
	{
		ROS_ERROR("Nodo0: Indique argumentos!\n");
		sleep(5000);
		return 0;
	}

  /*Inicio nodo de ROS*/
  ros::init(argc, argv, "Nodo0_controlMovimientoRob");
  ros::NodeHandle node;
  ROS_INFO("Nodo0_controlMovimientoRob just started\n");

  //Topicos susbcritos y publicados
  ros::Publisher chatter_pub1 = node.advertise<camina::DatosTrayectoriaPata>("datosTrayectoriaPataEntrada", 100);
  ros::Publisher chatter_pub2 = node.advertise<camina::AngulosMotor>("DatosDeMotores", 100);
  ros::Subscriber subInfo1=node.subscribe("/vrep/info",100,infoCallback);
//  ros::Subscriber subInfo2=node.subscribe("PlanificacionDePisada",100,ajusteCallback);

  // Prepara variables para calculos de trayectoria de PATA
  //delta_t = 1/divisionTrayectoriaPata;
  divisionTiempo = T/divisionTrayectoriaPata;
  f=1/divisionTiempo;
  /* La velocidad de envío de los datos se encarga de darme el tiempo total de trayectoria deseado */
  /* Velocidad de transmision */
  ros::Rate loop_rate(f);  //Frecuencia [Hz]
  /* Los nodos siguientes inician lento, necesitamos un delay para esperarlos
     ..tomar en cuenta para sincronización entre patas
     ..seria preferible hacer un delay fijo, que no dependa del periodo de trayectoria
  */
  loop_rate.sleep();
  loop_rate.sleep();
  loop_rate.sleep();
  //Inicializacion de datos de trayectoria
  datoTrayectoriaPata.divisionTrayectoriaPata=divisionTrayectoriaPata;
  datoTrayectoriaPata.dh=dh;
  datoTrayectoriaPata.p_Offset.push_back(x_Offset);
  datoTrayectoriaPata.p_Offset.push_back(y_Offset);
  datoTrayectoriaPata.p_Offset.push_back(z_Offset);
  datoTrayectoriaPata.phi=phi;
  for(i=0;i<Npatas;i++) {
    datoTrayectoriaPata.beta.push_back(beta);
    datoTrayectoriaPata.lambda_Apoyo.push_back(lambda_Apoyo);
    datoTrayectoriaPata.lambda_Transferencia.push_back(lambda_Transferencia);
    datoTrayectoriaPata.alfa.push_back(alfa[i]);
    datoTrayectoriaPata.desfasaje_t.push_back(desfasaje_t[i]);
  }

  while (ros::ok() && simulationRunning)
  {
    datoTrayectoriaPata.t_Trayectoria = delta_t;
    datoTrayectoriaPata.iteracion=cuenta;
    delta_t = delta_t + 1/divisionTrayectoriaPata;
    ++cuenta;
    if (fabs(delta_t-1)<0.001)
    {
        delta_t=0.0;
        cuenta=0;
    }
//        if(StopRob!=1) {
    chatter_pub1.publish(datoTrayectoriaPata);
//        } else {
//            qMotor.q1=pi/2;
//            qMotor.q2=0.0;
//            qMotor.q3=pi/2;
//            for(i=1;i<=6;i++){
//                qMotor.Npata=i;
//                chatter_pub2.publish(qMotor);
//            }
//        }
        ros::spinOnce();
        loop_rate.sleep();
  }

    ROS_INFO("Adios0!");
    ros::shutdown();
    return 0;
}

