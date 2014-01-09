#include "ros/ros.h"
#include "std_msgs/String.h"
#include "math.h"
//Librerias propias usadas
#include "constantes.hpp"
#include "camina/v_repConst.h"
// Used data structures:
#include <geometry_msgs/PoseStamped.h>
#include "../msg_gen/cpp/include/camina/DatosTrayectoriaRob.h"
// Used API services:
#include "vrep_common/VrepInfo.h"
#include "vrep_common/simRosGetObjectPose.h"

// variables Globales
bool simulationRunning=true;
bool sensorTrigger=false;
float simulationTime=0.0f;
camina::DatosTrayectoriaRob datoTrayectoriaRob;
geometry_msgs::PoseStamped PoseStamped;

//Servicios
ros::ServiceClient client_simRosGetObjectPose;
vrep_common::simRosGetObjectPose srv_simRosGetObjectPose; //Servicio para obtener posicion del robot

// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}

int main(int argc, char **argv)
{
  int handleArana=0,i=0;
  float divisionTrayectoriaRobot=0.0;
  float x_final=0.0, y_final=0.0, z_final=0.0;
  float x_ahora=0.0,y_ahora=0.0,z_ahora=0.0;
  float x_anterior=0.0,y_anterior=0.0,z_anterior=0.0;
//  float deta_x=0.0, delta_y=0.0, delta_z=0.0;
  float alfaInicio[6]={0.0,0.0,0.0,0.0,0.0,0.0};
  float tita=0.0, gamma=0.0, correcion_alfa=0.0;

    // (when V-REP launches this executable, V-REP will also provide the argument list)
	//numero de argumentos que mande (se excluye el fantasma que el manda solo)
	if (argc>=10)
	{
	    handleArana=atoi(argv[1]);
		divisionTrayectoriaRobot=atof(argv[2]);
		x_final=atof(argv[3]);
		y_final=atof(argv[4]);
		z_final=atof(argv[5]);
        alfaInicio[0]=atof(argv[6]);
		alfaInicio[1]=atof(argv[7]);
		alfaInicio[2]=atof(argv[8]);
		alfaInicio[3]=atof(argv[9]);
		alfaInicio[4]=atof(argv[10]);
		alfaInicio[5]=atof(argv[11]);
    }
	else
	{
		printf("Indique argumentos!\n");
		sleep(5000);
		return 0;
	}

  /*Inicio nodo de ROS*/
  ros::init(argc, argv, "Nodo0_ControldeTrayectoria");
  ros::NodeHandle n;
  ROS_INFO("Nodo0_ControldeTrayectoria just started\n");

  //Topicos susbcritos y publicados
  ros::Publisher chatter_pub = n.advertise<camina::DatosTrayectoriaRob>("datoTrayectoriaRob", 100);
  ros::Subscriber subInfo=n.subscribe("/vrep/info",1,infoCallback);
  //Servicios
  client_simRosGetObjectPose=n.serviceClient<vrep_common::simRosGetObjectPose>("/vrep/simRosGetObjectPose");
  //Detalles de servicio
  srv_simRosGetObjectPose.request.handle=handleArana;
  srv_simRosGetObjectPose.request.relativeToObjectHandle=-1;

  // Prepara variables para calculos de trayectoria de ROBOT
//  datoTrayectoria.delta_x = (x_final-x_inicio)/divisionTrayectoriaRobot;
//  datoTrayectoria.delta_y = (y_final-y_inicio)/divisionTrayectoriaRobot;
//  datoTrayectoria.delta_z = (z_final-z_inicio)/divisionTrayectoriaRobot;
  for(i=0;i<6;i++) datoTrayectoriaRob.alfa.push_back(alfaInicio[i]);

  /* Velocidad de transmision */
  ros::Rate loop_rate(1);  //Frecuencia [Hz]

  int cuenta = 0;
  while (ros::ok() && simulationRunning)
  {
//      cuenta++;
//      //Servicio checkea posicion del robot
//      if (client_simRosGetObjectPose.call(srv_simRosGetObjectPose)&&(srv_simRosGetObjectPose.response.result!=-1))
//      {
//            PoseStamped = srv_simRosGetObjectPose.response.pose;
//            x_ahora = PoseStamped.pose.position.x;
//            y_ahora = PoseStamped.pose.position.y;
//            z_ahora = PoseStamped.pose.position.z;
////            ROS_INFO("posicion: x=%.3f; y=%.3f; z=%.3f\n", x_ahora,y_ahora,z_ahora);
//
//            if(fabs(x_final-x_ahora)<0.1 && fabs(y_final-y_ahora)<0.1){
//                //ROS_INFO("STOP!!\n");
//                datoTrayectoriaRob.Stop=true;
//            } else {
//                datoTrayectoriaRob.Stop=false;
//                if(cuenta%10==0){
//                    gamma=atan2(y_final-y_anterior,x_final-x_anterior);
//                    tita=atan2(y_ahora-y_anterior,x_ahora-x_anterior);
//                    correcion_alfa = (gamma-tita)*180.0/pi;
//                    for(i=0;i<6;i++) datoTrayectoriaRob.alfa[i]=datoTrayectoriaRob.alfa[i]+correcion_alfa;
////                    ROS_INFO("alfa_correc=%.3f\n", correcion_alfa);
//                }
//            }
//       } else{
//            ROS_ERROR("Nodo0: No funciona servicio posicion");
//         }
//
//        chatter_pub.publish(datoTrayectoriaRob);
        ros::spinOnce();
        loop_rate.sleep();
//
//        x_anterior=x_ahora;
//        y_anterior=y_ahora;
//        z_anterior=z_ahora;
  }
    ROS_INFO("Adios12!");
    ros::shutdown();
    return 0;
}

