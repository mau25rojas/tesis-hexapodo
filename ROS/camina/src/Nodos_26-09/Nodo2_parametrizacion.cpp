#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include <cstdlib>
//Librerias propias usadas
#include "constantes.hpp"
#include "camina/v_repConst.h"
// Used data structures:
#include "../msg_gen/cpp/include/camina/DatosTrayectorias.h"
#include "../msg_gen/cpp/include/camina/AngulosMotor.h"
#include "../srv_gen/cpp/include/camina/CinversaParametros.h"
#include "../srv_gen/cpp/include/camina/TransTrayectoriaParametros.h"
// Used API services:
#include "vrep_common/VrepInfo.h"
//Clientes y Servicios
ros::ServiceClient client_Cinversa;
camina::CinversaParametros srv_Cinversa;
ros::ServiceClient client_TransTrayectoria_PataMundo;
camina::TransTrayectoriaParametros srv_TransTrayectoria_PataMundo;

// Global variables (modified by topic subscribers):
bool simulationRunning=true;
bool sensorTrigger=false;
float simulationTime=0.0f;
int Npata_arg=0, PataPrint=1;
//-- Calculo de trayectoria
camina::AngulosMotor qMotor;
ros::Publisher chatter_pub;
FILE *fp, *fp1;

// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}

/* Callback que escucha el topico DatosDeTrayectoria calcula la trayectoria
   deseaday cinematica inversa para de motores y los publica
*/
void datosCallback(const camina::DatosTrayectorias msg_datoTrayectoria)
{
    float x_S0=0.0, y_S0=0.0, z_S0=0.0;
    float x_S1=0.0, y_S1=0.0, z_S1=0.0;

    x_S0 = msg_datoTrayectoria.trayectoriaPata[Npata_arg-1].x;
    y_S0 = msg_datoTrayectoria.trayectoriaPata[Npata_arg-1].y;
    z_S0 = msg_datoTrayectoria.trayectoriaPata[Npata_arg-1].z;

    srv_TransTrayectoria_PataMundo.request.modo = Pata_Mundo;
    srv_TransTrayectoria_PataMundo.request.Npata = Npata_arg-1;
    srv_TransTrayectoria_PataMundo.request.x_S0 = x_S0;
    srv_TransTrayectoria_PataMundo.request.y_S0 = y_S0;
    srv_TransTrayectoria_PataMundo.request.z_S0 = z_S0;
    srv_TransTrayectoria_PataMundo.request.x_UbicacionRob = msg_datoTrayectoria.x_UbicacionRob;
    srv_TransTrayectoria_PataMundo.request.y_UbicacionRob = msg_datoTrayectoria.y_UbicacionRob;
    srv_TransTrayectoria_PataMundo.request.theta_Rob = msg_datoTrayectoria.theta_Rob;
    if (client_TransTrayectoria_PataMundo.call(srv_TransTrayectoria_PataMundo))
    {   x_S1 = srv_TransTrayectoria_PataMundo.response.x_Mundo;
        y_S1 = srv_TransTrayectoria_PataMundo.response.y_Mundo;
        z_S1 = srv_TransTrayectoria_PataMundo.response.z_Mundo;
//        if (Npata_arg==4 or Npata_arg==3) ROS_INFO("Nodo2: TransTrayectoria Pata: pata[%d] x=%.3f; y=%.3f; z=%.3f\n",Npata_arg,x_S1,y_S1,z_S1);
    } else {
        ROS_ERROR("Nodo2::pata[%d]: servicio de TransTrayectoria no funciona\n",Npata_arg);
    }

    qMotor.Npata = Npata_arg;
    //-----Cinematica Inversa
    srv_Cinversa.request.x = x_S1;
    srv_Cinversa.request.y = y_S1;
    srv_Cinversa.request.z = z_S1;

    if (client_Cinversa.call(srv_Cinversa)&&(srv_Cinversa.response.result!=-1))
    {   qMotor.q1 = srv_Cinversa.response.q1;
        qMotor.q2 = srv_Cinversa.response.q2;
        qMotor.q3 = srv_Cinversa.response.q3;
//                ROS_INFO("Servicio motores: q1=%.3f; q2=%.3f; q3=%.3f\n", _q1, _q2, _q3);
    } else {
        ROS_ERROR("Nodo 2::pata[%d]: servicio de Cinversa no funciona",Npata_arg);
//        ROS_WARN("Nodo2: result:%d\n",srv_Cinversa.response.result);
//        return;
    }
    //---------------------------------
//	// Creamos archivo solo para una pata
//	if (Npata_arg==PataPrint){
//        fprintf(fp,"%d,%d,%.3f,%.3f,%.3f\n",Npata_arg,qMotor.iteracion,qMotor.q1,qMotor.q2,qMotor.q3);
//        fprintf(fp1,"%d,%d,%.3f,%.3f,%.3f\n",Npata_arg,qMotor.iteracion,coordenadasCartesianas[0],coordenadasCartesianas[1],coordenadasCartesianas[2]);
////        fprintf(fp1,"%d,%d,%.3f,%.3f,%.3f\n",Npata_arg,qMotor.iteracion,x_S0,y_S0,z_S0);
//	}

    //---Publica angulos motores----
	chatter_pub.publish(qMotor);

      //ROS_INFO("2-Tiempo:[Npata=%d, ite=%d, t_Tray= %.3f, t_Tramo= %.3f]", qMotor.Npata, qMotor.iteracion, t_Trayectoria, t_Tramo);
      //ROS_INFO("3-Posicion xyz:[Npata=%d, ite=%d, x = %.3f, y = %.3f, z = %.3f]", qMotor.Npata, qMotor.iteracion, coordenadasCartesianas[0], coordenadasCartesianas[1], coordenadasCartesianas[2]);
}

int main(int argc, char **argv){

	// (when V-REP launches this executable, V-REP will also provide the argument list)
	// Se reciben 9 argumentos
	if (argc>=1)
	{
		Npata_arg=atoi(argv[1]);
	}
	else
	{
		ROS_ERROR("Nodo 2: Indique argumentos!\n");
		sleep(5000);
		return 0;
	}

    /*Inicio nodo de ROS*/
    std::string nodeName("Nodo2_Parametrizacion_pata");
	std::string Id(boost::lexical_cast<std::string>(Npata_arg));
	nodeName+=Id;
	ros::init(argc,argv,nodeName.c_str());
    ros::NodeHandle node;
    //ROS_INFO("Nodo2_Parametrizacion just started\n");

    //Topicos susbcritos y publicados
    chatter_pub = node.advertise<camina::AngulosMotor>("DatosDeMotores", 100);
//    ros::Subscriber sub = n.subscribe("datosTrayectoriaPataSalida", 100, datosCallback);
    ros::Subscriber sub = node.subscribe("datosTrayectoriaPata", 100, datosCallback);
    ros::Subscriber subInfo=node.subscribe("/vrep/info",1,infoCallback);
    //Clientes y Servicios
    client_Cinversa=node.serviceClient<camina::CinversaParametros>("Cinversa");
    client_TransTrayectoria_PataMundo=node.serviceClient<camina::TransTrayectoriaParametros>("TrayectoriaMundoPata");

//    if (Npata_arg==PataPrint){
//        fp = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina/datos/SalidaQ.txt","w+");
//        fp1 = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina/datos/SalidaX.txt","w+");
//    }
    while (ros::ok() && simulationRunning)
    {
	  ros::spinOnce();
    }
//    if (Npata_arg==PataPrint){
//        fclose(fp);
//        fclose(fp1);
//    }
    //ROS_INFO("Adios2!");
    ros::shutdown();
    return 0;
}

