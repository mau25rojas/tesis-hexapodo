#include "ros/ros.h"
#include "math.h"
#include "string.h"
//Librerias propias usadas
#include "constantes.hpp"
#include "camina2/v_repConst.h"
// Used data structures:
#include "camina2/DatosTrayectoriaPata.h"
#include "camina2/AngulosMotor.h"
#include "camina2/CinversaParametros.h"
#include "camina2/TransHomogeneaParametros.h"
// Used API services:
#include "vrep_common/VrepInfo.h"
//Definiciones
//Clientes y Servicios
ros::ServiceClient client_Cinversa;
camina2::CinversaParametros srv_Cinversa;
ros::ServiceClient client_TransHomogenea;
camina2::TransHomogeneaParametros srv_TransHomogenea;

//-- Global variables (modified by topic subscribers):
bool simulationRunning=true;
bool sensorTrigger=false;
float simulationTime=0.0f;
int Npata_arg=0;
//-- Calculo de trayectoria
float T=0.0, beta=0.0, lambda_Transferencia=0.0, dh=0.0, desfasaje_t=0.0, phi=0.0;
float x_S0=0.0, y_S0=0.0, z_S0=0.0;
float x_Offset=0.0, y_Offset=0.0, z_Offset=0.0;   //matriz de transformacion homogenea
camina2::AngulosMotor qMotor;
ros::Publisher chatter_pub;
//-- Funciones
void Trayectoria_FaseApoyo(float t_Trayectoria);
void Trayectoria_FaseTrans_Eliptica(float t_Trayectoria);

// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}

/* Callback que escucha el topico DatosDeTrayectoria calcula la trayectoria
   deseaday cinematica inversa para de motores y los publica
*/
void datosCallback(const camina2::DatosTrayectoriaPata msg_datoTrayectoria)
{
    float t_Trayectoria=0.0;
    float alfa=0.0;
    float x_S1=0.0, y_S1=0.0, z_S1=0.0;

    T = msg_datoTrayectoria.T[0];
	t_Trayectoria = msg_datoTrayectoria.t_Trayectoria;
    lambda_Transferencia = msg_datoTrayectoria.lambda_Transferencia[0];
    alfa = msg_datoTrayectoria.alfa;
    //--------Temporizacion----------
    t_Trayectoria=t_Trayectoria+desfasaje_t;
    t_Trayectoria=fmod(t_Trayectoria,T);
    //-----Parametrizacion de trayectoria eliptica en Sistema de Robot
    //---Apoyo----
    // Periodo A-B
    if (0<=t_Trayectoria && t_Trayectoria<beta*T)
    {
        Trayectoria_FaseApoyo(t_Trayectoria);
    } else {
    //---Transferencia------
    // Elipsis
        Trayectoria_FaseTrans_Eliptica(t_Trayectoria);
    }

    //-----Transformacion de trayectoria a Sistema de Pata
    srv_TransHomogenea.request.x_S0 = x_S0;
    srv_TransHomogenea.request.y_S0 = y_S0;
    srv_TransHomogenea.request.z_S0 = z_S0;
    srv_TransHomogenea.request.x_Trasl = x_Offset;
    srv_TransHomogenea.request.y_Trasl = y_Offset;
    srv_TransHomogenea.request.z_Trasl = z_Offset;
    srv_TransHomogenea.request.theta_Rot = phi+alfa;
    if (client_TransHomogenea.call(srv_TransHomogenea))
    {   x_S1 = srv_TransHomogenea.response.x_S1;
        y_S1 = srv_TransHomogenea.response.y_S1;
        z_S1 = srv_TransHomogenea.response.z_S1;
    } else {
        ROS_ERROR("Nodo 2::[%d] servicio de TransHomogenea no funciona\n",Npata_arg);
//        return;
    }
    //-----Cinematica Inversa
    srv_Cinversa.request.x = x_S1;
    srv_Cinversa.request.y = y_S1;
    srv_Cinversa.request.z = z_S1;

    if (client_Cinversa.call(srv_Cinversa)&&(srv_Cinversa.response.result!=-1))
    {   qMotor.q1 = srv_Cinversa.response.q1;
        qMotor.q2 = srv_Cinversa.response.q2;
        qMotor.q3 = srv_Cinversa.response.q3;
    } else {
        ROS_ERROR("Nodo 2::[%d] servicio de Cinversa no funciona\n",Npata_arg);
//        return;
    }
    //---Publica angulos motores----
    qMotor.Npata = Npata_arg;
	chatter_pub.publish(qMotor);
}

int main(int argc, char **argv){
	// (when V-REP launches this executable, V-REP will also provide the argument list)
	if (argc>=8)
	{
		Npata_arg=atoi(argv[1]);
		beta=atof(argv[2]);
		dh=atof(argv[3]);
        x_Offset=atof(argv[4]);
        y_Offset=atof(argv[5]);
        z_Offset=atof(argv[6]);
        phi=atof(argv[7])*pi/180.0;
        desfasaje_t=atof(argv[8]);
	}
	else
	{
		ROS_ERROR("Nodo 2::[%d] Indique argumentos!\n",Npata_arg);
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

//-- Topicos susbcritos y publicados
    chatter_pub = node.advertise<camina2::AngulosMotor>("DatosDeMotores", 100);
    ros::Subscriber subInfo = node.subscribe("/vrep/info",1,infoCallback);
    ros::Subscriber sub = node.subscribe("datosTrayectoria", 100, datosCallback);
//-- Clientes y Servicios
    client_Cinversa = node.serviceClient<camina2::CinversaParametros>("Cinversa");
    client_TransHomogenea = node.serviceClient<camina2::TransHomogeneaParametros>("TransHomogenea");

    while (ros::ok() && simulationRunning)
    {
	  ros::spinOnce();
    }
    ros::shutdown();
    return 0;
}

//---Caso parte 1 trayectoria----
//-- Periodo A-B
void Trayectoria_FaseApoyo(float t_Trayectoria){

    float velocidadApoyo=0.0;

    velocidadApoyo = lambda_Transferencia/(beta*T);

    x_S0 = -lambda_Transferencia/2 + velocidadApoyo*t_Trayectoria;
    y_S0 = 0.0;
    z_S0 = 0.0;
}

//---Caso parte 2 trayectoria----
//-- Elipsis
void Trayectoria_FaseTrans_Eliptica(float t_Trayectoria){

    float theta=0.0, t_aux=0.0;

    t_aux = (t_Trayectoria-beta*T)/((1-beta)*T);
    theta = pi*t_aux;

    x_S0 = (lambda_Transferencia/2)*cos(theta);
    y_S0 = 0.0;
    z_S0 = dh*sin(theta);
}
