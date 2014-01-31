#include "ros/ros.h"
#include "math.h"
#include "string.h"
//Librerias propias usadas
#include "constantes.hpp"
#include "camina6/v_repConst.h"
// Used data structures:
#include "camina6/DatosTrayectoriaPata.h"
#include "camina6/AngulosMotor.h"
#include "camina6/CinversaParametros.h"
#include "camina6/TransHomogeneaParametros.h"
// Used API services:
#include "vrep_common/VrepInfo.h"
//Definiciones
//#define TrayInicio -0.05
//Clientes y Servicios
ros::ServiceClient client_Cinversa;
camina6::CinversaParametros srv_Cinversa;
ros::ServiceClient client_TransHomogenea;
camina6::TransHomogeneaParametros srv_TransHomogenea;

//-- Global variables (modified by topic subscribers):
bool simulationRunning=true;
bool sensorTrigger=false;
float simulationTime=0.0f;
int Npata_arg=0;
bool Inicio=true, finTransferencia=true;
//-- Calculo de trayectoria
float T=0.0, delta_t=0.0, beta=0.0, lambda_Apoyo=0.0, lambda_Transferencia=0.0, dh=0.0, desfasaje_t=0.0, phi=0.0;
float x_S0=0.0, y_S0=0.0, z_S0=0.0;
float finTransferencia_x=0.0;
float x_Offset=0.0, y_Offset=0.0, z_Offset=0.0;   //matriz de transformacion homogenea
camina6::AngulosMotor qMotor;
ros::Publisher chatter_pub;
FILE *fp1, *fp2;
//-- Funciones
void Trayectoria_FaseApoyo(float t_Trayectoria,float PuntoInicio_x,float PuntoInicio_y,float PuntoInicio_z);
void Trayectoria_FaseTrans_Eliptica(float t_Trayectoria,float PuntoInicio_x,float PuntoInicio_y,float PuntoInicio_z);

// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}

/* Callback que escucha el topico DatosDeTrayectoria calcula la trayectoria
   deseaday cinematica inversa para de motores y los publica
*/
void datosCallback(const camina6::DatosTrayectoriaPata msg_datoTrayectoria)
{
    float t_Trayectoria=0.0;
    float alfa=0.0;
    float x_S1=0.0, y_S1=0.0, z_S1=0.0;
    float PuntoInicio_x=0.0, PuntoInicio_y=0.0, PuntoInicio_z=0.0;

    T = msg_datoTrayectoria.T;
	delta_t = T/msg_datoTrayectoria.divisionTrayectoriaPata;
	t_Trayectoria = msg_datoTrayectoria.t_Trayectoria;
    lambda_Transferencia = msg_datoTrayectoria.lambda_Transferencia;
    alfa = msg_datoTrayectoria.alfa;
    desfasaje_t = msg_datoTrayectoria.desfasaje_t;

    //---------------------------------
    if (Inicio){
    //-- La trayectoria inicial se hace con lambda de apoyo
        Inicio = false;
        lambda_Apoyo = msg_datoTrayectoria.lambda_Apoyo;
        finTransferencia_x = (y_Offset-FinEspacioTrabajo_y)-lambda_Apoyo;
    }
    //--------Temporizacion----------
//    t_Trayectoria=t_Trayectoria+desfasaje_t;
//    t_Trayectoria=fmod(t_Trayectoria,T);
    //-----Parametrizacion de trayectoria eliptica en Sistema de Robot
    //---Apoyo----
    // Periodo A-B
    if (0<t_Trayectoria && t_Trayectoria<=beta*T)
    {
        PuntoInicio_x=finTransferencia_x;
        PuntoInicio_y=0.0;
        PuntoInicio_z=0.0;
        Trayectoria_FaseApoyo(t_Trayectoria,PuntoInicio_x,PuntoInicio_y,PuntoInicio_z);
    } else {
    //---Transferencia------
    // Elipsis
        PuntoInicio_x=(y_Offset-FinEspacioTrabajo_y)-lambda_Transferencia/2;
        PuntoInicio_y=0.0;
        PuntoInicio_z=0.0;
        Trayectoria_FaseTrans_Eliptica(t_Trayectoria,PuntoInicio_x,PuntoInicio_y,PuntoInicio_z);
    }

    //-----Transformacion de trayectoria a Sistema de Pata
    x_S1 = x_Offset + x_S0*cos(phi+alfa) - y_S0*sin(phi+alfa);
    y_S1 = y_Offset + x_S0*sin(phi+alfa) + y_S0*cos(phi+alfa);
    z_S1 = z_Offset + z_S0;
    //-----Cinematica Inversa
    srv_Cinversa.request.x = x_S1;
    srv_Cinversa.request.y = y_S1;
    srv_Cinversa.request.z = z_S1;

    fprintf(fp1,"%d,%.3f,%.3f,%.3f,%.3f\n",Npata_arg,t_Trayectoria,x_S1,y_S1,z_S1);

    if (client_Cinversa.call(srv_Cinversa)&&(srv_Cinversa.response.result!=-1))
    {   qMotor.q1 = srv_Cinversa.response.q1;
        qMotor.q2 = srv_Cinversa.response.q2;
        qMotor.q3 = srv_Cinversa.response.q3;
        fprintf(fp2,"%d,%.3f,%.3f,%.3f,%.3f\n",Npata_arg,t_Trayectoria,qMotor.q1,qMotor.q2,qMotor.q3);
    } else {
        ROS_ERROR("Nodo 2::[%d] servicio de Cinversa no funciona\n",Npata_arg);
//        return;
    }

    //---Publica angulos motores----
    qMotor.Npata = Npata_arg;
	chatter_pub.publish(qMotor);
}

int main(int argc, char **argv){

	int Tripode=0;
	if (argc>=8)
	{
		Npata_arg=atoi(argv[1]);
		beta=atof(argv[2]);
		dh=atof(argv[3]);
        x_Offset=atof(argv[4]);
        y_Offset=atof(argv[5]);
        z_Offset=atof(argv[6]);
        phi=atof(argv[7])*pi/180.0;
        Tripode=atof(argv[8]);
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
    chatter_pub = node.advertise<camina6::AngulosMotor>("DatosDeMotores", 100);
    ros::Subscriber subInfo = node.subscribe("/vrep/info",1,infoCallback);
//-- Recibe topico especifico
    std::string topicName("datosTrayectoria_T");
    std::string Trip(boost::lexical_cast<std::string>(Tripode));
    topicName+=Trip;
    ros::Subscriber sub = node.subscribe(topicName.c_str(), 100, datosCallback);
//-- Clientes y Servicios
    client_Cinversa = node.serviceClient<camina6::CinversaParametros>("Cinversa");
    client_TransHomogenea = node.serviceClient<camina6::TransHomogeneaParametros>("TransHomogenea");

    std::string fileName("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina6/datos/SalidaX");
    std::string texto(".txt");
    fileName+=Id;
    fileName+=texto;
    fp1 = fopen(fileName.c_str(),"w+");

    std::string fileName2("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina6/datos/SalidaQ");
    fileName2+=Id;
    fileName2+=texto;
    fp2 = fopen(fileName2.c_str(),"w+");

    while (ros::ok() && simulationRunning)
    {
	  ros::spinOnce();
    }
    fclose(fp1);
    fclose(fp2);
    //ROS_INFO("Adios2!");
    ros::shutdown();
    return 0;
}

//---Caso parte 1 trayectoria----
//-- Periodo A-B
void Trayectoria_FaseApoyo(float t_Trayectoria,float PuntoInicio_x,float PuntoInicio_y,float PuntoInicio_z){

    float velocidadApoyo=0.0;

    velocidadApoyo = lambda_Apoyo/(beta*T);

    x_S0 = PuntoInicio_x + velocidadApoyo*t_Trayectoria;
    y_S0 = PuntoInicio_y;
    z_S0 = PuntoInicio_z;
//    if (fabs(t_Trayectoria-beta*T)<2*(delta_t)){
//        ROS_INFO("[%d] fin apoyo",Npata_arg);
//    }
}

//---Caso parte 2 trayectoria----
//-- Elipsis
void Trayectoria_FaseTrans_Eliptica(float t_Trayectoria,float PuntoInicio_x,float PuntoInicio_y,float PuntoInicio_z){

    float theta=0.0, t_aux=0.0;

    t_aux = (t_Trayectoria-beta*T)/((1-beta)*T);
    theta = pi*t_aux;

    x_S0 = PuntoInicio_x + (lambda_Transferencia/2)*cos(theta);
    y_S0 = PuntoInicio_y;
    z_S0 = PuntoInicio_z + dh*sin(theta);
    if (fabs(t_Trayectoria-T)<(delta_t)){
//        if (Npata_arg==1) ROS_INFO("Fin transferencia Pata[%d]",Npata_arg);
        finTransferencia = true;
        finTransferencia_x = x_S0;
        lambda_Apoyo = lambda_Transferencia;
//        ROS_INFO("Nodo2: Fin transferencia Pata[%d] - lambda_Apoyo=%.3f",Npata_arg,lambda_Transferencia);
    }
}
