#include "ros/ros.h"
#include "std_msgs/String.h"
#include "math.h"
//Librerias propias usadas
#include "constantes.hpp"
#include "camina/v_repConst.h"
// Used data structures:
#include <geometry_msgs/PoseStamped.h>
#include "../msg_gen/cpp/include/camina/AngulosMotor.h"
#include "../msg_gen/cpp/include/camina/DatosTrayectoriaPata.h"
#include "../msg_gen/cpp/include/camina/UbicacionRobot.h"
// Used API services:
#include "camina/TransTrayectoriaParametros.h"
#include "camina/TrayectoriaEliptica1Parametros.h"
#include "camina/CinversaParametros.h"
#include "vrep_common/VrepInfo.h"
#include "vrep_common/simRosGetObjectPose.h"
//Clientes y Servicios
ros::ServiceClient client_Cinversa;
camina::CinversaParametros srv_Cinversa;
ros::ServiceClient client_simRosGetObjectPose;
vrep_common::simRosGetObjectPose srv_simRosGetObjectPose; //Servicio para obtener posicion del robot
ros::ServiceClient client_TrayecEliptica1;
camina::TrayectoriaEliptica1Parametros srv_TrayecEliptica1;
ros::ServiceClient client_TransTrayectoria_MundoPata;
camina::TransTrayectoriaParametros srv_TransTrayectoria_MundoPata;
ros::ServiceClient client_TransTrayectoria_PataMundo;
camina::TransTrayectoriaParametros srv_TransTrayectoria_PataMundo;
// Funciones
void Trayectoria_FaseApoyo(float t_Trayectoria,float PuntoInicio_x,float PuntoInicio_y,float PuntoInicio_z);
void Trayectoria_FaseTrans_Eliptica(float t_Trayectoria,float PuntoInicio_x,float PuntoInicio_y,float PuntoInicio_z);

// variables Globales
ros::Publisher chatter_pub;
bool simulationRunning=true;
bool sensorTrigger=false;
float simulationTime=0.0f;
int Npata_arg=0;
float delta_t=0.0,beta=0.0,lambda_Apoyo=0.0,lambda_Transferencia=0.0,dh=0.0;
float x_Tray=0.0, y_Tray=0.0, z_Tray=0.0;
float x_FinApoyo=0.0,y_FinApoyo=0.0,z_FinApoyo=0.0;
bool finTrayectoria=false, Inicio=true;
float PosicionCuerpo_y=0.0, PosicionCuerpo_x=0.0, theta_CuerpoRobot=0.0;
float PuntoApoyo_Pata_x,PuntoApoyo_Pata_y,PuntoApoyo_Pata_z;
float PuntoApoyo_Mundo_x,PuntoApoyo_Mundo_y,PuntoApoyo_Mundo_z;
geometry_msgs::PoseStamped PataTipPose;
camina::AngulosMotor qMotor;
FILE *fp1;

// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}

void ubicacionRobCallback(camina::UbicacionRobot msgUbicacionRobot)
{
    PosicionCuerpo_x = msgUbicacionRobot.coordenadaCuerpo_x;
    PosicionCuerpo_y = msgUbicacionRobot.coordenadaCuerpo_y;
    theta_CuerpoRobot = msgUbicacionRobot.orientacionCuerpo_yaw;
}

void datosCallback(camina::DatosTrayectoriaPata msgDatosTrayectoriaPata){

    float t_Trayectoria=0.0, alfa=0.0, phi=0.0, desfasaje_t=0.0;
    float x_S0=0.0,y_S0=0.0,z_S0=0.0,x_S1=0.0,y_S1=0.0,z_S1=0.0;
    float x_Offset=0.0,y_Offset=0.0, z_Offset=0.0;

    beta = msgDatosTrayectoriaPata.beta;
    desfasaje_t = msgDatosTrayectoriaPata.desfasaje_t[Npata_arg-1];
    lambda_Apoyo = msgDatosTrayectoriaPata.lambda_Apoyo[Npata_arg-1];
    lambda_Transferencia = msgDatosTrayectoriaPata.lambda_Transferencia[Npata_arg-1];
    dh = msgDatosTrayectoriaPata.dh;
    x_Offset = msgDatosTrayectoriaPata.x_Offset;
    y_Offset = msgDatosTrayectoriaPata.y_Offset;
    z_Offset = msgDatosTrayectoriaPata.z_Offset;
    alfa = msgDatosTrayectoriaPata.alfa[Npata_arg-1];
    phi = msgDatosTrayectoriaPata.phi;
    //--- Temporizacion
    delta_t = 1/msgDatosTrayectoriaPata.divisionTrayectoriaPata;
    t_Trayectoria=msgDatosTrayectoriaPata.t_Trayectoria+desfasaje_t;
    t_Trayectoria=fmod(t_Trayectoria,1);

    if (finTrayectoria) {
        //-- Visto desde el mundo, el punto de apoyo es uno solo y es el mismo
        //-- ..de inicio de transferencia
        ROS_INFO("[%d] fin trayectoria",Npata_arg);
        finTrayectoria = false;
        if (client_simRosGetObjectPose.call(srv_simRosGetObjectPose)&&(srv_simRosGetObjectPose.response.result!=-1)){
            PataTipPose = srv_simRosGetObjectPose.response.pose;
            PuntoApoyo_Mundo_x = PataTipPose.pose.position.x;
            PuntoApoyo_Mundo_y = PataTipPose.pose.position.y;
//            PuntoApoyo_z = PataTipPose.pose.position.z;
            PuntoApoyo_Mundo_z = z_Offset;
//            ROS_INFO("actual: [%d] x=%.3f, y=%.3f, z=%.3f",Npata_arg,PuntoApoyo_x,PuntoApoyo_y,PuntoApoyo_z);
        } else {
            ROS_ERROR("Nodo1: servicio de posicion pata no funciona\n");
            ROS_ERROR("Nodo1: respuesta = %d\n",srv_simRosGetObjectPose.response.result);
        }
        //--- Antitransformamos la trayectoria hacia el sistema de la pata
        srv_TransTrayectoria_MundoPata.request.modo = Mundo_Pata;
        srv_TransTrayectoria_MundoPata.request.Npata = Npata_arg-1;
        srv_TransTrayectoria_MundoPata.request.x_S0 = PuntoApoyo_Mundo_x;
    //    srv_TransTrayectoria_MundoPata.request.y_S0 = y_Tray+0.02;
        srv_TransTrayectoria_MundoPata.request.y_S0 = PuntoApoyo_Mundo_y;
        srv_TransTrayectoria_MundoPata.request.z_S0 = PuntoApoyo_Mundo_z;
        srv_TransTrayectoria_MundoPata.request.x_UbicacionRob = PosicionCuerpo_x;
        srv_TransTrayectoria_MundoPata.request.y_UbicacionRob = PosicionCuerpo_y;
        srv_TransTrayectoria_MundoPata.request.theta_Rob = theta_CuerpoRobot;
        if (client_TransTrayectoria_MundoPata.call(srv_TransTrayectoria_MundoPata))
        {   PuntoApoyo_Pata_x = srv_TransTrayectoria_MundoPata.response.x_Pata;
            PuntoApoyo_Pata_y = srv_TransTrayectoria_MundoPata.response.y_Pata;
            PuntoApoyo_Pata_z = srv_TransTrayectoria_MundoPata.response.z_Pata;
        } else {
            ROS_ERROR("Nodo1: servicio de TransTrayectoria no funciona\n");
        }
    }
    if(Inicio){
        Inicio=false;
        ROS_INFO("Nodo1: Inicio Pata[%d], t=%.3f",Npata_arg,t_Trayectoria);
//        srv_TrayecEliptica1.request.x_Offset = x_Offset;
//        srv_TrayecEliptica1.request.y_Offset = y_Offset;
//        srv_TrayecEliptica1.request.z_Offset = z_Offset;
        srv_TrayecEliptica1.request.t_Trayectoria = t_Trayectoria;
        srv_TrayecEliptica1.request.beta = beta;
        srv_TrayecEliptica1.request.lambda_Apoyo = lambda_Apoyo;
        srv_TrayecEliptica1.request.lambda_Transferencia = lambda_Transferencia;
        srv_TrayecEliptica1.request.dh = dh;
        srv_TrayecEliptica1.request.alfa = alfa;
        srv_TrayecEliptica1.request.phi = phi;
        if (client_TrayecEliptica1.call(srv_TrayecEliptica1)){
            x_S0 = srv_TrayecEliptica1.response.x_S1;
            y_S0 = srv_TrayecEliptica1.response.y_S1;
            z_S0 = srv_TrayecEliptica1.response.z_S1;
//             ROS_INFO("inicio: [%d] x=%.3f\ty=%.3f\tz=%.3f",Npata_arg,x_S0,y_S0,z_S0);
//            if (Npata_arg==6) ROS_INFO("inicio: [%d] x=%.3f\ty=%.3f\tz=%.3f",Npata_arg,x_S0,y_S0,z_S0);
        } else {
            ROS_ERROR("Nodo1: servicio de TrayecEliptica1 no funciona\n");
        }
        x_S0 = -lambda_Apoyo/2;
        y_S0 = 0.0;
        z_S0 = 0.0;
        PuntoApoyo_Pata_x = x_S0;
        PuntoApoyo_Pata_y = y_S0;
        PuntoApoyo_Pata_z = z_Offset+z_S0;

        //---Se añade rotacion y traslacion a coordenadas encontradas---
        //---..se realiza traslacion a eje de pata
        //x en sistema 1
        x_S1 = x_Offset+x_S0*cos(alfa+phi)-y_S0*sin(alfa+phi);
        //y en sistema 1
        y_S1 = y_Offset+x_S0*sin(alfa+phi)+y_S0*cos(alfa+phi);
        //z en sistema 1
        z_S1 = z_Offset+z_S0;

        srv_TransTrayectoria_PataMundo.request.modo = Pata_Mundo;
        srv_TransTrayectoria_PataMundo.request.Npata = Npata_arg-1;
        srv_TransTrayectoria_PataMundo.request.x_S0 = x_S1;
        srv_TransTrayectoria_PataMundo.request.y_S0 = y_S1;
        srv_TransTrayectoria_PataMundo.request.z_S0 = z_S1;
        srv_TransTrayectoria_PataMundo.request.x_UbicacionRob = PosicionCuerpo_x;
        srv_TransTrayectoria_PataMundo.request.y_UbicacionRob = PosicionCuerpo_y;
        srv_TransTrayectoria_PataMundo.request.theta_Rob = theta_CuerpoRobot;
        if (client_TransTrayectoria_PataMundo.call(srv_TransTrayectoria_PataMundo))
        {   PuntoApoyo_Mundo_x = srv_TransTrayectoria_PataMundo.response.x_Mundo;
            PuntoApoyo_Mundo_y = srv_TransTrayectoria_PataMundo.response.y_Mundo;
            PuntoApoyo_Mundo_z = srv_TransTrayectoria_PataMundo.response.z_Mundo;
//             ROS_INFO("inicio: \n[%d] PuntoApoyo_x=%.3f\tPuntoApoyo_y=%.3f\tPuntoApoyo_z=%.3f",Npata_arg,PuntoApoyo_x,PuntoApoyo_y,PuntoApoyo_z);
        } else {
            ROS_ERROR("Nodo1: servicio de TransTrayectoria no funciona\n");
        }
    }   //Fin de inicio
    //--- Aqui obtenemos parametrizacion respecto al mundo
    if (0<=t_Trayectoria && t_Trayectoria<beta){

        Trayectoria_FaseApoyo(t_Trayectoria,PuntoApoyo_Pata_x,PuntoApoyo_Pata_y,PuntoApoyo_Pata_z);
        //---Se añade rotacion y traslacion a coordenadas encontradas---
        //---..se realiza traslacion a eje de pata
        //x en sistema 1
        x_S1 = x_Offset+x_Tray*cos(alfa+phi)-y_Tray*sin(alfa+phi);
        //y en sistema 1
        y_S1 = y_Offset+x_Tray*sin(alfa+phi)+y_Tray*cos(alfa+phi);
        //z en sistema 1
//        z_S1 = z_Offset+z_Tray;
        z_S1 = z_Tray;
    } else {

        Trayectoria_FaseTrans_Eliptica(t_Trayectoria,PuntoApoyo_Mundo_x,PuntoApoyo_Mundo_y,PuntoApoyo_Mundo_z);

        //--- Antitransformamos la trayectoria hacia el sistema de la pata
        srv_TransTrayectoria_MundoPata.request.modo = Mundo_Pata;
        srv_TransTrayectoria_MundoPata.request.Npata = Npata_arg-1;
        srv_TransTrayectoria_MundoPata.request.x_S0 = x_Tray;
        //    srv_TransTrayectoria_MundoPata.request.y_S0 = y_Tray+0.02;
        srv_TransTrayectoria_MundoPata.request.y_S0 = y_Tray;
        srv_TransTrayectoria_MundoPata.request.z_S0 = z_Tray;
        srv_TransTrayectoria_MundoPata.request.x_UbicacionRob = PosicionCuerpo_x;
        srv_TransTrayectoria_MundoPata.request.y_UbicacionRob = PosicionCuerpo_y;
        srv_TransTrayectoria_MundoPata.request.theta_Rob = theta_CuerpoRobot;
        if (client_TransTrayectoria_MundoPata.call(srv_TransTrayectoria_MundoPata))
        {   x_S1 = srv_TransTrayectoria_MundoPata.response.x_Pata;
            y_S1 = srv_TransTrayectoria_MundoPata.response.y_Pata;
            z_S1 = srv_TransTrayectoria_MundoPata.response.z_Pata;
        } else {
            ROS_ERROR("Nodo1: servicio de TransTrayectoria no funciona\n");
        }
    }
////         Creamos archivo
    fprintf(fp1,"%d,%.3f,%.3f,%.3f,%.3f\n",Npata_arg,t_Trayectoria,x_S1,y_S1,z_S1);
//    if (Npata_arg==1) ROS_INFO("x_Tray=%.3f\ty_Tray=%.3f\tz_Tray=%.3f",x_Tray,y_Tray,z_Tray);

    // Creamos archivo
//    fprintf(fp1,"%d,%.3f,%.3f,%.3f\n",Npata_arg,x_S1,y_S1,z_S1);
//        fprintf(fp1,"%d,%d,%.3f,%.3f,%.3f\n",Npata_arg,qMotor.iteracion,x_S0,y_S0,z_S0);
//    ROS_INFO("[%d] x_S1=%.3f\ty_S1=%.3f\tz_S1=%.3f,",Npata_arg,x_S1,y_S1,z_S1);
//    if (Npata_arg==1) ROS_INFO("[%d] x_S1=%.3f\ty_S1=%.3f\tz_S1=%.3f",Npata_arg,x_S1,y_S1,z_S1);
//    if (Npata_arg==2) ROS_INFO("[%d] x_S1=%.3f\ty_S1=%.3f\tz_S1=%.3f",Npata_arg,x_S1,y_S1,z_S1);

    //--- CINEMATICA INVERSA
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
        ROS_ERROR("Nodo 1::pata[%d]: servicio de Cinversa no funciona",Npata_arg);
//        ROS_WARN("Nodo2: result:%d\n",srv_Cinversa.response.result);
//        return;
    }
    //---Publica angulos motores----
	chatter_pub.publish(qMotor);
      //ROS_INFO("2-Tiempo:[Npata=%d, ite=%d, t_Tray= %.3f, t_Tramo= %.3f]", qMotor.Npata, qMotor.iteracion, t_Trayectoria, t_Tramo);
      //ROS_INFO("3-Posicion xyz:[Npata=%d, ite=%d, x = %.3f, y = %.3f, z = %.3f]", qMotor.Npata, qMotor.iteracion, coordenadasCartesianas[0], coordenadasCartesianas[1], coordenadasCartesianas[2]);
}


int main(int argc, char **argv){
  int Narg=0,PataTipHandle=0;

  Narg=5;
      // (when V-REP launches this executable, V-REP will also provide the argument list)
	//numero de argumentos que mande (se excluye el fantasma que el manda solo)
	if (argc>=Narg)
	{
	    Npata_arg=atoi(argv[1]);
		PosicionCuerpo_x=atof(argv[2]);
		PosicionCuerpo_y=atof(argv[3]);
		theta_CuerpoRobot=atof(argv[4]);
        PataTipHandle=atoi(argv[5]);
    } else {
		ROS_ERROR("Nodo1: Indique argumentos!\n");
		sleep(5000);
		return 0;
	}

    /*Inicio nodo de ROS*/
    std::string nodeName("Nodo1_Parametrizacion_pata");
	std::string Id(boost::lexical_cast<std::string>(Npata_arg));
	nodeName+=Id;
	ros::init(argc,argv,nodeName.c_str());
    ros::NodeHandle node;

    //Topicos susbcritos y publicados
    chatter_pub = node.advertise<camina::AngulosMotor>("DatosDeMotores", 100);
    ros::Subscriber subInfo0=node.subscribe("datosTrayectoriaPata",100,datosCallback);
    ros::Subscriber subInfo1=node.subscribe("/vrep/info",100,infoCallback);
    ros::Subscriber subInfo2=node.subscribe("UbicacionRobot",100,ubicacionRobCallback);
    //Clientes y Servicios
    client_Cinversa=node.serviceClient<camina::CinversaParametros>("Cinversa");
    client_simRosGetObjectPose=node.serviceClient<vrep_common::simRosGetObjectPose>("/vrep/simRosGetObjectPose");
    srv_simRosGetObjectPose.request.relativeToObjectHandle=-1;
    srv_simRosGetObjectPose.request.handle=PataTipHandle;
    client_TrayecEliptica1=node.serviceClient<camina::TrayectoriaEliptica1Parametros>("TrayectoriaEliptica1");
    client_TransTrayectoria_MundoPata=node.serviceClient<camina::TransTrayectoriaParametros>("TrayectoriaMundoPata");
    client_TransTrayectoria_PataMundo=node.serviceClient<camina::TransTrayectoriaParametros>("TrayectoriaMundoPata");

    std::string fileName("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina/datos/SalidaX");
    std::string texto(".txt");
    fileName+=Id;
    fileName+=texto;
    fp1 = fopen(fileName.c_str(),"w+");
    while (ros::ok() && simulationRunning){
            ros::spinOnce();
    }

//    ROS_INFO("Adios1!");
    fclose(fp1);
    ros::shutdown();
    return 0;
}

//---Caso parte 1 trayectoria----
// Periodo A-B
void Trayectoria_FaseApoyo(float t_Trayectoria,float PuntoInicio_x,float PuntoInicio_y,float PuntoInicio_z){

//    float x_aux=0.0, y_aux=0.0, z_aux=0.0;
    float velocidadApoyo=0.0;

    velocidadApoyo = lambda_Apoyo/beta;

//    x_Tray = (PuntoInicio_x+lambda_Apoyo/2) + velocidadApoyo*t_Trayectoria;
    x_Tray = PuntoInicio_x + velocidadApoyo*t_Trayectoria;
    y_Tray = PuntoInicio_y;
    z_Tray = PuntoInicio_z;
    if (fabs(t_Trayectoria-beta)<=(2*delta_t)){
//        ROS_INFO("[%d] fin apoyo",Npata_arg);
    }
}

//---Caso parte 2 trayectoria----
// Elipsis
void Trayectoria_FaseTrans_Eliptica(float t_Trayectoria,float PuntoInicio_x,float PuntoInicio_y,float PuntoInicio_z){

    float x_aux=0.0, y_aux=0.0, z_aux=0.0;
    float theta=0.0, t_aux=0.0;

    t_aux = (t_Trayectoria-beta)/(1-beta);
    theta = pi*(1-t_aux);

    x_aux = PuntoInicio_x;
    y_aux = (PuntoInicio_y-lambda_Transferencia/2) + (lambda_Transferencia/2)*cos(theta);
//    y_aux = PuntoInicio_y + (lambda_Transferencia/2)*cos(theta);
    z_aux = PuntoInicio_z + dh*sin(theta);
    if (fabs(t_Trayectoria-1)<=(2*delta_t)) finTrayectoria=true;

    x_Tray = x_aux;
    y_Tray = y_aux;
    z_Tray = z_aux;
}
