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
#include "../msg_gen/cpp/include/camina/DatosTrayectoriaPata.h"
#include "../msg_gen/cpp/include/camina/AngulosMotor.h"
#include "../srv_gen/cpp/include/camina/CinversaParametros.h"
#include "../srv_gen/cpp/include/camina/TransHomogeneaParametros.h"
// Used API services:
#include "vrep_common/VrepInfo.h"
//Clientes y Servicios
ros::ServiceClient client_Cinversa;
camina::CinversaParametros srv_Cinversa;
ros::ServiceClient client_TransHomogenea;
camina::TransHomogeneaParametros srv_TransHomogenea;

// Global variables (modified by topic subscribers):
bool simulationRunning=true;
bool sensorTrigger=false;
float simulationTime=0.0f;
int Npata_arg=0, PataPrint=1;
//-- Calculo de trayectoria
float beta[Npatas], lambda_Apoyo[Npatas],lambda_Transferencia[Npatas], dh=0.0, desfasaje_t[6];
float x_S0=0.0, y_S0=0.0, z_S0=0.0;
//float coordenadasCartesianas[3] = {0.0, 0.0, 0.0}, *ptr_x;   //Apuntadores a arreglos de angulos y coordenadas
camina::AngulosMotor qMotor;
ros::Publisher chatter_pub;
FILE *fp, *fp1;
// Funciones
void CalculoTrayectoria_Cuadrada (int Npata, float t_Trayectoria);
void CalculoTrayectoria_Eliptica (int Npata, float t_Trayectoria);
//void CalculoTrayectoria (int Npata,float velocidadApoyo, float velocidadTransferencia, float t_Tramo, int seleccionTramo, int TipoTrayectoria);
//void CalculoCinInversa (float *p, float x, float y, float z);

// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}

/* Callback que escucha el topico DatosDeTrayectoria calcula la trayectoria
   deseaday cinematica inversa para de motores y los publica
*/
void datosCallback(const camina::DatosTrayectoriaPata msg_datoTrayectoria)
{
    float t_Trayectoria=0.0;
    float alfa_rad[Npatas], phi_rad=0.0; //matriz de transformacion homogenea
    float x_Offset=0.0, y_Offset=0.0, z_Offset=0.0;   //matriz de transformacion homogenea
    float x_S1=0.0, y_S1=0.0, z_S1=0.0;
//    float q[3] = {0.0,0.0,0.0};	//Angulos de motores q1, q2, q3
	t_Trayectoria = msg_datoTrayectoria.t_Trayectoria;
	qMotor.iteracion=msg_datoTrayectoria.iteracion;
	dh=msg_datoTrayectoria.dh;
	x_Offset=msg_datoTrayectoria.p_Offset[0];
	y_Offset=msg_datoTrayectoria.p_Offset[1];
	z_Offset=msg_datoTrayectoria.p_Offset[2];
	phi_rad = msg_datoTrayectoria.phi*pi/180.0;
    for(int i=0;i<Npatas;i++) {
        beta[i]=msg_datoTrayectoria.beta[i];
        lambda_Apoyo[i]=msg_datoTrayectoria.lambda_Apoyo[i];
        lambda_Transferencia[i]=msg_datoTrayectoria.lambda_Transferencia[i];
        alfa_rad[i]=msg_datoTrayectoria.alfa[i]*pi/180.0;
        desfasaje_t[i]=msg_datoTrayectoria.desfasaje_t[i];
    }
    //---------------------------------
    qMotor.Npata = Npata_arg;
    //--------Temporizacion----------
    t_Trayectoria=t_Trayectoria+desfasaje_t[Npata_arg-1];
    t_Trayectoria=fmod(t_Trayectoria,1);
    //-----Parametrizacion de trayectoria eliptica en Sistema de Robot
//    CalculoTrayectoria_Cuadrada (Npata_arg-1,t_Trayectoria);
    CalculoTrayectoria_Eliptica (Npata_arg-1,t_Trayectoria);
    //-----Transformacion de trayectoria a Sistema de Pata
    srv_TransHomogenea.request.x_S0 = x_S0;
    srv_TransHomogenea.request.y_S0 = y_S0;
    srv_TransHomogenea.request.z_S0 = z_S0;
    srv_TransHomogenea.request.x_Trasl = x_Offset;
    srv_TransHomogenea.request.y_Trasl = y_Offset;
    srv_TransHomogenea.request.z_Trasl = z_Offset;
    srv_TransHomogenea.request.tita_Rot = alfa_rad[Npata_arg-1]+phi_rad;
    if (client_TransHomogenea.call(srv_TransHomogenea))
    {   x_S1 = srv_TransHomogenea.response.x_S1;
        y_S1 = srv_TransHomogenea.response.y_S1;
        z_S1 = srv_TransHomogenea.response.z_S1;
//                ROS_INFO("Servicio motores: q1=%.3f; q2=%.3f; q3=%.3f\n", _q1, _q2, _q3);
    } else {
        ROS_ERROR("Nodo 2: servicio de TransHomogenea no funciona\n");
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
//                ROS_INFO("Servicio motores: q1=%.3f; q2=%.3f; q3=%.3f\n", _q1, _q2, _q3);
    } else {
        ROS_ERROR("Nodo 2: servicio de Cinversa no funciona\n");
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
    ros::NodeHandle n;
    //ROS_INFO("Nodo2_Parametrizacion just started\n");

    //Topicos susbcritos y publicados
    chatter_pub = n.advertise<camina::AngulosMotor>("DatosDeMotores", 100);
//    ros::Subscriber sub = n.subscribe("datosTrayectoriaPataSalida", 100, datosCallback);
    ros::Subscriber sub = n.subscribe("datosTrayectoriaPataEntrada", 100, datosCallback);
    ros::Subscriber subInfo=n.subscribe("/vrep/info",1,infoCallback);
    //Clientes y Servicios
    client_Cinversa=n.serviceClient<camina::CinversaParametros>("Cinversa");
    client_TransHomogenea=n.serviceClient<camina::TransHomogeneaParametros>("TransHomogenea");

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

void CalculoTrayectoria_Cuadrada (int Npata, float t_Trayectoria) {

//    int seleccionTramo=0;
    float velocidadApoyo=0.0, velocidadTransferencia=0.0;
    float t_TramoABC=0.0, t_TramoABCD=0.0, t_Tramo=0.0;
//    float x_S0=0.0, y_S0=0.0, z_S0=0.0;   //coordenadas X, Y, Z

    velocidadApoyo = lambda_Apoyo[Npata]/beta[Npata];
	velocidadTransferencia = (2*dh+lambda_Apoyo[Npata])/(1-beta[Npata]);
    /* Divisiones de tiempo trayectoria A-B-C-D-A*/
    t_TramoABC = (dh/velocidadTransferencia)+beta[Npata];
    t_TramoABCD = ((dh+lambda_Apoyo[Npata])/velocidadTransferencia)+beta[Npata];

     /* Nota Mental:
        Prueba con trayectoria rectangular: existen 4 tramos/casos.
        Para cada tramo es necesario reiniciar el tiempo para pasarlo
        ..a la función que calcula la trayectoria (en base a la velocidad)
     */
    //---Caso parte 1 trayectoria----
    // Periodo A-B
    if (0<=t_Trayectoria && t_Trayectoria<beta[Npata])
        {
            t_Tramo = t_Trayectoria;
//            seleccionTramo=0;
//            CalculoTrayectoria(Npata,velocidadApoyo,velocidadTransferencia,t_Tramo,seleccionTramo,TrayectoriaCuadrada);
//            //Variables auxiliares
//            x_S0 = coordenadasCartesianas[0];  //x en sistema 0
//            x_S0 = coordenadasCartesianas[1];  //y en sistema 0
//            z_S0 = coordenadasCartesianas[2];  //z en sistema 0
            x_S0 = -lambda_Apoyo[Npata]/2 + velocidadApoyo*t_Tramo;
            y_S0 =0.0;
            z_S0 = -dh/2;
        }
    //---Caso parte 2 trayectoria------
    // Periodo B-C
     if (beta[Npata]<=t_Trayectoria && t_Trayectoria<t_TramoABC)
    	{
            t_Tramo = t_Trayectoria-beta[Npata];

            x_S0 = lambda_Apoyo[Npata]/2;
            y_S0 = 0.0;
            z_S0 = -dh/2 + velocidadTransferencia*t_Tramo;
        }
    //---Caso parte 3 trayectoria------
    // Periodo C-D
     if (t_TramoABC<=t_Trayectoria && t_Trayectoria<t_TramoABCD)
    	{
            t_Tramo = t_Trayectoria-t_TramoABC;

            x_S0 = lambda_Apoyo[Npata]/2 - velocidadTransferencia*t_Tramo;
            y_S0 = 0.0;
            z_S0 = dh/2;
        }
    //---Caso parte 4 trayectoria---------
    //Periodo D-A
     if (t_TramoABCD<=t_Trayectoria && t_Trayectoria<1)
    	{
            t_Tramo = t_Trayectoria-t_TramoABCD;

            x_S0 = -lambda_Apoyo[Npata]/2;
            y_S0 = 0.0;
            z_S0 = dh/2 - velocidadTransferencia*t_Tramo;
        }
}

void CalculoTrayectoria_Eliptica (int Npata, float t_Trayectoria) {

//    int seleccionTramo=0;
    float velocidadApoyo=0.0;//, velocidadTransferencia=0.0;
    float t_Tramo=0.0;
//    float x_S0=0.0, y_S0=0.0, z_S0=0.0;   //coordenadas X, Y, Z
    float alfa_arg=0.0, t_aux=0.0;
    float delta_lambda = 0.0;

    velocidadApoyo = lambda_Apoyo[Npata]/beta[Npata];
    t_Tramo = t_Trayectoria;
    delta_lambda = fabs(lambda_Transferencia[Npata]-lambda_Apoyo[Npata])/2;
     /* Nota Mental:
        Existen 2 partes, la parte de apoyo que es una linea
        recta, y la eliptica que es el arco descrito en el
        aire
     */
    //---Caso parte 1 trayectoria----
    // Periodo A-B
    if (0<=t_Trayectoria && t_Trayectoria<beta[Npata])
        {
            x_S0 = (-lambda_Apoyo[Npata]/2 + velocidadApoyo*t_Tramo) - delta_lambda;
            y_S0 = 0.0;
            z_S0 = 0.0;
        }
    //---Caso parte 2 trayectoria------
    // Elipsis
        if (beta[Npata]<=t_Trayectoria)
        {
            t_aux = (t_Tramo-beta[Npata])/(1-beta[Npata]);
            alfa_arg = pi*t_aux;
//                    a = lambda_Transferencia/2;
//                    b = dh;
//                    x = a*cos(alfa);
//                    z = b*sin(alfa);
            x_S0 = lambda_Transferencia[Npata]/2*cos(alfa_arg);
            y_S0 = 0.0;
            z_S0 = dh*sin(alfa_arg);
        }
//            //---Se añade rotacion y traslacion a coordenadas encontradas---
//            //---..se realiza traslacion a eje de pata
//            //x en sistema 1
//            coordenadasCartesianas[0] = x_Offset+x_S0*cos(alfa_rad[Npata]+phi_rad)-y_S0*sin(alfa_rad[Npata]+phi_rad);
//            //y en sistema 1
//            coordenadasCartesianas[1] = y_Offset+x_S0*sin(alfa_rad[Npata]+phi_rad)+y_S0*cos(alfa_rad[Npata]+phi_rad);
//            //z en sistema 1
//            coordenadasCartesianas[2] = z_Offset+z_S0;


}

/*Funcion de calculo de trayectoria

Parametros:
p: apuntador a la salida
lambda: dlongitud de paso (metros)
dh: altura de levantamiento de pata (metros)
beta: intervalo de tiempo de apoyo (fraccion del periodo) - (normalizado entre 0-1)
t_Tramo: fraccion del periodo. Cada cuanto se actualiza la trayectoria - (normalizado entre 0-1)
*/

//void CalculoTrayectoria (int Npata,float velocidadApoyo, float velocidadTransferencia, float t_Tramo, int seleccionTramo, int TipoTrayectoria){
//
//float coordenadaZ=0.0, coordenadaX=0.0, alfa_arg=0.0, t_aux=0.0;
//
//    switch (TipoTrayectoria){
//
//        case TrayectoriaCuadrada:
//            switch (seleccionTramo){
//
//                case 0:
//                    coordenadaX = -lambda_Apoyo[Npata]/2 + velocidadApoyo*t_Tramo;
//                    coordenadaZ = -dh/2;
//                break;
//
//                case 1:
//                    coordenadaX = lambda_Apoyo[Npata]/2;
//                    coordenadaZ = - dh/2 + velocidadTransferencia*t_Tramo;
//                break;
//
//                case 2:
//                    coordenadaX = lambda_Apoyo[Npata]/2 - velocidadTransferencia*t_Tramo;
//                    coordenadaZ = dh/2;
//                break;
//
//                case 3:
//                    coordenadaX = -lambda_Apoyo[Npata]/2;
//                    coordenadaZ = dh/2 - velocidadTransferencia*t_Tramo;
//                break;
//
//                default:
//                    ROS_ERROR("Nodo2: Error en trayectoria");
//            }
//
//        case TrayectoriaEliptica:
//
//            switch (seleccionTramo){
//                case 0:
//                    coordenadaX = -lambda_Apoyo[Npata]/2 + velocidadApoyo*t_Tramo;
//                    coordenadaZ = 0;
//                break;
//
//                case 1:
//                    t_aux = (t_Tramo-beta[Npata])/(1-beta[Npata]);
//                    alfa_arg = pi*t_aux;
////                    a = lambda_Transferencia/2;
////                    b = dh;
////                    x = a*cos(alfa);
////                    z = b*sin(alfa);
//                    coordenadaX = lambda_Transferencia[Npata]/2*cos(alfa_arg);
//                    coordenadaZ = dh*sin(alfa_arg);
//                break;
//
//                default:
//                    ROS_ERROR("Nodo2: Error en trayectoria");
//            }
//    }
//    coordenadasCartesianas[0] = coordenadaX;
//    coordenadasCartesianas[1] = 0.0;
//    coordenadasCartesianas[2] = coordenadaZ;
//}

/*Funcion de calculo de cinematica inversa

Parametros:
p: apuntador a la salida.
px: coordenada x de la punta de la pata respecto al inicio de la pata [m].
py: coordenada y de la punta de la pata respecto al inicio de la pata [m].
pz: coordenada z de la punta de la pata respecto al inicio de la pata [m].
*/

//void CalculoCinInversa (float *p, float px, float py, float pz){
//
//	float L23=0, L23_aux=0, beta=0, teta=0, gamma=0, arg=0, arg1=0;
//
//    //------calculo q1------
//    //p[0] = atan2(px,py);
//    p[0] = atan2(py,px);
//    //----------------------
//
//    L23_aux = sqrt(px*px + py*py) - L1;
//    L23 = sqrt(L23_aux*L23_aux + pz*pz);
//
//    //------calculo q3------
//	arg = (L2*L2 + L3*L3 - L23*L23)/(2*L2*L3);
//	if (fabs(arg)>=1) {
//        ROS_ERROR("ERROR acos q3");
//        return;
//	}
//	beta = acos(arg);
//    //El ajuste de pi se hace para coincidir con eje de D-H
//    //p[2] = pi - beta;
//    p[2] = beta;
//    //----------------------
//
//    //------calculo q2------
//	arg1= (L3/L23)*sin(beta);
//	if (fabs(arg1)>=1) {
//        ROS_ERROR("ERROR asin q2");
//        return;
//	}
//	teta = atan(-pz/L23_aux);
//	gamma = asin(arg1);
//	//El ajuste de pi/2 se hace para coincidir con eje de D-H
//    //p[1] = pi/2 - (teta-gamma);
//    p[1] = teta-gamma;
//    //----------------------
//}


