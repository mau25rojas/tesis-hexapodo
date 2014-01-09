#include "ros/ros.h"
//usando los valores de cada uno de los eslabones (definidos en "constantes.hpp")
#include "constantes.hpp"
#include "../msg_gen/cpp/include/camina/DatosTrayectoriaPata.h"
#include "../msg_gen/cpp/include/camina/AngulosMotor.h"
#include "vrep_common/VrepInfo.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>

#define pi 3.141592

// Global variables (modified by topic subscribers):
bool simulationRunning=true;
bool sensorTrigger=false;
float simulationTime=0.0f;

int Npata_arg=0;

void CalculoTrayectoria (float *p, float landa, float dh, float velocidadApoyo, float velocidadTransferencia, float t_Tramo, int seleccionTramo);
void CalculoCinInversa (float *p, float x, float y, float z);

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

void datosCallback(const camina::DatosTrayectoriaPata msg_datoTrayectoria)
{
  int seleccionTramo=0, i=0;
  float t_Trayectoria=0.0, t_Tramo=0.0, velocidadApoyo=0.0, velocidadTransferencia=0.0;
  float beta=0.0, landa=0.0, dh=0.0, desfasaje_t[6]={0.0, 0.0, 0.0,0.0, 0.0, 0.0};
  float x_Offset=0.0, y_Offset=0.0, z_Offset=0.0;   //matriz de transformacion homogenea
  float alfa_rad[6]={0.0, 0.0, 0.0,0.0, 0.0, 0.0},alfa_grd[6]={0.0, 0.0, 0.0,0.0, 0.0, 0.0}, x_S0=0.0, y_S0=0.0, z_S0=0.0;    //matriz de transformacion homogenea
  float coordenadasCartesianas[3] = {0.0, 0.0, 0.0};   //coordenadas X, Y, Z
  float q[3] = {0.0,0.0,0.0};	//Angulos de motores q1, q2, q3
  float *ptr1, *ptr2;   //Apuntadores a arreglos de angulos y coordenadas
  float t_TramoABC=0.0, t_TramoABCD=0.0;

	t_Trayectoria = msg_datoTrayectoria.t_Trayectoria;
	qMotor.iteracion=msg_datoTrayectoria.iteracion;
	beta=msg_datoTrayectoria.beta;
	landa=msg_datoTrayectoria.landa;
	dh=msg_datoTrayectoria.dh;
	x_Offset=msg_datoTrayectoria.p_Offset[0];
	y_Offset=msg_datoTrayectoria.p_Offset[1];
	z_Offset=msg_datoTrayectoria.p_Offset[2];
	for(i=0;i<6;i++) alfa_grd[i]=msg_datoTrayectoria.alfa[i];
    for(i=0;i<6;i++) desfasaje_t[i]=msg_datoTrayectoria.desfasaje_t[i];

	velocidadApoyo = landa/beta;
	velocidadTransferencia = (2*dh+landa)/(1-beta);
    /* Divisiones de tiempo trayectoria A-B-C-D-A*/
    t_TramoABC = (dh/velocidadTransferencia)+beta;
    t_TramoABCD = ((dh+landa)/velocidadTransferencia)+beta;
    //---------------------------------

    //--------Temporizacion----------
    t_Trayectoria=t_Trayectoria+desfasaje_t[Npata_arg-1];
    t_Trayectoria=fmod(t_Trayectoria,1);
    //---Rotacion para la trayectoria-----
    for(int i=0;i<6;i++) alfa_rad[i] = alfa_grd[i]*pi/180.0;
    //---Apuntadores a salidas----
    ptr1 = coordenadasCartesianas;
    ptr2 = q;

     /* Nota Mental:
        Prueba con trayectoria rectangular: existen 4 tramos/casos.
        Para cada tramo es necesario reiniciar el tiempo para pasarlo
        ..a la función que calcula la trayectoria (en base a la velocidad)
     */
    //---Caso parte 1 trayectoria----
    // Periodo A-B
    if (0<=t_Trayectoria && t_Trayectoria<beta)
        {
            t_Tramo = t_Trayectoria;
            seleccionTramo=0;
            CalculoTrayectoria(ptr1, landa, dh, velocidadApoyo, velocidadTransferencia, t_Tramo, seleccionTramo);
            qMotor.Npata = Npata_arg;
            //Variables auxiliares
            x_S0 = coordenadasCartesianas[0];  //x en sistema 0
            y_S0 = coordenadasCartesianas[1];  //y en sistema 0
            z_S0 = coordenadasCartesianas[2];  //z en sistema 0

            //---Se añade rotacion y traslacion a coordenadas encontradas---
            //x en sistema 1
            coordenadasCartesianas[0] = x_Offset+x_S0*cos(alfa_rad[Npata_arg-1])-y_S0*sin(alfa_rad[Npata_arg-1]);
            //y en sistema 1
            coordenadasCartesianas[1] = y_Offset+x_S0*sin(alfa_rad[Npata_arg-1])+y_S0*cos(alfa_rad[Npata_arg-1]);
            //z en sistema 1
            coordenadasCartesianas[2] = z_Offset+z_S0;

            CalculoCinInversa(ptr2,coordenadasCartesianas[0],coordenadasCartesianas[1],coordenadasCartesianas[2]);

            qMotor.q1 = q[0];
            qMotor.q2 = q[1];
            qMotor.q3 = q[2];
        }
    //---------------------------------

    //---Caso parte 2 trayectoria------
    // Periodo B-C
     if (beta<=t_Trayectoria && t_Trayectoria<t_TramoABC)
    	{
            t_Tramo = t_Trayectoria-beta;
            seleccionTramo=1;
            CalculoTrayectoria(ptr1, landa, dh, velocidadApoyo, velocidadTransferencia, t_Tramo, seleccionTramo);
            qMotor.Npata = Npata_arg;
            //Variables auxiliares
            x_S0 = coordenadasCartesianas[0];  //x en sistema 0
            y_S0 = coordenadasCartesianas[1];  //y en sistema 0
            z_S0 = coordenadasCartesianas[2];  //z en sistema 0

            //---Se añade rotacion y traslacion a coordenadas encontradas---
            //x en sistema 1
            coordenadasCartesianas[0] = x_Offset+x_S0*cos(alfa_rad[Npata_arg-1])-y_S0*sin(alfa_rad[Npata_arg-1]);
            //y en sistema 1
            coordenadasCartesianas[1] = y_Offset+x_S0*sin(alfa_rad[Npata_arg-1])+y_S0*cos(alfa_rad[Npata_arg-1]);
            //z en sistema 1
            coordenadasCartesianas[2] = z_Offset+z_S0;

            CalculoCinInversa(ptr2,coordenadasCartesianas[0],coordenadasCartesianas[1],coordenadasCartesianas[2]);

            qMotor.q1 = q[0];
            qMotor.q2 = q[1];
            qMotor.q3 = q[2];
        }
    //---------------------------------

    //---Caso parte 3 trayectoria------
    // Periodo C-D
     if (t_TramoABC<=t_Trayectoria && t_Trayectoria<t_TramoABCD)
    	{
            t_Tramo = t_Trayectoria-t_TramoABC;
            seleccionTramo=2;
            CalculoTrayectoria(ptr1, landa, dh, velocidadApoyo, velocidadTransferencia, t_Tramo, seleccionTramo);
            qMotor.Npata = Npata_arg;
            //Variables auxiliares
            x_S0 = coordenadasCartesianas[0];  //x en sistema 0
            y_S0 = coordenadasCartesianas[1];  //y en sistema 0
            z_S0 = coordenadasCartesianas[2];  //z en sistema 0

            //---Se añade rotacion y traslacion a coordenadas encontradas---
            //x en sistema 1
            coordenadasCartesianas[0] = x_Offset+x_S0*cos(alfa_rad[Npata_arg-1])-y_S0*sin(alfa_rad[Npata_arg-1]);
            //y en sistema 1
            coordenadasCartesianas[1] = y_Offset+x_S0*sin(alfa_rad[Npata_arg-1])+y_S0*cos(alfa_rad[Npata_arg-1]);
            //z en sistema 1
            coordenadasCartesianas[2] = z_Offset+z_S0;

            CalculoCinInversa(ptr2,coordenadasCartesianas[0],coordenadasCartesianas[1],coordenadasCartesianas[2]);

            qMotor.q1 = q[0];
            qMotor.q2 = q[1];
            qMotor.q3 = q[2];
        }
      //----------------------------------

    //---Caso parte 4 trayectoria---------
    //Periodo D-A
     if (t_TramoABCD<=t_Trayectoria && t_Trayectoria<1)
    	{
            t_Tramo = t_Trayectoria-t_TramoABCD;
            seleccionTramo=3;
            CalculoTrayectoria(ptr1, landa, dh, velocidadApoyo, velocidadTransferencia, t_Tramo, seleccionTramo);
            qMotor.Npata = Npata_arg;
            //Variables auxiliares
            x_S0 = coordenadasCartesianas[0];  //x en sistema 0
            y_S0 = coordenadasCartesianas[1];  //y en sistema 0
            z_S0 = coordenadasCartesianas[2];  //z en sistema 0

            //---Se añade rotacion y traslacion a coordenadas encontradas---
            //x en sistema 1
            coordenadasCartesianas[0] = x_Offset+x_S0*cos(alfa_rad[Npata_arg-1])-y_S0*sin(alfa_rad[Npata_arg-1]);
            //y en sistema 1
            coordenadasCartesianas[1] = y_Offset+x_S0*sin(alfa_rad[Npata_arg-1])+y_S0*cos(alfa_rad[Npata_arg-1]);
            //z en sistema 1
            coordenadasCartesianas[2] = z_Offset+z_S0;

            CalculoCinInversa(ptr2,coordenadasCartesianas[0],coordenadasCartesianas[1],coordenadasCartesianas[2]);

            qMotor.q1 = q[0];
            qMotor.q2 = q[1];
            qMotor.q3 = q[2];
        }
    //---------------------------------
//	fp = fopen("SalidaQ.txt","a");
//	   fprintf(fp,"%d,%.3f,%.3f,%.3f\n",qMotor.iteracion,q[0],q[1],q[2]);
//	fclose(fp);
//
//	fp1 = fopen("SalidaX.txt","a");
//	   fprintf(fp1,"%d,%.3f,%.3f,%.3f\n",qMotor.iteracion,coordenadasCartesianas[0],coordenadasCartesianas[1],coordenadasCartesianas[2]);
//	fclose(fp1);

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
		printf("Indique argumentos!\n");
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
    ros::Subscriber sub = n.subscribe("datosTrayectoriaPataSalida", 100, datosCallback);
    ros::Subscriber subInfo=n.subscribe("/vrep/info",1,infoCallback);

    while (ros::ok() && simulationRunning)
    {
	  ros::spinOnce();
    }
    //ROS_INFO("Adios2!");
    ros::shutdown();
    return 0;
}

/*Funcion de calculo de trayectoria

Parametros:
p: apuntador a la salida
landa: dlongitud de paso (metros)
dh: altura de levantamiento de pata (metros)
beta: intervalo de tiempo de apoyo (fraccion del periodo) - (normalizado entre 0-1)
t_Tramo: fraccion del periodo. Cada cuanto se actualiza la trayectoria - (normalizado entre 0-1)
*/

void CalculoTrayectoria (float *p, float landa, float dh, float velocidadApoyo, float velocidadTransferencia, float t_Tramo, int seleccionTramo){

float coordenadaZ=0.0, coordenadaX=0.0;

    switch (seleccionTramo){

    case 0:

        coordenadaX = -landa/2 + velocidadApoyo*t_Tramo ;
        coordenadaZ = -dh/2;

    break;

    case 1:

        coordenadaX = landa/2;
        coordenadaZ = - dh/2 + velocidadTransferencia*t_Tramo;

    break;

    case 2:

        coordenadaX = landa/2 - velocidadTransferencia*t_Tramo;
        coordenadaZ = dh/2;

    break;

    case 3:

        coordenadaX = -landa/2;
        coordenadaZ = dh/2 - velocidadTransferencia*t_Tramo;

    break;
    }

    p[0] = coordenadaX;
    p[1] = 0.0;
    p[2] = coordenadaZ;
}

/*Funcion de calculo de cinematica inversa

Parametros:
p: apuntador a la salida.
px: coordenada x de la punta de la pata respecto al inicio de la pata [m].
py: coordenada y de la punta de la pata respecto al inicio de la pata [m].
pz: coordenada z de la punta de la pata respecto al inicio de la pata [m].
*/

void CalculoCinInversa (float *p, float px, float py, float pz){

	float L23=0, L23_aux=0, beta=0, teta=0, gamma=0, arg=0, arg1=0;

    //------calculo q1------
    //p[0] = atan2(px,py);
    p[0] = atan2(py,px);
    //----------------------

    L23_aux = sqrt(px*px + py*py) - L1;
    L23 = sqrt(L23_aux*L23_aux + pz*pz);

    //------calculo q3------
	arg = (L2*L2 + L3*L3 - L23*L23)/(2*L2*L3);
	if (fabs(arg)>=1) {
        ROS_ERROR("ERROR acos q3");
        return;
	}
	beta = acos(arg);
    //El ajuste de pi se hace para coincidir con eje de D-H
    //p[2] = pi - beta;
    p[2] = beta;
    //----------------------

    //------calculo q2------
	arg1= (L3/L23)*sin(beta);
	if (fabs(arg1)>=1) {
        ROS_ERROR("ERROR asin q2");
        return;
	}
	teta = atan(-pz/L23_aux);
	gamma = asin(arg1);
	//El ajuste de pi/2 se hace para coincidir con eje de D-H
    //p[1] = pi/2 - (teta-gamma);
    p[1] = teta-gamma;
    //----------------------
}


