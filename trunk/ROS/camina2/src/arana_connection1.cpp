#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
//Librerias propias usadas
//#include <camina2/pata.h>
#include "constantes.hpp"
#include "../util/comm/comm.hpp"
#include "camina2/v_repConst.h"
// Used data structures:
#include <camina2/AnguloParaVrep.h>
#include <camina2/commData.h>
// Used API services:
#include "vrep_common/VrepInfo.h"
// Definicion de las variables globales a utilizar y de los nodos de comunicación
#define LIM_SUP_Q1 2400
#define LIM_INF_Q1 600
#define LIM_SUP_Q2 1900
#define LIM_INF_Q2 600
#define LIM_SUP_Q3 2400
#define LIM_INF_Q3 1550
#define OFFSET_Q1 1500.0
#define OFFSET_Q2 1500.0
#define OFFSET_Q3 1500.0
// Variables globales
bool simulationRunning=true;
bool sensorTrigger=false;
float simulationTime=0.0f;
ros::Publisher salida_serial;
ros::Subscriber q_calculados;
unsigned int Posiciones[18];
bool move[18];
int pata = 0;

// Funciones
void tramaEnviar();

// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}
// Callback
void recepcion_qCallback(camina2::AnguloParaVrep posicion)
{
		if (Posiciones[posicion.Npata*3+0] != (unsigned int) OFFSET_Q1 + posicion.Q1*2000.0/180.0)
	{
		Posiciones[posicion.Npata*3+0] = (unsigned int) OFFSET_Q1 + posicion.Q1*2000.0/180.0;
		if (Posiciones[posicion.Npata*3+0] < LIM_INF_Q1)	Posiciones[posicion.Npata*3+0] = LIM_INF_Q1;
		if (Posiciones[posicion.Npata*3+0] > LIM_SUP_Q1)	Posiciones[posicion.Npata*3+0] = LIM_SUP_Q1;
		move[posicion.Npata*3+0] = true;
	}
	if (Posiciones[posicion.Npata*3+1] != (unsigned int) OFFSET_Q2 + posicion.Q2*2000.0/180.0)
	{
        Posiciones[posicion.Npata*3+1] = (unsigned int) OFFSET_Q2 + posicion.Q2*2000.0/180.0;
        if (Posiciones[posicion.Npata*3+1] < LIM_INF_Q2)	Posiciones[posicion.Npata*3+1] = LIM_INF_Q2;
        if (Posiciones[posicion.Npata*3+1] > LIM_SUP_Q2)	Posiciones[posicion.Npata*3+1] = LIM_SUP_Q2;
        move[posicion.Npata*3+1] = true;
	}
	if (Posiciones[posicion.Npata*3+2] != (unsigned int) OFFSET_Q3 + posicion.Q3*2000.0/180.0)
	{
        Posiciones[posicion.Npata*3+2] = (unsigned int) OFFSET_Q3 + posicion.Q3*2000.0/180.0;
        if (Posiciones[posicion.Npata*3+2] < LIM_INF_Q3)	Posiciones[posicion.Npata*3+2] = LIM_INF_Q3;
        if (Posiciones[posicion.Npata*3+2] > LIM_SUP_Q3)	Posiciones[posicion.Npata*3+2] = LIM_SUP_Q3;
		move[posicion.Npata*3+2] = true;
	}
}

int main(int argc, char **argv) //Inicia el Manejo de cada uno de los nodos
{
	ros::init(argc, argv, "arana_connection");
	ros::NodeHandle n;
	ros::Subscriber subInfo1=n.subscribe("/vrep/info",1,infoCallback);
	salida_serial = n.advertise<camina2::commData>("COMM_MSG_TX", 50);
	q_calculados = n.subscribe("arana/leg_position", 100, recepcion_qCallback);
	ros::Rate loop_rate(120);
//    ros::Rate loop_rate(1);
	for(int i=0;i<18;i++)
		move[i] = true;
	for (int i=0;i<6;i++)
	{
		Posiciones[i*3] = OFFSET_Q1;
		Posiciones[i*3+1] = OFFSET_Q2;
		Posiciones[i*3+2] = OFFSET_Q3;
	}
	for (int i=0;i<6;i++) tramaEnviar();


	while (ros::ok() && simulationRunning)
  	{
		for (int i=0;i<100;i++)
			ros::spinOnce();
//		ROS_INFO("Hola arana_connection\n");
		tramaEnviar();
		loop_rate.sleep();
	}
    ROS_INFO("Adios arana_conection1!");
	ros::shutdown();
	return(0);
}

/*	La siguiente funcion se utiliza para el envio de la trama hacia el nodo serial que realizara la comunicacion	*/
void tramaEnviar(){

	std::stringstream buffer;
	std::string trama_envio;
	camina2::commData trama;

	if (pata == 0)
	{
		for(int i=0; i<3;i++)//Pata 1
			if(move[i] == true)
			{
				buffer << "#" << i << " P" << Posiciones[i] << " ";
			}
		if(move[0] == true or move[1] == true or move[2] == true)
			buffer << "\r";
		for(int i=0; i<3;i++)
			move[i] = false;
	}

	if (pata == 1)
	{
		for(int i=0; i<3;i++)//Pata 3
			if(move[i+6] == true)
			{
				buffer << "#" << i+4 << " P" << Posiciones[i+6] << " ";
			}
		if(move[6] == true or move[7] == true or move[8] == true)
			buffer << "\r";
		for(int i=0; i<3;i++)
			move[i+6] = false;

	}

	if (pata == 2)
	{
		for(int i=0; i<3;i++)//Pata 5
			if(move[i+12] == true)
			{
				buffer << "#" << i+8 << " P" << Posiciones[i+12] << " ";
			}
		if(move[12] == true or move[13] == true or move[14] == true)
			buffer << "\r";
		for(int i=0; i<3;i++)
			move[i+12] = false;

	}

	if (pata == 3)
	{
		for(int i=0; i<3;i++)//Pata 2
			if(move[i+3] == true)
			{
				buffer << "#" << i+16 << " P" << Posiciones[i+3] << " ";
			}
		if(move[3] == true or move[4] == true or move[5] == true)
			buffer << "\r";
		for(int i=0; i<3;i++)
			move[i+3] = false;

	}

	if (pata == 4)
	{
		for(int i=0; i<3;i++)//Pata 4
			if(move[i+9] == true)
			{
				buffer << "#" << i+20 << " P" << Posiciones[i+9] << " ";
			}
		if(move[9] == true or move[10] == true or move[11] == true)
			buffer << "\r";
		for(int i=0; i<3;i++)
			move[i+9] = false;

	}

	if (pata == 5)
	{
		for(int i=0; i<3;i++)//Pata 6
			if(move[i+15] == true)
			{
				buffer << "#" << i+24 << " P" << Posiciones[i+15] << " ";
			}
		if(move[15] == true or move[16] == true or move[17] == true)
			buffer << "\r";
		for(int i=0; i<3;i++)
			move[i+15] = false;

	}

	trama_envio = buffer.str();
	if (trama_envio.length() > 1)
		std::cout << trama_envio << std::endl;
	for(unsigned int i=0; i<trama_envio.length(); i++)
	{
		trama.data_hexapodo = trama_envio[i];
		if (trama_envio.length() > 1){
//		    ROS_INFO("arana_connection: Envio trama\n");
			salida_serial.publish(trama);
		}
	}
	if (pata <6)
		pata++;
	else
		pata = 0;
}
