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
unsigned int Posiciones[Nmotores];
bool MueveMotor[Nmotores], CambioNuevo;
int canales_Motores[Nmotores] = {0,1,2,16,17,18,4,5,6,20,21,22,8,9,10,24,25,26};

// Funciones
void EnviarTrama();

// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}

void recepcion_qCallback(camina2::AnguloParaVrep posicion)
{
//	ROS_INFO("arana_connection: Posicion Recibida - pata[%d]",posicion.Npata);

	if (Posiciones[posicion.Npata*3+0] != (unsigned int) OFFSET_Q1 + posicion.Q1*2000.0/180.0)
	{
	    CambioNuevo = true;
		Posiciones[posicion.Npata*3+0] = (unsigned int) OFFSET_Q1 + posicion.Q1*2000.0/180.0;
		if (Posiciones[posicion.Npata*3+0] < LIM_INF_Q1)	Posiciones[posicion.Npata*3+0] = LIM_INF_Q1;
		if (Posiciones[posicion.Npata*3+0] > LIM_SUP_Q1)	Posiciones[posicion.Npata*3+0] = LIM_SUP_Q1;
		MueveMotor[posicion.Npata*3+0] = true;
	}
	if (Posiciones[posicion.Npata*3+1] != (unsigned int) OFFSET_Q2 + posicion.Q2*2000.0/180.0)
	{
	    CambioNuevo = true;
        Posiciones[posicion.Npata*3+1] = (unsigned int) OFFSET_Q2 + posicion.Q2*2000.0/180.0;
        if (Posiciones[posicion.Npata*3+1] < LIM_INF_Q2)	Posiciones[posicion.Npata*3+1] = LIM_INF_Q2;
        if (Posiciones[posicion.Npata*3+1] > LIM_SUP_Q2)	Posiciones[posicion.Npata*3+1] = LIM_SUP_Q2;
        MueveMotor[posicion.Npata*3+1] = true;
	}
	if (Posiciones[posicion.Npata*3+2] != (unsigned int) OFFSET_Q3 + posicion.Q3*2000.0/180.0)
	{
	    CambioNuevo = true;
        Posiciones[posicion.Npata*3+2] = (unsigned int) OFFSET_Q3 + posicion.Q3*2000.0/180.0;
        if (Posiciones[posicion.Npata*3+2] < LIM_INF_Q3)	Posiciones[posicion.Npata*3+2] = LIM_INF_Q3;
        if (Posiciones[posicion.Npata*3+2] > LIM_SUP_Q3)	Posiciones[posicion.Npata*3+2] = LIM_SUP_Q3;
		MueveMotor[posicion.Npata*3+2] = true;
	}
}


int main(int argc, char **argv)
{
    float periodo=0.0, f=0.0;

	ros::init(argc, argv, "arana_connection");
	ROS_INFO("Inicia arana_connection");
	ros::NodeHandle n;
	//Subscripciones y publicaciones
	ros::Subscriber subInfo1=n.subscribe("/vrep/info",1,infoCallback);
	ros::Subscriber q_calculados = n.subscribe("arana/leg_position", 100, recepcion_qCallback);
	salida_serial = n.advertise<camina2::commData>("COMM_MSG_TX", 50);

    periodo=5;
    f=1/periodo;
    ros::Rate loop_rate(120);  //Frecuencia [Hz]

	int i=0;

    CambioNuevo = true;
	for(i=0;i<Nmotores;i++) MueveMotor[i] = true;
	for(i=0;i<Npatas;i++){
		Posiciones[i*3] = OFFSET_Q1;
		Posiciones[i*3+1] = OFFSET_Q2;
		Posiciones[i*3+2] = OFFSET_Q3;
	}
	EnviarTrama();
//    int cuenta=0;
	while (ros::ok() && simulationRunning)
  	{
		for (int i=0;i<100;i++)
			ros::spinOnce();
//		for(i=0;i<Nmotores;i++) MueveMotor[i] = true;
        loop_rate.sleep();
        EnviarTrama();
	}
    ROS_INFO("Adios arana_conection!");
	ros::shutdown();
	return(0);
}

/*	EnviarTrama: contruye la trama con las posiciones de los motores y si requieren moverse las envía	*/
void EnviarTrama(){

	std::stringstream buffer;
	std::string trama_envio;
	camina2::commData trama;
	int i=0;
	unsigned int j=0;

    if(CambioNuevo){
        for(i=0;i<Nmotores;i++){
            if (MueveMotor[i]) {
                buffer << "#" << canales_Motores[i] << " P" << Posiciones[i] << " ";
                MueveMotor[i] = false;
            }
        }
        buffer << "\r";
        trama_envio = buffer.str();
//        std::cout << "trama #char: " << trama_envio.length() << "\n";
        if (trama_envio.length() > 1){
//            ROS_INFO("arana_connection: Envio trama");
            std::cout << trama_envio << std::endl;
            for(j=0; j<trama_envio.length(); j++){
                trama.data_hexapodo = trama_envio[j];
                salida_serial.publish(trama);
            }
        }
        CambioNuevo = false;
    } else {
//        ROS_INFO("arana_connection: No hubo cambios");
    }
}

///*	EnviarTrama: contruye la trama con las posiciones de los motores y si requieren moverse las envía	*/
//void EnviarTrama(int pata){
//
//	std::stringstream buffer;
//	std::string trama_envio;
//	camina2::commData trama;
//	int i=0;
//	unsigned int j=0;
//
//        if (pata==Pata1){
//        //Pata 1
//            for(i=0; i<NmotorP;i++){
//                if(MueveMotor[i+0*NmotorP]) buffer << "#" << canal_Pata1+i << " P" << Posiciones[0*NmotorP] << " ";
//            }
//            if(MueveMotor[0] or MueveMotor[1] or MueveMotor[2]){
//                buffer << "\r";
//                trama_envio = buffer.str();
//                std::cout << "Pata1 #char: " << trama_envio.length() << "\n";
//                if (trama_envio.length() > 1){
//                    ROS_INFO("arana_connection: Envio trama pata1");
//                    std::cout << trama_envio << std::endl;
//                    for(j=0; j<trama_envio.length(); j++){
//                        trama.data_hexapodo = trama_envio[j];
//                        salida_serial.publish(trama);
//                    }
//                }
//            }
//            for(int i=0; i<3;i++) MueveMotor[i+0*NmotorP] = false;
//        }
//
//        if (pata==Pata2){
//        //Pata 2
//            for(i=0; i<NmotorP;i++){
//                if(MueveMotor[i+1*NmotorP]) buffer << "#" << canal_Pata2+i << " P" << Posiciones[i+1*NmotorP] << " ";
//            }
//            if(MueveMotor[3] or MueveMotor[4] or MueveMotor[5]){
//                buffer << "\r";
//                trama_envio = buffer.str();
//                std::cout << "Pata2 #char: " << trama_envio.length() << "\n";
//                if (trama_envio.length() > 1){
//                    ROS_INFO("arana_connection: Envio trama pata2");
//                    std::cout << trama_envio << std::endl;
//                    for(j=0; j<trama_envio.length(); j++){
//                        trama.data_hexapodo = trama_envio[j];
//                        salida_serial.publish(trama);
//                    }
//                }
//            }
//            for(int i=0; i<3;i++) MueveMotor[i+1*NmotorP] = false;
//        }
//
//        if (pata==Pata3){
//        //Pata 3
//            for(i=0; i<NmotorP;i++){
//                if(MueveMotor[i+2*NmotorP] == true)	buffer << "#" << canal_Pata3+i << " P" << Posiciones[i+2*NmotorP] << " ";
//            }
//            if(MueveMotor[6] or MueveMotor[7] or MueveMotor[8]){
//                buffer << "\r";
//                trama_envio = buffer.str();
//                std::cout << "Pata2 #char: " << trama_envio.length() << "\n";
//                if (trama_envio.length() > 1){
//                    ROS_INFO("arana_connection: Envio trama pata3");
//                    std::cout << trama_envio << std::endl;
//                    for(j=0; j<trama_envio.length(); j++){
//                        trama.data_hexapodo = trama_envio[i];
//                        salida_serial.publish(trama);
//                    }
//                }
//            }
//            for(int i=0; i<3;i++) MueveMotor[i+2*NmotorP] = false;
//        }
//
//        if (pata==Pata4){
//        //Pata 4
//            for(i=0;i<NmotorP;i++){
//                if(MueveMotor[i+3*NmotorP]) buffer << "#" << canal_Pata4+i << " P" << Posiciones[i+3*NmotorP] << " ";
//            }
//            if(MueveMotor[9] or MueveMotor[10] or MueveMotor[11]){
//                buffer << "\r";
//                trama_envio = buffer.str();
//                std::cout << "Pata4 #char: " << trama_envio.length() << "\n";
//                if (trama_envio.length() > 1){
//                    ROS_INFO("arana_connection: Envio trama pata4");
//                    std::cout << trama_envio << std::endl;
//                    for(j=0; j<trama_envio.length(); j++){
//                        trama.data_hexapodo = trama_envio[j];
//                        salida_serial.publish(trama);
//                    }
//                }
//            }
//            for(int i=0; i<3;i++) MueveMotor[i+3*NmotorP] = false;
//        }
//
//        if (pata==Pata5){
//        //Pata 5
//            for(i=0;i<NmotorP;i++){
//                if(MueveMotor[i+4*NmotorP]) buffer << "#" << canal_Pata5+i << " P" << Posiciones[i+4*NmotorP] << " ";
//            }
//            if(MueveMotor[12] or MueveMotor[13] or MueveMotor[14]){
//                buffer << "\r";
//                trama_envio = buffer.str();
//                std::cout << "Pata5 #char: " << trama_envio.length() << "\n";
//                if (trama_envio.length() > 1){
//                    ROS_INFO("arana_connection: Envio trama pata5");
//                    std::cout << trama_envio << std::endl;
//                    for(j=0; j<trama_envio.length(); j++){
//                        trama.data_hexapodo = trama_envio[j];
//                        salida_serial.publish(trama);
//                    }
//                }
//            }
//            for(int i=0; i<3;i++) MueveMotor[i+4*NmotorP] = false;
//        }
//
//        if (pata==Pata6){
//        //Pata 6
//            for(i=0;i<NmotorP;i++){
//                if(MueveMotor[i+5*NmotorP]) buffer << "#" << canal_Pata6+i << " P" << Posiciones[i+5*NmotorP] << " ";
//            }
//            if(MueveMotor[15] or MueveMotor[16] or MueveMotor[17]){
//                buffer << "\r";
//                trama_envio = buffer.str();
//                std::cout << "Pata6 #char: " << trama_envio.length() << "\n";
//                if (trama_envio.length() > 1){
//                    ROS_INFO("arana_connection: Envio trama pata6");
//                    std::cout << trama_envio << std::endl;
//                    for(j=0; j<trama_envio.length(); j++){
//                        trama.data_hexapodo = trama_envio[j];
//                        salida_serial.publish(trama);
//                    }
//                }
//            }
//            for(int i=0; i<3;i++) MueveMotor[i+5*NmotorP] = false;
//        }
//}

