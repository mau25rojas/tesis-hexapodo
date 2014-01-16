#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
//Librerias propias usadas
#include "constantes.hpp"
#include "../util/comm/comm.hpp"
// Used data structures:
#include <hexacamina/AngulosParaRobot.h>
#include <hexacamina/commData.h>
// Definicion de las variables globales a utilizar y de los nodos de comunicación
#define LIM_SUP_Q1 2250
#define LIM_INF_Q1 700
#define LIM_SUP_Q2 1900
#define LIM_INF_Q2 600
#define LIM_SUP_Q3 2250
#define LIM_INF_Q3 1150
#define LIM_SUP_Q3_NEG 1950
#define LIM_INF_Q3_NEG 850
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
int i=0;
// Funciones
void EnviarTrama();

// Topic subscriber callbacks:
void recepcion_qCallback(hexacamina::AngulosParaRobot posicion)
{
    CambioNuevo = true;
        i = posicion.Npata;

        if (Posiciones[i*3+0] != (unsigned int) OFFSET_Q1 + posicion.Q1[i]*2000.0/180.0)
        {
            Posiciones[i*3+0] = (unsigned int) OFFSET_Q1 + posicion.Q1[i]*2000.0/180.0;
            if (Posiciones[i*3+0] < LIM_INF_Q1)	Posiciones[i*3+0] = LIM_INF_Q1;
            if (Posiciones[i*3+0] > LIM_SUP_Q1)	Posiciones[i*3+0] = LIM_SUP_Q1;
            MueveMotor[i*3+0] = true;
        }
        if (Posiciones[i*3+1] != (unsigned int) OFFSET_Q2 + posicion.Q2[i]*2000.0/180.0)
        {
            Posiciones[i*3+1] = (unsigned int) OFFSET_Q2 + posicion.Q2[i]*2000.0/180.0;
            if (Posiciones[i*3+1] < LIM_INF_Q2)	Posiciones[i*3+1] = LIM_INF_Q2;
            if (Posiciones[i*3+1] > LIM_SUP_Q2)	Posiciones[i*3+1] = LIM_SUP_Q2;
            MueveMotor[i*3+1] = true;
        }
        if (Posiciones[i*3+2] != (unsigned int) OFFSET_Q3 + posicion.Q3[i]*2000.0/180.0)
        {
            if (i == 0 or i == 2 or i == 4)
            {
                Posiciones[i*3+2] = (unsigned int) OFFSET_Q3 + posicion.Q3[i]*2000.0/180.0;
                if (Posiciones[i*3+2] < LIM_INF_Q3_NEG)	Posiciones[i*3+2] = LIM_INF_Q3;
                if (Posiciones[i*3+2] > LIM_SUP_Q3_NEG)	Posiciones[i*3+2] = LIM_SUP_Q3;
            }
            if (i == 1 or i == 3 or i == 5)
            {
                Posiciones[i*3+2] = (unsigned int) OFFSET_Q3 - posicion.Q3[i]*2000.0/180.0;
                if (Posiciones[i*3+2] < LIM_INF_Q3)	Posiciones[i*3+2] = LIM_INF_Q3_NEG;
                if (Posiciones[i*3+2] > LIM_SUP_Q3)	Posiciones[i*3+2] = LIM_SUP_Q3_NEG;
            }
            MueveMotor[i*3+2] = true;
        }
    EnviarTrama();
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "arana_connection");
	ROS_INFO("Inicia arana_connection");
	ros::NodeHandle n;
	//Subscripciones y publicaciones
	ros::Subscriber q_calculados = n.subscribe("arana/leg_position", 100, recepcion_qCallback);
	salida_serial = n.advertise<hexacamina::commData>("COMM_MSG_TX", 50);

	int i=0;
    CambioNuevo = true;
	for(i=0;i<Nmotores;i++) MueveMotor[i] = true;
	for(i=0;i<Npatas;i++){
		Posiciones[i*3] = OFFSET_Q1;
		Posiciones[i*3+1] = OFFSET_Q2;
		Posiciones[i*3+2] = OFFSET_Q3;
	}
	EnviarTrama();

	while (ros::ok() && simulationRunning)
  	{
        ros::spinOnce();
	}
    ROS_INFO("Adios arana_conection!");
	ros::shutdown();
	return(0);
}

/*	EnviarTrama: contruye la trama con las posiciones de los motores y si requieren moverse las envía	*/
void EnviarTrama(){

	std::stringstream buffer;
	std::string trama_envio;
	hexacamina::commData trama;
	int i=0;
	unsigned int j=0;

    if(CambioNuevo){
        for(i=0;i<Nmotores;i++){
//        for(i=Nmotores;i>0;i--){
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
//            std::cout << trama_envio << std::endl;
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
