#include "ros/ros.h"
//Librerias propias usadas
#include "constantes.hpp"
#include "camina4/v_repConst.h"
// Used data structures:
#include "camina4/CinversaParametros.h"
// Used API services:
#include "vrep_common/VrepInfo.h"
// Global variables (modified by topic subscribers):
bool simulationRunning=true;
bool sensorTrigger=false;
float simulationTime=0.0f;

void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}

/* Servicio que devuelve posicion de motores (radianes) para movimiento de pata
  ..Todos los cálculos se realizan en el sistema del robot y se transforma a cada una de las patas:
  ..Se reciben coordenadas de trayectoria en Sistema0
  ..Se transforman a Sistema1 (sistema de cada pata)
  ..Se calcula la cinematica inversa a estas coordenadas transformadas
*/

bool CinematicaInversa(camina4::CinversaParametros::Request  &req,
                        camina4::CinversaParametros::Response &res)
{   float x=0.0, y=0.0, z=0.0;
    float L23=0.0, L23_aux=0.0, beta=0.0, teta=0.0, gamma1=0.0, arg=0;
    res.result = 0;

    x = req.x;
    y = req.y;
    z = req.z;

    //---Cinematica inversa---
    //------calculo q1------
    //p[0] = atan2(px,py);
    res.q1 = atan2(y,x)-pi/2;
    //----------------------

    L23_aux = sqrt(x*x + y*y) - L1;
    L23 = sqrt(L23_aux*L23_aux + z*z);

    //------calculo q3------
	arg = (L2*L2 + L3*L3 - L23*L23)/(2*L2*L3);
	if (fabs(arg)>1) {
//        ROS_ERROR("ERROR Cinversa_server acos q3");
        res.result = -1;
        return -1;
	}
	beta = acos(arg);
    //El ajuste de pi se hace para coincidir con eje de D-H
    //p[2] = pi - beta;
    res.q3 = beta-pi/2;
    //----------------------

    //------calculo q2------
	arg= (L3/L23)*sin(beta);
	if (fabs(arg)>1) {
//        ROS_ERROR("ERROR Cinversa_server asin q2");
        res.result = -1;
        return -1;
	}
	teta = atan(-z/L23_aux);
	gamma1 = asin(arg);
	//El ajuste de pi/2 se hace para coincidir con eje de D-H
    //p[1] = pi/2 - (teta-gamma);
    res.q2 = teta-gamma1;

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Cinversa_server");
  ros::NodeHandle n;

  ros::Subscriber subInfo=n.subscribe("/vrep/info",1,infoCallback);
  ros::ServiceServer service = n.advertiseService("Cinversa", CinematicaInversa);

  while (ros::ok() && simulationRunning) {
    ros::spinOnce();
  }

  return 0;
}
