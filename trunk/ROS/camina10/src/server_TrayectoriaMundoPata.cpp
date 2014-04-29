#include "ros/ros.h"
//Librerias propias usadas
#include "constantes.hpp"
#include "camina10/v_repConst.h"
// Used data structures:
#include "camina10/TransTrayectoriaParametros.h"
// Used API services:
#include "vrep_common/VrepInfo.h"
// Global variables (modified by topic subscribers):
bool simulationRunning=true;
bool sensorTrigger=false;
float simulationTime=0.0f;
float PosicionPata_x=0.0, PosicionPata_y=0.0, PosicionPata_x2=0.0, PosicionPata_y2=0.0;
float origenPata_x[6], origenPata_y[6], rotacionPata[6];

void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}

/* Servicio que devuelve transformacion homogenea de un punto
    en el MUNDO hacia una PATA determinada del robot
*/

bool TrayectoriaMundoPata(camina10::TransTrayectoriaParametros::Request  &req,
                        camina10::TransTrayectoriaParametros::Response &res)
{
    float x_Robot=0.0,y_Robot=0.0;

    switch (req.modo){
        case Pata_Mundo:
        //--Primera transformacion: Sistema ROBOT en el Sistema MUNDO
            x_Robot = origenPata_x[req.Npata] + req.x_S0*cos(rotacionPata[req.Npata]) - req.y_S0*sin(rotacionPata[req.Npata]);
            y_Robot = origenPata_y[req.Npata] + req.x_S0*sin(rotacionPata[req.Npata]) + req.y_S0*cos(rotacionPata[req.Npata]);
        //--Segunda transformacion: del Sistema PATA en el Sistema MUNDO
            res.x_Mundo = req.x_UbicacionRob + x_Robot*cos(req.theta_Rob) - y_Robot*sin(req.theta_Rob);
            res.y_Mundo = req.y_UbicacionRob + x_Robot*sin(req.theta_Rob) + y_Robot*cos(req.theta_Rob);
            res.z_Mundo = req.z_S0;
        break;

        case Mundo_Pata:
        //--Devolvemos segunda transformacion
            x_Robot = -(req.x_UbicacionRob*cos(req.theta_Rob)+req.y_UbicacionRob*sin(req.theta_Rob)) + req.x_S0*cos(req.theta_Rob) + req.y_S0*sin(req.theta_Rob);
            y_Robot = -(-req.x_UbicacionRob*sin(req.theta_Rob)+req.y_UbicacionRob*cos(req.theta_Rob)) - req.x_S0*sin(req.theta_Rob) + req.y_S0*cos(req.theta_Rob);
        //--Devolvemos primera transformacion: del Sistema ROBOT al Sistema MUNDO
            res.x_Pata = -(origenPata_x[req.Npata]*cos(rotacionPata[req.Npata])+origenPata_y[req.Npata]*sin(rotacionPata[req.Npata])) + x_Robot*cos(rotacionPata[req.Npata]) + y_Robot*sin(rotacionPata[req.Npata]);
            res.y_Pata = -(-origenPata_x[req.Npata]*sin(rotacionPata[req.Npata])+origenPata_y[req.Npata]*cos(rotacionPata[req.Npata])) - x_Robot*sin(rotacionPata[req.Npata]) + y_Robot*cos(rotacionPata[req.Npata]);
            res.z_Pata = req.z_S0;
        break;
    }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TrayectoriaMundoPata_server");
  ros::NodeHandle n;

  ros::Subscriber subInfo=n.subscribe("/vrep/info",1,infoCallback);
  ros::ServiceServer service = n.advertiseService("TrayectoriaMundoPata", TrayectoriaMundoPata);

    PosicionPata_x = radioCuerpo*cos(anguloPatas);
    PosicionPata_y = radioCuerpo*sin(anguloPatas);
    PosicionPata_x2 = radioCuerpo;
    PosicionPata_y2 = 0.0;
    //-- Pata1
    origenPata_y[0]=PosicionPata_y;
    origenPata_x[0]=-PosicionPata_x;
    rotacionPata[0]=rotacion_Pata1;
    //-- Pata2
    origenPata_y[1]=PosicionPata_y;
    origenPata_x[1]=PosicionPata_x;
    rotacionPata[1]=rotacion_Pata2;
    //-- Pata3
    origenPata_y[2]=PosicionPata_y2;
    origenPata_x[2]=-PosicionPata_x2;
    rotacionPata[2]=rotacion_Pata3;
    //-- Pata4
    origenPata_y[3]=PosicionPata_y2;
    origenPata_x[3]=PosicionPata_x2;
    rotacionPata[3]=rotacion_Pata4;
    //-- Pata5
    origenPata_y[4]=-PosicionPata_y;
    origenPata_x[4]=-PosicionPata_x;
    rotacionPata[4]=rotacion_Pata5;
    //-- Pata6
    origenPata_y[5]=-PosicionPata_y;
    origenPata_x[5]=PosicionPata_x;
    rotacionPata[5]=rotacion_Pata6;

  while (ros::ok() && simulationRunning) {
    ros::spinOnce();
  }

  return 0;
}
