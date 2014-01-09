#include "ros/ros.h"
//Librerias propias usadas
#include "constantes.hpp"
#include "camina3/v_repConst.h"
// Used data structures:
#include "camina3/EspacioTrabajoParametros.h"
// Used API services:
#include "vrep_common/VrepInfo.h"
// Global variables (modified by topic subscribers):
bool simulationRunning=true;
bool sensorTrigger=false;
float simulationTime=0.0f;
float PosicionPata_x=0.0, PosicionPata_y=0.0, PosicionPata_x2=0.0, PosicionPata_y2=0.0, anguloPatas_rad=0.0;
float origenPata_x[6], origenPata_y[6], rotacionPata[6];
float EspacioTrabajoPatas_yx[6][8], EspacioTrabajoPatas_ij[6][8];
/*
EspacioTrabajoPatas_yx[6][8]
    EspacioTrabajoPatas_yx(ij)[Pata1]: P1_y(i);P1_x(j);P2_y(i);P2_x(j);P3_y(i);P3_x(j);P4_y(i);P4_x(j)
    EspacioTrabajoPatas_yx(ij)[Pata2]: P1_y(i);P1_x(j);P2_y(i);P2_x(j);P3_y(i);P3_x(j);P4_y(i);P4_x(j)
    EspacioTrabajoPatas_yx(ij)[Pata3]: P1_y(i);P1_x(j);P2_y(i);P2_x(j);P3_y(i);P3_x(j);P4_y(i);P4_x(j)
    EspacioTrabajoPatas_yx(ij)[Pata4]: P1_y(i);P1_x(j);P2_y(i);P2_x(j);P3_y(i);P3_x(j);P4_y(i);P4_x(j)
    EspacioTrabajoPatas_yx(ij)[Pata5]: P1_y(i);P1_x(j);P2_y(i);P2_x(j);P3_y(i);P3_x(j);P4_y(i);P4_x(j)
    EspacioTrabajoPatas_yx(ij)[Pata6]: P1_y(i);P1_x(j);P2_y(i);P2_x(j);P3_y(i);P3_x(j);P4_y(i);P4_x(j)
*/

void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}

bool EspacioTrabajoPatas(camina3::EspacioTrabajoParametros::Request  &req,
                        camina3::EspacioTrabajoParametros::Response &res)
{
    for(int k=0;k<Npuntos;k++){
        res.EspacioTrabajoPata1_x.push_back(0);
        res.EspacioTrabajoPata2_x.push_back(0);
        res.EspacioTrabajoPata3_x.push_back(0);
        res.EspacioTrabajoPata4_x.push_back(0);
        res.EspacioTrabajoPata5_x.push_back(0);
        res.EspacioTrabajoPata6_x.push_back(0);
        res.EspacioTrabajoPata1_y.push_back(0);
        res.EspacioTrabajoPata2_y.push_back(0);
        res.EspacioTrabajoPata3_y.push_back(0);
        res.EspacioTrabajoPata4_y.push_back(0);
        res.EspacioTrabajoPata5_y.push_back(0);
        res.EspacioTrabajoPata6_y.push_back(0);
    }

    float P1_x=0.0,P1_y=0.0,P2_x=0.0,P2_y=0.0,P3_x=0.0,P3_y=0.0,P4_x=0.0,P4_y=0.0;
    float Py=0.0,Px=0.0;

    for(int k=0;k<Npatas;k++){
        //--Punto1
        Py = 0.0;
        Px = -EspacioTrabajo_X;
        P1_y = sin(rotacionPata[k])*Px+cos(rotacionPata[k])*Py+origenPata_y[k];
        P1_x = cos(rotacionPata[k])*Px-sin(rotacionPata[k])*Py+origenPata_x[k];
        //--Punto2
        Py = EspacioTrabajo_Y;
        Px = -EspacioTrabajo_X;
        P2_y = sin(rotacionPata[k])*Px+cos(rotacionPata[k])*Py+origenPata_y[k];
        P2_x = cos(rotacionPata[k])*Px-sin(rotacionPata[k])*Py+origenPata_x[k];
        //--Punto3
        Py = 0.0;
        Px = EspacioTrabajo_X;
        P3_y = sin(rotacionPata[k])*Px+cos(rotacionPata[k])*Py+origenPata_y[k];
        P3_x = cos(rotacionPata[k])*Px-sin(rotacionPata[k])*Py+origenPata_x[k];
        //--Punto4
        Py = EspacioTrabajo_Y;
        Px = EspacioTrabajo_X;
        P4_y = sin(rotacionPata[k])*Px+cos(rotacionPata[k])*Py+origenPata_y[k];
        P4_x = cos(rotacionPata[k])*Px-sin(rotacionPata[k])*Py+origenPata_x[k];

        switch(k+1){
            case Pata1:
                //-- Transformacion de coordenadas segun posicion y rotacion del robot
                res.EspacioTrabajoPata1_y[0] = sin(req.theta_CuerpoRobot)*P1_x+cos(req.theta_CuerpoRobot)*P1_y+req.PosicionCuerpo_y;
                res.EspacioTrabajoPata1_x[0] = cos(req.theta_CuerpoRobot)*P1_x-sin(req.theta_CuerpoRobot)*P1_y+req.PosicionCuerpo_x;
                res.EspacioTrabajoPata1_y[1] = sin(req.theta_CuerpoRobot)*P2_x+cos(req.theta_CuerpoRobot)*P2_y+req.PosicionCuerpo_y;
                res.EspacioTrabajoPata1_x[1] = cos(req.theta_CuerpoRobot)*P2_x-sin(req.theta_CuerpoRobot)*P2_y+req.PosicionCuerpo_x;
                res.EspacioTrabajoPata1_y[2] = sin(req.theta_CuerpoRobot)*P3_x+cos(req.theta_CuerpoRobot)*P3_y+req.PosicionCuerpo_y;
                res.EspacioTrabajoPata1_x[2] = cos(req.theta_CuerpoRobot)*P3_x-sin(req.theta_CuerpoRobot)*P3_y+req.PosicionCuerpo_x;
                res.EspacioTrabajoPata1_y[3] = sin(req.theta_CuerpoRobot)*P4_x+cos(req.theta_CuerpoRobot)*P4_y+req.PosicionCuerpo_y;
                res.EspacioTrabajoPata1_x[3] = cos(req.theta_CuerpoRobot)*P4_x-sin(req.theta_CuerpoRobot)*P4_y+req.PosicionCuerpo_x;

            break;
            case Pata2:
                //-- Transformacion de coordenadas segun posicion y rotacion del robot
                res.EspacioTrabajoPata2_y[0] = sin(req.theta_CuerpoRobot)*P1_x+cos(req.theta_CuerpoRobot)*P1_y+req.PosicionCuerpo_y;
                res.EspacioTrabajoPata2_x[0] = cos(req.theta_CuerpoRobot)*P1_x-sin(req.theta_CuerpoRobot)*P1_y+req.PosicionCuerpo_x;
                res.EspacioTrabajoPata2_y[1] = sin(req.theta_CuerpoRobot)*P2_x+cos(req.theta_CuerpoRobot)*P2_y+req.PosicionCuerpo_y;
                res.EspacioTrabajoPata2_x[1] = cos(req.theta_CuerpoRobot)*P2_x-sin(req.theta_CuerpoRobot)*P2_y+req.PosicionCuerpo_x;
                res.EspacioTrabajoPata2_y[2] = sin(req.theta_CuerpoRobot)*P3_x+cos(req.theta_CuerpoRobot)*P3_y+req.PosicionCuerpo_y;
                res.EspacioTrabajoPata2_x[2] = cos(req.theta_CuerpoRobot)*P3_x-sin(req.theta_CuerpoRobot)*P3_y+req.PosicionCuerpo_x;
                res.EspacioTrabajoPata2_y[3] = sin(req.theta_CuerpoRobot)*P4_x+cos(req.theta_CuerpoRobot)*P4_y+req.PosicionCuerpo_y;
                res.EspacioTrabajoPata2_x[3] = cos(req.theta_CuerpoRobot)*P4_x-sin(req.theta_CuerpoRobot)*P4_y+req.PosicionCuerpo_x;
            break;
            case Pata3:
                //-- Transformacion de coordenadas segun posicion y rotacion del robot
                res.EspacioTrabajoPata3_y[0] = sin(req.theta_CuerpoRobot)*P1_x+cos(req.theta_CuerpoRobot)*P1_y+req.PosicionCuerpo_y;
                res.EspacioTrabajoPata3_x[0] = cos(req.theta_CuerpoRobot)*P1_x-sin(req.theta_CuerpoRobot)*P1_y+req.PosicionCuerpo_x;
                res.EspacioTrabajoPata3_y[1] = sin(req.theta_CuerpoRobot)*P2_x+cos(req.theta_CuerpoRobot)*P2_y+req.PosicionCuerpo_y;
                res.EspacioTrabajoPata3_x[1] = cos(req.theta_CuerpoRobot)*P2_x-sin(req.theta_CuerpoRobot)*P2_y+req.PosicionCuerpo_x;
                res.EspacioTrabajoPata3_y[2] = sin(req.theta_CuerpoRobot)*P3_x+cos(req.theta_CuerpoRobot)*P3_y+req.PosicionCuerpo_y;
                res.EspacioTrabajoPata3_x[2] = cos(req.theta_CuerpoRobot)*P3_x-sin(req.theta_CuerpoRobot)*P3_y+req.PosicionCuerpo_x;
                res.EspacioTrabajoPata3_y[3] = sin(req.theta_CuerpoRobot)*P4_x+cos(req.theta_CuerpoRobot)*P4_y+req.PosicionCuerpo_y;
                res.EspacioTrabajoPata3_x[3] = cos(req.theta_CuerpoRobot)*P4_x-sin(req.theta_CuerpoRobot)*P4_y+req.PosicionCuerpo_x;
            break;
            case Pata4:
                //-- Transformacion de coordenadas segun posicion y rotacion del robot
                res.EspacioTrabajoPata4_y[0] = sin(req.theta_CuerpoRobot)*P1_x+cos(req.theta_CuerpoRobot)*P1_y+req.PosicionCuerpo_y;
                res.EspacioTrabajoPata4_x[0] = cos(req.theta_CuerpoRobot)*P1_x-sin(req.theta_CuerpoRobot)*P1_y+req.PosicionCuerpo_x;
                res.EspacioTrabajoPata4_y[1] = sin(req.theta_CuerpoRobot)*P2_x+cos(req.theta_CuerpoRobot)*P2_y+req.PosicionCuerpo_y;
                res.EspacioTrabajoPata4_x[1] = cos(req.theta_CuerpoRobot)*P2_x-sin(req.theta_CuerpoRobot)*P2_y+req.PosicionCuerpo_x;
                res.EspacioTrabajoPata4_y[2] = sin(req.theta_CuerpoRobot)*P3_x+cos(req.theta_CuerpoRobot)*P3_y+req.PosicionCuerpo_y;
                res.EspacioTrabajoPata4_x[2] = cos(req.theta_CuerpoRobot)*P3_x-sin(req.theta_CuerpoRobot)*P3_y+req.PosicionCuerpo_x;
                res.EspacioTrabajoPata4_y[3] = sin(req.theta_CuerpoRobot)*P4_x+cos(req.theta_CuerpoRobot)*P4_y+req.PosicionCuerpo_y;
                res.EspacioTrabajoPata4_x[3] = cos(req.theta_CuerpoRobot)*P4_x-sin(req.theta_CuerpoRobot)*P4_y+req.PosicionCuerpo_x;
            break;
            case Pata5:
                //-- Transformacion de coordenadas segun posicion y rotacion del robot
                res.EspacioTrabajoPata5_y[0] = sin(req.theta_CuerpoRobot)*P1_x+cos(req.theta_CuerpoRobot)*P1_y+req.PosicionCuerpo_y;
                res.EspacioTrabajoPata5_x[0] = cos(req.theta_CuerpoRobot)*P1_x-sin(req.theta_CuerpoRobot)*P1_y+req.PosicionCuerpo_x;
                res.EspacioTrabajoPata5_y[1] = sin(req.theta_CuerpoRobot)*P2_x+cos(req.theta_CuerpoRobot)*P2_y+req.PosicionCuerpo_y;
                res.EspacioTrabajoPata5_x[1] = cos(req.theta_CuerpoRobot)*P2_x-sin(req.theta_CuerpoRobot)*P2_y+req.PosicionCuerpo_x;
                res.EspacioTrabajoPata5_y[2] = sin(req.theta_CuerpoRobot)*P3_x+cos(req.theta_CuerpoRobot)*P3_y+req.PosicionCuerpo_y;
                res.EspacioTrabajoPata5_x[2] = cos(req.theta_CuerpoRobot)*P3_x-sin(req.theta_CuerpoRobot)*P3_y+req.PosicionCuerpo_x;
                res.EspacioTrabajoPata5_y[3] = sin(req.theta_CuerpoRobot)*P4_x+cos(req.theta_CuerpoRobot)*P4_y+req.PosicionCuerpo_y;
                res.EspacioTrabajoPata5_x[3] = cos(req.theta_CuerpoRobot)*P4_x-sin(req.theta_CuerpoRobot)*P4_y+req.PosicionCuerpo_x;
            break;
            case Pata6:
                //-- Transformacion de coordenadas segun posicion y rotacion del robot
                res.EspacioTrabajoPata6_y[0] = sin(req.theta_CuerpoRobot)*P1_x+cos(req.theta_CuerpoRobot)*P1_y+req.PosicionCuerpo_y;
                res.EspacioTrabajoPata6_x[0] = cos(req.theta_CuerpoRobot)*P1_x-sin(req.theta_CuerpoRobot)*P1_y+req.PosicionCuerpo_x;
                res.EspacioTrabajoPata6_y[1] = sin(req.theta_CuerpoRobot)*P2_x+cos(req.theta_CuerpoRobot)*P2_y+req.PosicionCuerpo_y;
                res.EspacioTrabajoPata6_x[1] = cos(req.theta_CuerpoRobot)*P2_x-sin(req.theta_CuerpoRobot)*P2_y+req.PosicionCuerpo_x;
                res.EspacioTrabajoPata6_y[2] = sin(req.theta_CuerpoRobot)*P3_x+cos(req.theta_CuerpoRobot)*P3_y+req.PosicionCuerpo_y;
                res.EspacioTrabajoPata6_x[2] = cos(req.theta_CuerpoRobot)*P3_x-sin(req.theta_CuerpoRobot)*P3_y+req.PosicionCuerpo_x;
                res.EspacioTrabajoPata6_y[3] = sin(req.theta_CuerpoRobot)*P4_x+cos(req.theta_CuerpoRobot)*P4_y+req.PosicionCuerpo_y;
                res.EspacioTrabajoPata6_x[3] = cos(req.theta_CuerpoRobot)*P4_x-sin(req.theta_CuerpoRobot)*P4_y+req.PosicionCuerpo_x;
            break;
        }
    }
  return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "EspacioTrabajo_server");
    ros::NodeHandle n;

    ros::Subscriber subInfo=n.subscribe("/vrep/info",1,infoCallback);
    ros::ServiceServer service = n.advertiseService("EspacioTrabajo", EspacioTrabajoPatas);

//    anguloPatas_rad = anguloPatas*pi/180;
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
