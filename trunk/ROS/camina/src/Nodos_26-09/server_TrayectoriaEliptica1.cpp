#include "ros/ros.h"
//Librerias propias usadas
#include "constantes.hpp"
#include "camina/v_repConst.h"
// Used data structures:
#include "camina/TrayectoriaEliptica1Parametros.h"
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

bool TrayectoriaEliptica1(camina::TrayectoriaEliptica1Parametros::Request  &req,
                        camina::TrayectoriaEliptica1Parametros::Response &res)
{
    float velocidadApoyo=0.0;//, velocidadTransferencia=0.0;
    float alfa_arg=0.0, t_aux=0.0;
    float x_S0=0.0, y_S0=0.0, z_S0=0.0;

    velocidadApoyo = req.lambda_Apoyo/req.beta;
     /* Nota Mental:
        Existen 2 partes, la parte de apoyo que es una linea
        recta, y la eliptica que es el arco descrito en el
        aire
     */
    //---Caso parte 1 trayectoria----
    // Periodo A-B
    if (0<=req.t_Trayectoria && req.t_Trayectoria<req.beta)
        {
            x_S0 = -req.lambda_Apoyo/2 + velocidadApoyo*req.t_Trayectoria;
            y_S0 = 0.0;
            z_S0 = 0.0;
        }
    //---Caso parte 2 trayectoria------
    // Elipsis
        else {
            t_aux = (req.t_Trayectoria-req.beta)/(1-req.beta);
            alfa_arg = pi*t_aux;
            x_S0 = (req.lambda_Apoyo/2)*cos(alfa_arg);
//            x_S0 = req.lambda_Transferencia/2*cos(alfa_arg);
            y_S0 = 0.0;
            z_S0 = req.dh*sin(alfa_arg);
        }
//        ROS_INFO("beta = %.3f",req.beta);
//        ROS_INFO("t=%.3f: x_S0=%.3f\ty_S0=%.3f\tz_S0=%.3f",req.t_Trayectoria,x_S0,y_S0,z_S0);
//            //---Se añade rotacion y traslacion a coordenadas encontradas---
//            //---..se realiza traslacion a eje de pata
//            //x en sistema 1
//            res.x_S1 = req.x_Offset+x_S0*cos(req.alfa+req.phi)-y_S0*sin(req.alfa+req.phi);
//            //y en sistema 1
//            res.y_S1 = req.y_Offset+x_S0*sin(req.alfa+req.phi)+y_S0*cos(req.alfa+req.phi);
//            //z en sistema 1
//            res.z_S1 = req.z_Offset+z_S0;
//            ROS_INFO("t=%.3f: x_S1=%.3f\ty_S1=%.3f\tz_S1=%.3f",req.t_Trayectoria,res.x_S1,res.y_S1,res.z_S1);

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TrayectoriaEliptica1_server");
  ros::NodeHandle n;

  ros::Subscriber subInfo=n.subscribe("/vrep/info",1,infoCallback);
  ros::ServiceServer service = n.advertiseService("TrayectoriaEliptica1", TrayectoriaEliptica1);

  while (ros::ok() && simulationRunning) {
    ros::spinOnce();
  }

  return 0;
}
