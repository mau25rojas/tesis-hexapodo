#include "ros/ros.h"
//Librerias propias usadas
#include "constantes.hpp"
#include "camina/v_repConst.h"
// Used data structures:
#include "camina/TransHomogeneaParametros.h"
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

/* Servicio que devuelve transformacion homogenea de un punto
    segun una rotacion en X-Y y traslacion en XYZ
*/

bool TransformacionHomogenea(camina::TransHomogeneaParametros::Request  &req,
                        camina::TransHomogeneaParametros::Response &res)
{
    res.x_S1 = req.x_Trasl + req.x_S0*cos(req.theta_Rot) - req.y_S0*sin(req.theta_Rot);
    res.y_S1 = req.y_Trasl + req.x_S0*sin(req.theta_Rot) + req.y_S0*cos(req.theta_Rot);
    res.z_S1 = req.z_Trasl + req.z_S0;

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TransHomogenea_server");
  ros::NodeHandle n;

  ros::Subscriber subInfo=n.subscribe("/vrep/info",1,infoCallback);
  ros::ServiceServer service = n.advertiseService("TransHomogenea", TransformacionHomogenea);

  while (ros::ok() && simulationRunning) {
    ros::spinOnce();
  }

  return 0;
}
