#include "ros/ros.h"
#include "camina/TransHomogeneaParametros.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TransHomogenea_client");
  if (argc != 8)
  {
    ROS_INFO("error: client_Transhomogenea");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<camina::TransHomogeneaParametros>("TransHomogenea");
  camina::TransHomogeneaParametros srv;
  srv.request.x_Trasl   = atof(argv[1]);
  srv.request.y_Trasl   = atof(argv[2]);
  srv.request.z_Trasl   = atof(argv[3]);
  srv.request.x_S0      = atof(argv[4]);
  srv.request.y_S0      = atof(argv[5]);
  srv.request.z_S0      = atof(argv[6]);
  srv.request.tita_Rot  = atof(argv[7]);
  if (client.call(srv))
  {
    ROS_INFO("x_S1:%.3f, y_S1:%.3f, z_S1:%.3f", srv.response.x_S1,srv.response.y_S1, srv.response.z_S1);
  }
  else
  {
    ROS_ERROR("Failed to call service TransHomogenea_server");
    return 1;
  }

  return 0;
}
