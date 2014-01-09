#include "ros/ros.h"
#include "camina/CinversaParametros.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Cinversa_client");
  if (argc != 4)
  {
    ROS_INFO("usage: Cinversa_client X Y Z");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<camina::CinversaParametros>("Cinversa");
  camina::CinversaParametros srv;
  srv.request.x = atof(argv[1]);
  srv.request.y = atof(argv[2]);
  srv.request.z = atof(argv[3]);
  if (client.call(srv))
  {
    ROS_INFO("q1:%.3f, q2:%.3f, q3:%.3f", srv.response.q1,srv.response.q2, srv.response.q3);
  }
  else
  {
    ROS_ERROR("Failed to call service Cinversa_server");
    return 1;
  }

  return 0;
}
