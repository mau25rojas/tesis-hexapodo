#include "ros/ros.h"
#include "camina/AnguloParaVrep.h"
#include "sensor_msgs/Joy.h"

#include <sstream>

ros::Publisher chatter_pub;
camina::AnguloParaVrep msg2;
bool todos = false;

void chatterCallback(const sensor_msgs::Joy msg)
{
	//valor de los 3 angulos de las articulaciones (Q1 Q2 Q3)
  msg2.Q1 = msg.axes[2]*70.0;	//69.3-202.5	Centro:139.5
  msg2.Q2 = msg.axes[1]*46;	//93.6-175.5	Centro:139.5
  msg2.Q3 = msg.axes[3]*63;	//103.5-202.5	Centro:139.5

  if (msg.buttons[6] == true)
  {
  	msg2.Npata = 0; //selecciona la pata 0
  }
  if (msg.buttons[7] == true)
  {
  	msg2.Npata = 1; //selecciona la pata 1
  }
  if (msg.buttons[8] == true)
  {
  	msg2.Npata = 2; //selecciona la pata 2
  }
  if (msg.buttons[9] == true)
  {
  	msg2.Npata = 3; //selecciona la pata 3
  }
  if (msg.buttons[10] == true)
  {
  	msg2.Npata = 4; //selecciona la pata 4
  }
  if (msg.buttons[11] == true)
  {
  	msg2.Npata = 5; //selecciona la pata 5
  }
  if (msg.buttons[4] == true)
  {
  	todos = true;
  }
  if (msg.buttons[5] == true)
  {
  	todos = false;
  }
	//-----------------------------------
  if (todos == false)
  {
	  chatter_pub.publish(msg2);
  }
  else
  {
	msg2.Npata = 0;
	chatter_pub.publish(msg2);
	msg2.Npata = 1;
	chatter_pub.publish(msg2);
	msg2.Npata = 2;
	chatter_pub.publish(msg2);
	msg2.Npata = 3;
	chatter_pub.publish(msg2);
	msg2.Npata = 4;
	chatter_pub.publish(msg2);
	msg2.Npata = 5;
	chatter_pub.publish(msg2);
	//envio una copia del mensaje a cada una de las patas (todas las patas iguales)
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "force3d_to_hexapodo");
  ros::NodeHandle n;
  chatter_pub = n.advertise<camina::AnguloParaVrep>("arana/leg_position", 100);
  ros::Subscriber sub = n.subscribe("joy", 10, chatterCallback);
  msg2.Npata = 0;
  ros::spin();
  return 0;
}
