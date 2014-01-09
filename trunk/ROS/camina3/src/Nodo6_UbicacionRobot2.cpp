#include <stdio.h>
#include <string.h>
#include <iostream>
#include <stdlib.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <boost/circular_buffer.hpp>
#include <boost/circular_buffer_fwd.hpp>
//Librerias propias usadas
#include "constantes.hpp"
#include "camina3/v_repConst.h"
// Used data structures:
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>
#include "camina3/UbicacionRobot.h"
#include "camina3/TransTrayectoriaParametros.h"
// Used API services:
#include "vrep_common/VrepInfo.h"
#include "vrep_common/ObjectGroupData.h"
//-- Clientes y servicios
ros::ServiceClient client_Trans_MundoPata;
camina3::TransTrayectoriaParametros srv_Trans_MundoPata;
//-- Global variables:
bool simulationRunning=true;
bool sensorTrigger=false;
bool infoCuerpo=false, infoPatas=false, infoVel=false;
float simulationTime=0.0f;
float Veloy_twist=0.0;
double tiempo_ahora2=0.0, tiempo_anterior2=0.0;
ros::Time time_stamp;
FILE *fp;
camina3::UbicacionRobot ubicacionRobot;
tf::Quaternion CuerpoOrientacion_Q;
tfScalar roll, pitch, yaw;

// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}

void ubicacionCallback(const vrep_common::ObjectGroupData msgUbicacionPatas)
{
	for(int k=1; k<Npatas+1; k++){
        ubicacionRobot.coordenadaPata_x[k-1]=msgUbicacionPatas.floatData.data[0+k*Npatas/2];
        ubicacionRobot.coordenadaPata_y[k-1]=msgUbicacionPatas.floatData.data[1+k*Npatas/2];
        ubicacionRobot.coordenadaPata_z[k-1]=msgUbicacionPatas.floatData.data[2+k*Npatas/2];

    //-- Transformacion de trayectoria a Sistema de Pata
        srv_Trans_MundoPata.request.modo=Mundo_Pata;
        srv_Trans_MundoPata.request.Npata=k-1;
        srv_Trans_MundoPata.request.x_S0=ubicacionRobot.coordenadaPata_x[k-1];
        srv_Trans_MundoPata.request.y_S0=ubicacionRobot.coordenadaPata_y[k-1];
        srv_Trans_MundoPata.request.z_S0=ubicacionRobot.coordenadaPata_z[k-1];
        srv_Trans_MundoPata.request.x_UbicacionRob=ubicacionRobot.coordenadaCuerpo_x;
        srv_Trans_MundoPata.request.y_UbicacionRob=ubicacionRobot.coordenadaCuerpo_y;
        srv_Trans_MundoPata.request.theta_Rob=ubicacionRobot.orientacionCuerpo_yaw;
        if (client_Trans_MundoPata.call(srv_Trans_MundoPata))
        {   ubicacionRobot.coordenadaPataSistemaPata_x[k-1] = srv_Trans_MundoPata.response.x_Pata;
            ubicacionRobot.coordenadaPataSistemaPata_y[k-1] = srv_Trans_MundoPata.response.y_Pata;
            ubicacionRobot.coordenadaPataSistemaPata_z[k-1] = srv_Trans_MundoPata.response.z_Pata;
    //                ROS_INFO("Servicio motores: q1=%.3f; q2=%.3f; q3=%.3f\n", _q1, _q2, _q3);
        } else {
            ROS_ERROR("Nodo6:[%d] servicio de Trans_MundoPata no funciona\n",k-1);
    //        return;
        }

//        if (ubicacionRobot.coordenadaPata_z[k-1]<=umbral_Z_Apoyo) {
//    //    if (ubicacionRobot.pataTipFuerza_z[k]>=umbralFuerzaApoyo) {
//        //-- La pata esta en apoyo
//            ubicacionRobot.pataApoyo[k-1]=1;
//        } else {
//        //-- La pata esta en transferencia
//            ubicacionRobot.pataApoyo[k-1]=0;
//        }
//    ROS_INFO("Nodo6: pata[%d], x=%.3f, y=%.3f",k-1,ubicacionRobot.coordenadaPata_x[k-1],ubicacionRobot.coordenadaPata_y[k-1]);
    }
    infoPatas = true;
}

void ubicacionCuerpoCallback(geometry_msgs::PoseStamped msgUbicacionCuerpo)
{
    tiempo_anterior2 = tiempo_ahora2;
    time_stamp = msgUbicacionCuerpo.header.stamp;
    tiempo_ahora2 = time_stamp.toSec();
    ubicacionRobot.coordenadaCuerpo_x = msgUbicacionCuerpo.pose.position.x;
    ubicacionRobot.coordenadaCuerpo_y = msgUbicacionCuerpo.pose.position.y;
    tf::quaternionMsgToTF(msgUbicacionCuerpo.pose.orientation,CuerpoOrientacion_Q);
    tf::Matrix3x3(CuerpoOrientacion_Q).getRPY(roll, pitch, yaw);
    ubicacionRobot.orientacionCuerpo_roll = roll;
    ubicacionRobot.orientacionCuerpo_pitch = pitch;
    ubicacionRobot.orientacionCuerpo_yaw = yaw;

    infoCuerpo = true;
}
void velocidadCuerpoCallback(geometry_msgs::TwistStamped msgVelocidadCuerpo)
{
//    ubicacionRobot.velocidadCuerpo_x = msgVelocidadCuerpo.Twist.linear.x;
//    ubicacionRobot.velocidadCuerpo_y = msgVelocidadCuerpo.twist.linear.y;
    Veloy_twist = msgVelocidadCuerpo.twist.linear.y;

//    ROS_INFO("v_y=%.3f",ubicacionRobot.velocidadCuerpo_y);

    infoVel = true;
}

void fuerzaCallback(const vrep_common::ObjectGroupData msgFuerzaPatas)
{
	for(int k=1; k<Npatas+1; k++){
        ubicacionRobot.pataTipFuerza_z[k]=msgFuerzaPatas.floatData.data[3+k*Npatas];
	}
//        if (ubicacionRobot.coordenadaPata_z[k-1]<=umbral_Z_Apoyo) {
//    //    if (ubicacionRobot.pataTipFuerza_z[k]>=umbralFuerzaApoyo) {
//        //-- La pata esta en apoyo
//            ubicacionRobot.pataApoyo[k-1]=1;
//        } else {
//        //-- La pata esta en transferencia
//            ubicacionRobot.pataApoyo[k-1]=0;
//        }
}


// Main code:
int main(int argc,char* argv[])
{
	float Periodo=0.0, f=0.0;
	int Narg=0;
	std::string dummys,cuerpo, vel, fuerza;
	int window_size=10;
    boost::circular_buffer<float> cb(window_size);

	Narg=4;

	if (argc>=Narg)
	{
	    dummys = argv[1];
	    cuerpo = argv[2];
	    vel = argv[3];
	    fuerza = argv[4];
//	    printf("%s\n",dummys_topicName.c_str());
	} else {
		ROS_ERROR("Nodo6:Indique argumentos completos!\n");
		return (0);
	}

	std::string topicDummy("/vrep/");
	std::string topicCuerpo("/vrep/");
	std::string topicVelCuerpo("/vrep/");
	std::string topicFuerza("/vrep/");
	topicDummy+=dummys;
	topicCuerpo+=cuerpo;
	topicVelCuerpo+=vel;
	topicFuerza+=fuerza;
//-- Inicializacion de variables del mensaje
    for (int k=0;k<Npatas;k++) {
        ubicacionRobot.coordenadaPata_x.push_back(0);
        ubicacionRobot.coordenadaPata_y.push_back(0);
        ubicacionRobot.coordenadaPata_z.push_back(0);
        ubicacionRobot.coordenadaPataSistemaPata_x.push_back(0);
        ubicacionRobot.coordenadaPataSistemaPata_y.push_back(0);
        ubicacionRobot.coordenadaPataSistemaPata_z.push_back(0);
        ubicacionRobot.pataTipFuerza_z.push_back(0);
        ubicacionRobot.pataApoyo.push_back(0);
    }
	// Create a ROS node:
	int _argc = 0;
	char** _argv = NULL;
	ros::init(_argc,_argv,"Nodo6_UbicacionArana");
	if(!ros::master::check()) return(0);

	ros::NodeHandle node;
	ROS_INFO("Nodo6_UbicacionArana just started\n");

//-- Topicos susbcritos y publicados
    ros::Subscriber subInfo=node.subscribe("/vrep/info",1,infoCallback);
    ros::Subscriber subInfo1=node.subscribe(topicDummy,100,ubicacionCallback);
    ros::Subscriber subInfo2=node.subscribe(topicCuerpo,100,ubicacionCuerpoCallback);
    ros::Subscriber subInfo3=node.subscribe(topicVelCuerpo,100,velocidadCuerpoCallback);
    ros::Subscriber subInfo4=node.subscribe(topicFuerza,100,fuerzaCallback);
    ros::Publisher chatter_pub = node.advertise<camina3::UbicacionRobot>("UbicacionRobot", 100);
    client_Trans_MundoPata = node.serviceClient<camina3::TransTrayectoriaParametros>("TrayectoriaMundoPata");

    fp = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina3/datos/Nodo6.txt","w+");

        double tiempo_ahora=0.0, tiempo_anterior=0.0;
        float delta_t=0.0,delta_t2=0.0;
        float delta_x=0.0, delta_y=0.0, x_anterior=0.0, y_anterior=0.0, x_actual=0.0, y_actual=0.0;
        float vel1=0.0, vel2=0.0;

    /* Velocidad de transmision */
    Periodo = 0.1;
    f=1/Periodo;
    ros::Rate loop_rate(f);  //Frecuencia [Hz]
	while (ros::ok() && simulationRunning)
	{
		loop_rate.sleep();
		ros::spinOnce();
        for (int k=0;k<Npatas;k++) {
            if (ubicacionRobot.coordenadaPata_z[k]<=umbral_Z_Apoyo)
            {
//                 and (ubicacionRobot.pataTipFuerza_z[k]>=umbralFuerzaApoyo)){
            //    if (ubicacionRobot.pataTipFuerza_z[k]>=umbralFuerzaApoyo) {
                //-- La pata esta en apoyo
                    ubicacionRobot.pataApoyo[k]=1;
                } else {
                //-- La pata esta en transferencia
                    ubicacionRobot.pataApoyo[k]=0;
                }
        }

    //-- Calculo de velocidad de robot
    //    x_anterior = x_actual;
        y_anterior = y_actual;
    //    x_actual = msgUbicacionRobot.coordenadaCuerpo_x;
        y_actual = ubicacionRobot.coordenadaCuerpo_y;
        tiempo_ahora = ros::Time::now().toSec();
        delta_t = (float) (tiempo_ahora - tiempo_anterior);
        delta_t2 = (float) (tiempo_ahora2 - tiempo_anterior2);
    //    delta_x = fabs(x_actual-x_anterior);
        delta_y = fabs(y_actual-y_anterior);
//        ROS_INFO("\nNodo6: delta_t=%.3f, delta_y=%.3f",delta_t,delta_y);

        if (delta_t==0) {
            vel1 = 0.0;
        } else {
            vel1 = delta_y/delta_t;
        }

        if (delta_t2==0) {
            vel2 = 0.0;
        } else {
            vel2 = delta_y/delta_t2;
        }
    //-- Filtrado de medida de velocidad
        cb.push_back(vel1);
        float acc=0.0;
        for(int i=0;i<window_size;i++) acc=acc+cb[i];
        ubicacionRobot.velocidadCuerpo_y = acc/window_size;

        fprintf(fp,"%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n",tiempo_ahora,delta_t,delta_t2,vel1,vel2,Veloy_twist);

        tiempo_anterior = tiempo_ahora;
        tiempo_anterior2 = tiempo_ahora2;

        if (infoCuerpo and infoPatas){
            infoCuerpo=false;
            infoPatas=false;
//            infoVel=false;
            chatter_pub.publish(ubicacionRobot);
        }
	}
	fclose(fp);
	ROS_INFO("Adios6!");
	ros::shutdown();
	return(0);
}
