#include <stdio.h>
#include <string.h>
#include <iostream>
#include <stdlib.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <boost/circular_buffer.hpp>
#include <boost/circular_buffer_fwd.hpp>
//Librerias propias usadas
#include "camina12/constantes.hpp"
#include "camina12/vector3d.hpp"
#include "camina12/v_repConst.h"
// Used data structures:
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>
#include "camina12/UbicacionRobot.h"
// Used API services:
#include "vrep_common/VrepInfo.h"
#include "vrep_common/ObjectGroupData.h"
// Definiciones
#define tamano_ventana 11
//-- Global variables:
bool simulationRunning=true;
bool sensorTrigger=false;
bool infoCuerpo=false, infoPatas=false, infoVel=false;
float simulationTime=0.0f;
float Veloy_twist=0.0, ventana_prom[tamano_ventana];
double tiempo_ahora2=0.0;
ros::Time time_stamp;
FILE *fp,*fpc,*fp1,*fp2,*fp3,*fp4,*fp5,*fp6;
camina12::UbicacionRobot ubicacionRobot;
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
	punto3d coordenadaPata[Npatas],coordenadaPataSistemaPata[Npatas],Robot;
	Robot.x = ubicacionRobot.coordenadaCuerpo_x;
    Robot.y = ubicacionRobot.coordenadaCuerpo_y;
	float theta_Rob=ubicacionRobot.orientacionCuerpo_yaw;

	for(int k=1; k<Npatas+1; k++){
        coordenadaPata[k-1].x=ubicacionRobot.coordenadaPata_x[k-1]=msgUbicacionPatas.floatData.data[0+k*Npatas/2];
        coordenadaPata[k-1].y=ubicacionRobot.coordenadaPata_y[k-1]=msgUbicacionPatas.floatData.data[1+k*Npatas/2];
        coordenadaPata[k-1].z=ubicacionRobot.coordenadaPata_z[k-1]=msgUbicacionPatas.floatData.data[2+k*Npatas/2];
        //-- Transformacion de trayectoria a Sistema de Pata
        //-- NOTA: la trasnsformacion sobre Z no se realiza
        coordenadaPataSistemaPata[k-1] = Transformada_Mundo_Pata(k-1,Mundo_Pata,coordenadaPata[k-1],Robot,theta_Rob);
//        ROS_INFO("Nodo6: pata[%d], x=%.3f, y=%.3f",k-1,ubicacionRobot.coordenadaPata_x[k-1],ubicacionRobot.coordenadaPata_y[k-1]);
        ubicacionRobot.coordenadaPataSistemaPata_x[k-1]=coordenadaPataSistemaPata[k-1].x;
        ubicacionRobot.coordenadaPataSistemaPata_y[k-1]=coordenadaPataSistemaPata[k-1].y;
        ubicacionRobot.coordenadaPataSistemaPata_z[k-1]=coordenadaPataSistemaPata[k-1].z;
    }
    infoPatas = true;
}

void ubicacionCuerpoCallback(geometry_msgs::PoseStamped msgUbicacionCuerpo)
{
    time_stamp = msgUbicacionCuerpo.header.stamp;
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
}


// Main code:
int main(int argc,char* argv[])
{
	float Periodo=0.0, f=0.0;
	int Narg=0;
	std::string dummys,cuerpo, vel, fuerza;
    boost::circular_buffer<float> ventana_vel(tamano_ventana);

	Narg=3;

	if (argc>=Narg)
	{
	    dummys = argv[1];
	    cuerpo = argv[2];
	    vel = argv[3];
	} else {
		ROS_ERROR("Nodo6:Indique argumentos completos!\n");
		return (0);
	}

	std::string topicDummy("/vrep/");
	std::string topicCuerpo("/vrep/");
	std::string topicVelCuerpo("/vrep/");
	topicDummy+=dummys;
	topicCuerpo+=cuerpo;
	topicVelCuerpo+=vel;
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
    ros::Publisher chatter_pub = node.advertise<camina12::UbicacionRobot>("UbicacionRobot", 100);

    fp = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina12/datos/Nodo6_vel.txt","w+");
    fpc = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina12/datos/Nodo6_body.txt","w+");
    fp1 = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina12/datos/Nodo6_P1.txt","w+");
    fp2 = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina12/datos/Nodo6_P2.txt","w+");
    fp3 = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina12/datos/Nodo6_P3.txt","w+");
    fp4 = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina12/datos/Nodo6_P4.txt","w+");
    fp5 = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina12/datos/Nodo6_P5.txt","w+");
    fp6 = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina12/datos/Nodo6_P6.txt","w+");

    float delta_y=0.0, y_anterior=0.0, y_actual=0.0;
    float vel1=0.0;

    /* Velocidad de transmision */
    Periodo = 0.05;
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
        y_anterior = y_actual;
        y_actual = ubicacionRobot.coordenadaCuerpo_y;
        delta_y = fabs(y_actual-y_anterior);
//        ROS_INFO("\nNodo6: delta_t=%.3f, delta_y=%.3f",delta_t,delta_y);
        vel1 = delta_y/Periodo;
    //-- Filtrado de medida de velocidad
    //.. los primeros instantes de tiempo no son importantes porque el robot no sera controlado en ese momento
//        ventana.push_back(vel1);
//        float acc=0.0;
//        for(int i=0;i<tamano_ventana;i++) acc=acc+ventana[i];
//        ubicacionRobot.velocidadCuerpo_y = acc/tamano_ventana;
        int datoSalida=(tamano_ventana+1)/2;
        ventana_vel.push_back(vel1);
        for(int i=0;i<tamano_ventana;i++) ventana_prom[i]=ventana_vel[i];
        std::sort (ventana_prom, ventana_prom+tamano_ventana);
        ubicacionRobot.velocidadCuerpo_y = ventana_prom[datoSalida];

        fprintf(fp,"%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n",y_actual,delta_y,Veloy_twist,vel1,ubicacionRobot.velocidadCuerpo_y);
        fprintf(fpc,"%.3f\t%.3f\t%.3f\n",ubicacionRobot.coordenadaCuerpo_x,ubicacionRobot.coordenadaCuerpo_y,ubicacionRobot.orientacionCuerpo_yaw);
        if (infoCuerpo and infoPatas){
            infoCuerpo=false;
            infoPatas=false;
//            infoVel=false;
//            fprintf(fp1,"%d\t%.3f\t%.3f\t%.3f\n",ubicacionRobot.pataApoyo[0],ubicacionRobot.coordenadaPata_x[0],ubicacionRobot.coordenadaPata_y[0],ubicacionRobot.coordenadaPata_z[0]);
//            fprintf(fp2,"%d\t%.3f\t%.3f\t%.3f\n",ubicacionRobot.pataApoyo[1],ubicacionRobot.coordenadaPata_x[1],ubicacionRobot.coordenadaPata_y[1],ubicacionRobot.coordenadaPata_z[1]);
//            fprintf(fp3,"%d\t%.3f\t%.3f\t%.3f\n",ubicacionRobot.pataApoyo[2],ubicacionRobot.coordenadaPata_x[2],ubicacionRobot.coordenadaPata_y[2],ubicacionRobot.coordenadaPata_z[2]);
//            fprintf(fp4,"%d\t%.3f\t%.3f\t%.3f\n",ubicacionRobot.pataApoyo[3],ubicacionRobot.coordenadaPata_x[3],ubicacionRobot.coordenadaPata_y[3],ubicacionRobot.coordenadaPata_z[3]);
//            fprintf(fp5,"%d\t%.3f\t%.3f\t%.3f\n",ubicacionRobot.pataApoyo[4],ubicacionRobot.coordenadaPata_x[4],ubicacionRobot.coordenadaPata_y[4],ubicacionRobot.coordenadaPata_z[4]);
//            fprintf(fp6,"%d\t%.3f\t%.3f\t%.3f\n",ubicacionRobot.pataApoyo[5],ubicacionRobot.coordenadaPata_x[5],ubicacionRobot.coordenadaPata_y[5],ubicacionRobot.coordenadaPata_z[5]);
            fprintf(fp1,"%d\t%.3f\t%.3f\t%.3f\n",ubicacionRobot.pataApoyo[0],ubicacionRobot.coordenadaPataSistemaPata_x[0],ubicacionRobot.coordenadaPataSistemaPata_y[0],ubicacionRobot.coordenadaPataSistemaPata_z[0]);
            fprintf(fp2,"%d\t%.3f\t%.3f\t%.3f\n",ubicacionRobot.pataApoyo[1],ubicacionRobot.coordenadaPataSistemaPata_x[1],ubicacionRobot.coordenadaPataSistemaPata_y[1],ubicacionRobot.coordenadaPataSistemaPata_z[1]);
            fprintf(fp3,"%d\t%.3f\t%.3f\t%.3f\n",ubicacionRobot.pataApoyo[2],ubicacionRobot.coordenadaPataSistemaPata_x[2],ubicacionRobot.coordenadaPataSistemaPata_y[2],ubicacionRobot.coordenadaPataSistemaPata_z[2]);
            fprintf(fp4,"%d\t%.3f\t%.3f\t%.3f\n",ubicacionRobot.pataApoyo[3],ubicacionRobot.coordenadaPataSistemaPata_x[3],ubicacionRobot.coordenadaPataSistemaPata_y[3],ubicacionRobot.coordenadaPataSistemaPata_z[3]);
            fprintf(fp5,"%d\t%.3f\t%.3f\t%.3f\n",ubicacionRobot.pataApoyo[4],ubicacionRobot.coordenadaPataSistemaPata_x[4],ubicacionRobot.coordenadaPataSistemaPata_y[4],ubicacionRobot.coordenadaPataSistemaPata_z[4]);
            fprintf(fp6,"%d\t%.3f\t%.3f\t%.3f\n",ubicacionRobot.pataApoyo[5],ubicacionRobot.coordenadaPataSistemaPata_x[5],ubicacionRobot.coordenadaPataSistemaPata_y[5],ubicacionRobot.coordenadaPataSistemaPata_z[5]);

             chatter_pub.publish(ubicacionRobot);
        }
	}
	fclose(fp);
	fclose(fp1);
	fclose(fp2);
	fclose(fp3);
	fclose(fp4);
	fclose(fp5);
	fclose(fp6);
	ROS_INFO("Adios6!");
	ros::shutdown();
	return(0);
}
