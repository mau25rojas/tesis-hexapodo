#include "ros/ros.h"
//Librerias propias usadas
#include "constantes.hpp"
#include "camina10/v_repConst.h"
#include "camina10/EspacioTrabajoParametros.h"
#include "../../Convexhull/vector3d.hpp"
#include "../../Convexhull/convexhull.cpp"
#include "../../Convexhull/analisis.cpp"
// Used API services:
#include "vrep_common/VrepInfo.h"
//-- Variables globales
bool simulationRunning=true;
bool sensorTrigger=false;
float simulationTime=0.0f;
float PosicionPata_x=0.0, PosicionPata_y=0.0, PosicionPata_x2=0.0, PosicionPata_y2=0.0, anguloPatas_rad=0.0;
float rotacionPata[Npatas], phi[Npatas], ang_offset=90*pi/180;
punto3d origenPata[Npatas], Offset;
//-- Log
FILE *fp1;
//-- Funciones
punto3d TransformacionHomogenea(punto3d Punto_in, punto3d L_traslacion, float ang_rotacion);

void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}

bool EspacioTrabajoPatas(camina10::EspacioTrabajoParametros::Request  &req,
                        camina10::EspacioTrabajoParametros::Response &res)
{
    int Npata = req.Pata;
    float ang_rotacion=0.0;
    punto3d P1,P2,P3,P4,P_aux,P_EDT,P_Ocuerpo;

    //-- Esquinas ((..[1]izq-arrib,[2]der-arrib,[3]der-aba,[4]izq-aba..))
    res.EspacioTrabajoP1_x=0.0;
    res.EspacioTrabajoP1_y=0.0;
    res.EspacioTrabajoP2_x=0.0;
    res.EspacioTrabajoP2_y=0.0;
    res.EspacioTrabajoP3_x=0.0;
    res.EspacioTrabajoP3_y=0.0;
    res.EspacioTrabajoP4_x=0.0;
    res.EspacioTrabajoP4_y=0.0;

//-- Transformacion 1: trayectoria
    //--Punto1 - izq/arr
    P_aux.x = EspacioTrabajo_X1;
    P_aux.y = EspacioTrabajo_Y1;
    P1 = TransformacionHomogenea(P_aux,Offset,phi[Npata]+ang_offset);//+ang_offset+req.alfa);
    //--Punto2 - der/arr
    P_aux.x = EspacioTrabajo_X2;
    P_aux.y = EspacioTrabajo_Y1;
    P2 = TransformacionHomogenea(P_aux,Offset,phi[Npata]+ang_offset);//+ang_offset+req.alfa);
    //--Punto3 - der/aba
    P_aux.x = EspacioTrabajo_X2;
    P_aux.y = EspacioTrabajo_Y2;
    P3 = TransformacionHomogenea(P_aux,Offset,phi[Npata]+ang_offset);//+ang_offset+req.alfa);
    //--Punto4 - izq/aba
    P_aux.x = EspacioTrabajo_X1;
    P_aux.y = EspacioTrabajo_Y2;
    P4 = TransformacionHomogenea(P_aux,Offset,phi[Npata]+ang_offset);//+ang_offset+req.alfa);

//-- Transformacion 2: a origen de cada pata
    //--Punto1
    P_aux = P1;
    P1 = TransformacionHomogenea(P_aux,origenPata[Npata],rotacionPata[Npata]);
    //--Punto2
    P_aux = P2;
    P2 = TransformacionHomogenea(P_aux,origenPata[Npata],rotacionPata[Npata]);
    //--Punto3
    P_aux = P3;
    P3 = TransformacionHomogenea(P_aux,origenPata[Npata],rotacionPata[Npata]);
    //--Punto4
    P_aux = P4;
    P4 = TransformacionHomogenea(P_aux,origenPata[Npata],rotacionPata[Npata]);


    res.EspacioTrabajoP1_x = P1.x;
    res.EspacioTrabajoP1_y = P1.y;
    res.EspacioTrabajoP2_x = P2.x;
    res.EspacioTrabajoP2_y = P2.y;
    res.EspacioTrabajoP3_x = P3.x;
    res.EspacioTrabajoP3_y = P3.y;
    res.EspacioTrabajoP4_x = P4.x;
    res.EspacioTrabajoP4_y = P4.y;

    fprintf(fp1,"%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n",P1.x,P1.y,P2.x,P2.y,P3.x,P3.y,P4.x,P4.y);

//-- Transformacion 3: segun posicion y rotacion del robot
//    P_Ocuerpo.x = req.PosicionCuerpo_x;
//    P_Ocuerpo.y = req.PosicionCuerpo_y;
//    ang_rotacion = req.theta_CuerpoRobot;
//
//    P_EDT = TransformacionHomogenea(P1,P_Ocuerpo,ang_rotacion);
//    res.EspacioTrabajoP1_x = P_EDT.x;
//    res.EspacioTrabajoP1_y = P_EDT.y;
//    P_EDT = TransformacionHomogenea(P2,P_Ocuerpo,ang_rotacion);
//    res.EspacioTrabajoP2_x = P_EDT.x;
//    res.EspacioTrabajoP2_y = P_EDT.y;
//    P_EDT = TransformacionHomogenea(P3,P_Ocuerpo,ang_rotacion);
//    res.EspacioTrabajoP3_x = P_EDT.x;
//    res.EspacioTrabajoP3_y = P_EDT.y;
//    P_EDT = TransformacionHomogenea(P4,P_Ocuerpo,ang_rotacion);
//    res.EspacioTrabajoP4_x = P_EDT.x;
//    res.EspacioTrabajoP4_y = P_EDT.y;

  return true;
}

int main(int argc, char **argv)
{
    int Narg=0;
      Narg=8;
	if (argc>=Narg)
	{
        Offset.x = atof(argv[1]);   //offset de la trayectoria
        Offset.y = atof(argv[2]);   //offset de la trayectoria
        for(int k=0;k<Npatas;k++) phi[k]=atof(argv[3+k])*pi/180.0;
    } else {
		ROS_ERROR("server_EspacioTrabajo: Indique argumentos!\n");
		return 0;
	}
    ros::init(argc, argv, "EspacioTrabajo_server");
    ros::NodeHandle n;
    ROS_INFO("server_EspacioTrabajo just started\n");

    ros::Subscriber subInfo=n.subscribe("/vrep/info",1,infoCallback);
    ros::ServiceServer service = n.advertiseService("EspacioTrabajo", EspacioTrabajoPatas);

//    for(int k=0;k<Npatas;k++) ROS_INFO("%.3f\n",phi[k]*180.0/pi);

//    anguloPatas_rad = anguloPatas*pi/180;
    PosicionPata_x = radioCuerpo*cos(anguloPatas);
    PosicionPata_y = radioCuerpo*sin(anguloPatas);
    PosicionPata_x2 = radioCuerpo;
    PosicionPata_y2 = 0.0;
    //-- Pata1
    origenPata[0].x=-PosicionPata_x;
    origenPata[0].y=PosicionPata_y;
    rotacionPata[0]=rotacion_Pata1;
    //-- Pata2
    origenPata[1].x=PosicionPata_x;
    origenPata[1].y=PosicionPata_y;
    rotacionPata[1]=rotacion_Pata2;
    //-- Pata3
    origenPata[2].x=-PosicionPata_x2;
    origenPata[2].y=PosicionPata_y2;
    rotacionPata[2]=rotacion_Pata3;
    //-- Pata4
    origenPata[3].x=PosicionPata_x2;
    origenPata[3].y=PosicionPata_y2;
    rotacionPata[3]=rotacion_Pata4;
    //-- Pata5
    origenPata[4].x=-PosicionPata_x;
    origenPata[4].y=-PosicionPata_y;
    rotacionPata[4]=rotacion_Pata5;
    //-- Pata6
    origenPata[5].x=PosicionPata_x;
    origenPata[5].y=-PosicionPata_y;
    rotacionPata[5]=rotacion_Pata6;

    fp1 = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina10/datos/LogEDT.txt","w+");

  while (ros::ok() && simulationRunning) {
    ros::spinOnce();
  }

  return 0;
}

punto3d TransformacionHomogenea(punto3d Punto_in, punto3d L_traslacion, float ang_rotacion){

    punto3d Punto_out;

    Punto_out.x = L_traslacion.x + Punto_in.x*cos(ang_rotacion) - Punto_in.y*sin(ang_rotacion);
    Punto_out.y = L_traslacion.y + Punto_in.x*sin(ang_rotacion) + Punto_in.y*cos(ang_rotacion);
    Punto_out.z = L_traslacion.z + Punto_in.z;

    return(Punto_out);
}
