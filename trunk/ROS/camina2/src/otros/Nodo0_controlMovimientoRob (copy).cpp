#include "ros/ros.h"
#include "std_msgs/String.h"
#include "math.h"
//Librerias propias usadas
#include "constantes.hpp"
#include "camina/v_repConst.h"
// Used data structures:
#include <geometry_msgs/PoseStamped.h>
#include "../msg_gen/cpp/include/camina/DatosTrayectorias.h"
#include "../msg_gen/cpp/include/camina/Punto.h"
#include "camina/TransTrayectoriaParametros.h"
#include "../msg_gen/cpp/include/camina/UbicacionRobot.h"
// Used API services:
#include "vrep_common/VrepInfo.h"
#include "vrep_common/simRosGetObjectPose.h"
//Clientes y Servicios
ros::ServiceClient client_simRosGetObjectPose;
vrep_common::simRosGetObjectPose srv_simRosGetObjectPose; //Servicio para obtener posicion del robot
ros::ServiceClient client_TransTrayectoria_MundoPata;
camina::TransTrayectoriaParametros srv_TransTrayectoria_MundoPata;
ros::ServiceClient client_TransTrayectoria_PataMundo;
camina::TransTrayectoriaParametros srv_TransTrayectoria_PataMundo;
// Funciones
void Trayectoria_Eliptica(int Npata,float t_Trayectoria,float PuntoInicio_x,float PuntoInicio_y,float PuntoInicio_z);

// variables Globales
bool simulationRunning=true;
bool sensorTrigger=false;
float simulationTime=0.0f;
float divisionTrayectoriaPata=0.0, beta=0.0, lambda_Apoyo[Npatas],lambda_Transferencia[Npatas], dh=0.0, desfasaje_t[6];
float x_Tray=0.0, y_Tray=0.0, z_Tray=0.0;
float x_FinApoyo=0.0,y_FinApoyo=0.0,z_FinApoyo=0.0;
bool finTrayectoria=false;
float PosicionCuerpo_y=0.0, PosicionCuerpo_x=0.0, theta_CuerpoRobot=0.0;
float trayectoriaPata_x[Npatas][100],trayectoriaPata_y[Npatas][100],trayectoriaPata_z[Npatas][100];
float posicionActualPata_x[Npatas],posicionActualPata_y[Npatas],posicionActualPata_z[Npatas];
//float Tripode_desfasaje_t[6]={0.0,0.5,0.5,0.0,0.0,0.5};
//float Sixpode_desfasaje_t[6]={0.0,3/6,1/6,4/6,2/6,5/6};
int k=1;
//camina::DatosTrayectorias TraysPatas;
camina::DatosTrayectorias TraysPatasEnvio;
FILE *fp, *fp1;

// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}

//void ajusteCallback(const camina::DatosTrayectoriaPata msg_DatosTrayectoriaPata)
//{
//     for(k=0;k<6;k++){
//        datoTrayectoriaPata.beta[k]=msg_DatosTrayectoriaPata.beta[k];
//        datoTrayectoriaPata.lambda2[k]=msg_DatosTrayectoriaPata.lambda2[k];
//     }
//
//}

void ubicacionRobCallback(camina::UbicacionRobot msgUbicacionRobot)
{
    PosicionCuerpo_x = msgUbicacionRobot.coordenadaCuerpo_x;
    PosicionCuerpo_y = msgUbicacionRobot.coordenadaCuerpo_y;
    theta_CuerpoRobot = msgUbicacionRobot.orientacionCuerpo_yaw;
//    ROS_WARN("Posicion cuerpo: x=%.3f, y=%.3f, theta=%.3f",PosicionCuerpo_x,PosicionCuerpo_y,theta_CuerpoRobot);
//    for(int k=0; k<Npatas;k++) {
//        posicionActualPata_x[k] = msgUbicacionRobot.coordenadaPata_x[k];
//        posicionActualPata_y[k] = msgUbicacionRobot.coordenadaPata_y[k];
//        posicionActualPata_z[k] = msgUbicacionRobot.coordenadaPata_z[k];
//    }
}

int main(int argc, char **argv)
{
  float T=0.0, divisionTiempo=0.0, f=0.0;
  float x_Offset=0.0, y_Offset=0.0, z_Offset=0.0, phi=0.0, phi_rad=0.0;
  float alfa[Npatas], alfa_rad[Npatas],desfasaje_t[Npatas];
  float lambda_Apoyo_arg=0.0,lambda_Transferencia_arg=0.0;
  int i=0, Narg=0,PataTipHandle[Npatas];
  float delta_t=0.0;
  geometry_msgs::PoseStamped PataTipPose[Npatas];

  Narg=25;
      // (when V-REP launches this executable, V-REP will also provide the argument list)
	//numero de argumentos que mande (se excluye el fantasma que el manda solo)
	if (argc>=Narg)
	{
		T=atof(argv[1]); // Periodo de trayectoria [seg]
		divisionTrayectoriaPata=atof(argv[2]);  //N puntos
		beta=atof(argv[3]);
		lambda_Apoyo_arg=atof(argv[4]);
		lambda_Transferencia_arg=atof(argv[5]);
		dh=atof(argv[6]);
		x_Offset=atof(argv[7]);
		y_Offset=atof(argv[8]);
		z_Offset=atof(argv[9]);
		phi=atof(argv[10]);
		PosicionCuerpo_x=atof(argv[11]);
		PosicionCuerpo_y=atof(argv[12]);
		theta_CuerpoRobot=atof(argv[13]);
		for (i=0;i<Npatas;i++) alfa[i] = atof(argv[14+i]);
        for (i=0;i<Npatas;i++) desfasaje_t[i] = atof(argv[14+Npatas+i]);
        for (i=0;i<Npatas;i++) PataTipHandle[i] = atoi(argv[14+2*Npatas+i]);
    } else {
		ROS_ERROR("Nodo0: Indique argumentos!\n");
		sleep(5000);
		return 0;
	}

    /*Inicio nodo de ROS*/
    ros::init(argc, argv, "Nodo0_controlMovimientoRob");
    ros::NodeHandle node;
    ROS_INFO("Nodo0_controlMovimientoRob just started\n");

//Topicos susbcritos y publicados
    ros::Publisher chatter_pub1 = node.advertise<camina::DatosTrayectorias>("datosTrayectoriaPata", 100);
//  ros::Publisher chatter_pub2 = node.advertise<camina::AngulosMotor>("DatosDeMotores", 100);
    ros::Subscriber subInfo1=node.subscribe("/vrep/info",100,infoCallback);
//  ros::Subscriber subInfo2=node.subscribe("PlanificacionDePisada",100,ajusteCallback);
    ros::Subscriber subInfo2=node.subscribe("UbicacionRobot",100,ubicacionRobCallback);
//Clientes y Servicios
    client_simRosGetObjectPose=node.serviceClient<vrep_common::simRosGetObjectPose>("/vrep/simRosGetObjectPose");
    srv_simRosGetObjectPose.request.relativeToObjectHandle=-1;
    client_TransTrayectoria_MundoPata=node.serviceClient<camina::TransTrayectoriaParametros>("TrayectoriaMundoPata");
    client_TransTrayectoria_PataMundo=node.serviceClient<camina::TransTrayectoriaParametros>("TrayectoriaMundoPata");

    // Prepara variables para calculos de trayectoria de PATA
    //delta_t = 1/divisionTrayectoriaPata;
    divisionTiempo = T/divisionTrayectoriaPata;
    f=1/divisionTiempo;
    /* La velocidad de envío de los datos se encarga de darme el tiempo total de trayectoria deseado */
    /* Velocidad de transmision */
    ros::Rate loop_rate(f);  //Frecuencia [Hz]

    for(i=0;i<Npatas;i++) {
        lambda_Apoyo[i] = lambda_Apoyo_arg;
        lambda_Transferencia[i]=lambda_Transferencia_arg;
        alfa_rad[i] = alfa[i]*pi/180;
    }
	phi_rad = phi*pi/180;

    float t_Trayectoria=0.0;
    float x_S1=0.0, y_S1=0.0, z_S1=0.0;
    float x_S0=0.0, y_S0=0.0, z_S0=0.0;
    float PuntoInicio_x=0.0, PuntoInicio_y=0.0, PuntoInicio_z=0.0;

    fp = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina/datos/SalidaTray.txt","w+");
    fp1 = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina/datos/SalidaTrayMundo.txt","w+");

ROS_INFO("Inicio loop de trayectoria");
while (ros::ok() && simulationRunning){

for(int k=0; k<Npatas; k++){
    srv_simRosGetObjectPose.request.handle=PataTipHandle[k];
    if (client_simRosGetObjectPose.call(srv_simRosGetObjectPose)&&(srv_simRosGetObjectPose.response.result!=-1)){
        PataTipPose[k] = srv_simRosGetObjectPose.response.pose;
        posicionActualPata_x[k] = PataTipPose[k].pose.position.x;
        posicionActualPata_y[k] = PataTipPose[k].pose.position.y;
        posicionActualPata_z[k] = PataTipPose[k].pose.position.z;

    } else {
            ROS_ERROR("Nodo0: servicio de posicion pata no funciona\n");
            ROS_ERROR("Nodo0: respuesta = %d\n",srv_simRosGetObjectPose.response.result);
    }
    for(int l=0; l<(int)divisionTrayectoriaPata; l++){
        ros::spinOnce();
        delta_t = delta_t + 1/divisionTrayectoriaPata;
        if (fabs(delta_t-1)<umbralDiferencia) delta_t = 0.0;
        //--------Temporizacion----------
        t_Trayectoria=delta_t+desfasaje_t[k];
        t_Trayectoria=fmod(t_Trayectoria,1);
        //-----Parametrizacion de trayectoria eliptica en Sistema de coordenadas generico
//        if (l==0){
////            ROS_INFO("posicionActualPata_x=%.3f,posicionActualPata_y=%.3f,posicionActualPata_z=%.3f",posicionActualPata_x[k],posicionActualPata_y[k],posicionActualPata_z[k]);
//        //--Busqueda de punto de inicio de trayectoria
//            srv_TransTrayectoria_PataMundo.request.modo = Pata_Mundo;
//            srv_TransTrayectoria_PataMundo.request.Npata = k;
//            srv_TransTrayectoria_PataMundo.request.x_S0 = posicionActualPata_x[k];
//            srv_TransTrayectoria_PataMundo.request.y_S0 = posicionActualPata_y[k];
//            srv_TransTrayectoria_PataMundo.request.z_S0 = posicionActualPata_z[k];
//            srv_TransTrayectoria_PataMundo.request.x_UbicacionRob = PosicionCuerpo_x;
//            srv_TransTrayectoria_PataMundo.request.y_UbicacionRob = PosicionCuerpo_y;
//            srv_TransTrayectoria_PataMundo.request.theta_Rob = theta_CuerpoRobot;
//            if (client_TransTrayectoria_PataMundo.call(srv_TransTrayectoria_PataMundo))
//            {   x_S1 = srv_TransTrayectoria_PataMundo.response.x_Mundo;
//                y_S1 = srv_TransTrayectoria_PataMundo.response.y_Mundo;
//                z_S1 = srv_TransTrayectoria_PataMundo.response.z_Mundo;
//            } else {
//                ROS_ERROR("Nodo0: servicio de TransTrayectoria no funciona\n");
//            }
////            ROS_INFO("x_S1=%.3f,y_S1=%.3f,z_S1=%.3f",x_S1,y_S1,z_S1);
//            PuntoInicio_x = -(x_Offset*cos(alfa_rad[k]+phi_rad)+y_Offset*sin(alfa_rad[k]+phi_rad)) + x_S1*cos(alfa_rad[k]+phi_rad) + y_S1*sin(alfa_rad[k]+phi_rad);
//            PuntoInicio_y = -(-x_Offset*sin(alfa_rad[k]+phi_rad)+y_Offset*cos(alfa_rad[k]+phi_rad)) - x_S1*sin(alfa_rad[k]+phi_rad) + y_S1*cos(alfa_rad[k]+phi_rad);
//            PuntoInicio_z = z_S1;
//            ROS_INFO("PuntoInicio_x=%.3f,PuntoInicio_y=%.3f,PuntoInicio_z=%.3f",PuntoInicio_x,PuntoInicio_y,PuntoInicio_z);
//        //-------------------------------------------------------------------------------
//        }
        Trayectoria_Eliptica(k,t_Trayectoria,-lambda_Apoyo[k]/2,0.0,0.0);
        //----Rotacion y traslacion segun trayectoria deseada
        x_S0 = x_Offset + x_Tray*cos(alfa_rad[k]+phi_rad) - y_Tray*sin(alfa_rad[k]+phi_rad);
        y_S0 = y_Offset + x_Tray*sin(alfa_rad[k]+phi_rad) + y_Tray*cos(alfa_rad[k]+phi_rad);
        z_S0 = z_Offset + z_Tray;
//        if(k==(Pata4-1)) fprintf(fp,"%.3f,%.3f,%.3f\n",x_S0,y_S0,z_S0);
        //-----Transformacion de trayectoria a Sistema de Mundo
        srv_TransTrayectoria_MundoPata.request.modo = Mundo_Pata;
        srv_TransTrayectoria_MundoPata.request.Npata = k;
        srv_TransTrayectoria_MundoPata.request.x_S0 = x_S0;
        srv_TransTrayectoria_MundoPata.request.y_S0 = y_S0;
        srv_TransTrayectoria_MundoPata.request.z_S0 = z_S0;
        srv_TransTrayectoria_MundoPata.request.x_UbicacionRob = PosicionCuerpo_x;
        srv_TransTrayectoria_MundoPata.request.y_UbicacionRob = PosicionCuerpo_y;
        srv_TransTrayectoria_MundoPata.request.theta_Rob = theta_CuerpoRobot;
        if (client_TransTrayectoria_MundoPata.call(srv_TransTrayectoria_MundoPata))
        {   x_S1 = srv_TransTrayectoria_MundoPata.response.x_Pata;
            y_S1 = srv_TransTrayectoria_MundoPata.response.y_Pata;
            z_S1 = srv_TransTrayectoria_MundoPata.response.z_Pata;
    //                ROS_INFO("Servicio motores: q1=%.3f; q2=%.3f; q3=%.3f\n", _q1, _q2, _q3);
        } else {
            ROS_ERROR("Nodo0: servicio de TransTrayectoria no funciona\n");
        }
//        if(k==(Pata4-1)) {
//            fprintf(fp1,"%.3f,%.3f,%.3f\n",x_S1,y_S1,z_S1);
////            ROS_INFO("Nodo0: TransTrayectoria Mundo: pata[%d] x=%.3f; y=%.3f; z=%.3f\n",Pata4,x_S1,y_S1,z_S1);
//        }
        trayectoriaPata_x[k][l] = x_S1;
        trayectoriaPata_y[k][l] = y_S1;
        trayectoriaPata_z[k][l] = z_S1;
    } //fin for patas
}
for(int l=0; l<(int)divisionTrayectoriaPata; l++){
    ros::spinOnce();
    TraysPatasEnvio.x_UbicacionRob = PosicionCuerpo_x;
    TraysPatasEnvio.y_UbicacionRob = PosicionCuerpo_y;
    TraysPatasEnvio.theta_Rob = theta_CuerpoRobot;
    for(int k=0; k<Npatas; k++){
        TraysPatasEnvio.trayectoriaPata[k].x=trayectoriaPata_x[k][l];
        TraysPatasEnvio.trayectoriaPata[k].y=trayectoriaPata_y[k][l];
        TraysPatasEnvio.trayectoriaPata[k].z=trayectoriaPata_z[k][l];
    }
    loop_rate.sleep();
    chatter_pub1.publish(TraysPatasEnvio);
} //Fin for evio
}

    ROS_INFO("Adios0!");
    fclose(fp);
    fclose(fp1);
    ros::shutdown();
    return 0;
}

void Trayectoria_Eliptica(int Npata,float t_Trayectoria,float PuntoInicio_x,float PuntoInicio_y,float PuntoInicio_z) {

    float x_aux=0.0, y_aux=0.0, z_aux=0.0;
    float velocidadApoyo=0.0;
    float alfa_arg=0.0, t_aux=0.0;

    velocidadApoyo = lambda_Apoyo[Npata]/beta;
    //---Caso parte 1 trayectoria----
    // Periodo A-B
    if (0<t_Trayectoria && t_Trayectoria<=beta) {
        x_aux = PuntoInicio_x + velocidadApoyo*t_Trayectoria;
        y_aux = PuntoInicio_y;
        z_aux = PuntoInicio_z;
        if (fabs(t_Trayectoria-beta)<=(1/divisionTrayectoriaPata)){
            x_FinApoyo = x_aux;
            y_FinApoyo = y_aux;
            z_FinApoyo = z_aux;
        }
    } else {
    //---Caso parte 2 trayectoria------
    // Elipsis
        t_aux = (t_Trayectoria-beta)/(1-beta);
        alfa_arg = pi*t_aux;
//                    a = lambda_Transferencia/2;
//                    b = dh;
//                    x = a*cos(alfa);
//                    z = b*sin(alfa);
        x_aux = x_FinApoyo + (lambda_Transferencia[Npata]/2)*cos(alfa_arg);
        y_aux = y_FinApoyo;
        z_aux = z_FinApoyo + dh*sin(alfa_arg);
        if (fabs(t_Trayectoria-1)<=(1/divisionTrayectoriaPata)) finTrayectoria=true;
    }
    x_Tray = x_aux;
    y_Tray = y_aux;
    z_Tray = z_aux;
}

//void CalculoTrayectoria_Cuadrada (int Npata, float t_Trayectoria) {
//
////    int seleccionTramo=0;
//    float velocidadApoyo=0.0, velocidadTransferencia=0.0;
//    float t_TramoABC=0.0, t_TramoABCD=0.0, t_Tramo=0.0;
//
//    velocidadApoyo = lambda_Apoyo[Npata]/beta;
//	velocidadTransferencia = (2*dh+lambda_Apoyo[Npata])/(1-beta);
//    /* Divisiones de tiempo trayectoria A-B-C-D-A*/
//    t_TramoABC = (dh/velocidadTransferencia)+beta;
//    t_TramoABCD = ((dh+lambda_Apoyo[Npata])/velocidadTransferencia)+beta;
//
//     /* Nota Mental:
//        Prueba con trayectoria rectangular: existen 3 tramos/casos.
//        Para cada tramo es necesario reiniciar el tiempo para pasarlo
//        ..a la función que calcula la trayectoria (en base a la velocidad)
//     */
//    //---Caso parte 1 trayectoria----
//    // Periodo A-B
//    if (0<=t_Trayectoria && t_Trayectoria<beta)
//        {
//            t_Tramo = t_Trayectoria;
//            x_Tray = -lambda_Apoyo[Npata]/2 + velocidadApoyo*t_Tramo;
//            y_Tray =0.0;
//            z_Tray = -dh/2;
//        }
//    //---Caso parte 2 trayectoria------
//    // Periodo B-C
//     if (beta<=t_Trayectoria && t_Trayectoria<t_TramoABC)
//    	{
//            t_Tramo = t_Trayectoria-beta;
//            x_Tray = lambda_Apoyo[Npata]/2;
//            y_Tray = 0.0;
//            z_Tray = -dh/2 + velocidadTransferencia*t_Tramo;
//        }
//    //---Caso parte 3 trayectoria------
//    // Periodo C-D
//     if (t_TramoABC<=t_Trayectoria && t_Trayectoria<t_TramoABCD)
//    	{
//            t_Tramo = t_Trayectoria-t_TramoABC;
//            x_Tray = lambda_Apoyo[Npata]/2 - velocidadTransferencia*t_Tramo;
//            y_Tray = 0.0;
//            z_Tray = dh/2;
//        }
//    //---Caso parte 4 trayectoria---------
//    //Periodo D-A
//     if (t_TramoABCD<=t_Trayectoria && t_Trayectoria<1)
//    	{
//            t_Tramo = t_Trayectoria-t_TramoABCD;
//            x_Tray = -lambda_Apoyo[Npata]/2;
//            y_Tray = 0.0;
//            z_Tray = dh/2 - velocidadTransferencia*t_Tramo;
//        }
//}
//
