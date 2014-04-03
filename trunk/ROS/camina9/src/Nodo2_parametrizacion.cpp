#include "ros/ros.h"
#include "math.h"
#include "string.h"
//Librerias propias usadas
#include "constantes.hpp"
#include "camina9/v_repConst.h"
#include "../../Convexhull/vector3d.hpp"
#include "../../Convexhull/convexhull.cpp"
#include "../../Convexhull/analisis.cpp"
// Used data structures:
#include "camina9/DatosTrayectoriaPata.h"
#include "camina9/AngulosMotor.h"
#include "camina9/CinversaParametros.h"
#include "camina9/TransHomogeneaParametros.h"
// Used API services:
#include "vrep_common/VrepInfo.h"
//Definiciones
//#define TrayInicio -0.05
//Clientes y Servicios
ros::ServiceClient client_Cinversa;
camina9::CinversaParametros srv_Cinversa;
ros::ServiceClient client_TransHomogenea;
camina9::TransHomogeneaParametros srv_TransHomogenea;

//-- Global variables (modified by topic subscribers):
bool simulationRunning=true;
bool sensorTrigger=false;
float simulationTime=0.0f;
int Npata_arg=0, Tripode=0, Estado=0, prevEstado=0;
bool Inicio=true;
//-- Calculo de trayectoria
float T=0.0, beta=0.0, lambda_Apoyo=0.0, lambda_Transferencia=0.0, dh=0.0, desfasaje_t=0.0, phi=0.0;
punto3d Offset, P_oA, P_oT, P0;

camina9::AngulosMotor qMotor;
ros::Publisher chatter_pub;
FILE *fp1;
//-- Funciones
punto3d Trayectoria_FaseApoyo(float t_Trayectoria,punto3d PInicio);
punto3d Trayectoria_FaseTrans_Eliptica(float t_Trayectoria,punto3d PInicio);
punto3d TransformacionHomogenea(punto3d Punto_in, punto3d L_traslacion, float ang_rotacion);
punto3d TransformacionHomogenea_Inversa(punto3d Punto_in, punto3d L_traslacion, float ang_rotacion);

// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}

/* Callback que escucha el topico DatosDeTrayectoria calcula la trayectoria
   deseaday cinematica inversa para de motores y los publica
*/
void datosCallback(const camina9::DatosTrayectoriaPata msg_datoTrayectoria)
{
    int correccion_ID=-1, cambioEstado=0;
    float t_Trayectoria=0.0,alfa=0.0,InicioApoyo=0.0,correccion_di=0.0;
    punto3d P1, PInicio, P_o;

    T = msg_datoTrayectoria.T_apoyo[Tripode-1];
	t_Trayectoria = msg_datoTrayectoria.t_Trayectoria[Tripode-1];
    lambda_Apoyo = msg_datoTrayectoria.lambda_Apoyo[Tripode-1];
    lambda_Transferencia = msg_datoTrayectoria.lambda_Transferencia[Tripode-1];
    alfa = msg_datoTrayectoria.alfa[Tripode-1];
    desfasaje_t = msg_datoTrayectoria.desfasaje_t[Tripode-1];
    Estado = msg_datoTrayectoria.vector_estados[Tripode-1];
    cambioEstado = msg_datoTrayectoria.cambio_estado[Tripode-1];
    correccion_di = msg_datoTrayectoria.correccion_di[Npata_arg-1];
    correccion_ID = msg_datoTrayectoria.correccion_ID[Npata_arg-1];
    P_o.x = msg_datoTrayectoria.posicionActualSistemaPata_x[Npata_arg-1];
    P_o.y = msg_datoTrayectoria.posicionActualSistemaPata_y[Npata_arg-1];
    P_o.z = msg_datoTrayectoria.posicionActualSistemaPata_z[Npata_arg-1];

    if(cambioEstado==1) {
//        P_oA = TransformacionHomogenea_Inversa(P0, Offset, phi+alfa);
//        P_oT = TransformacionHomogenea_Inversa(P0, Offset, phi+alfa);
//        ROS_WARN("cambio de estado, correccion_di=%.4f",correccion_di);
//        P_oA = P0;
//        P_oA.x = P_oT.x+correccion_di;
//        P_oT = P0;
//        P_oT.x = P_oT.x-lambda_Transferencia/2-correccion_di;
        P_oA = P_oT = P0;
        P_oT.x = P_oT.x-lambda_Transferencia/2;
    }

    if (Inicio){
        Inicio = false;
        P_oA.x = (Offset.y-FinEspacioTrabajo_y)-lambda_maximo;
        P_oA.y = 0.0;
        P_oA.z = 0.0;
        P_oT.x = (Offset.y-FinEspacioTrabajo_y)-lambda_maximo+lambda_Transferencia/2;
        P_oT.y = 0.0;
        P_oT.z = 0.0;
    }

//    InicioApoyo = (y_Offset-FinEspacioTrabajo_y)-lambda_maximo+correccion_di;

    //-----Parametrizacion de trayectoria eliptica en Sistema de Robot
    // Periodo A-B
    if (Estado==Apoyo)
    {
    //---Apoyo------
        PInicio=P_oA;
        P0 = Trayectoria_FaseApoyo(t_Trayectoria,PInicio);
    } else {
    //---Transferencia------
    // Elipsis
        PInicio=P_oT;
        P0 = Trayectoria_FaseTrans_Eliptica(t_Trayectoria,PInicio);
    }

    //-----Transformacion de trayectoria a Sistema de Pata
    P1 = TransformacionHomogenea(P0, Offset, phi+alfa);
//    P1.x = Offset.x + P0.x*cos(phi+alfa) - P0.y*sin(phi+alfa);
//    P1.y = Offset.y + P0.x*sin(phi+alfa) + P0.y*cos(phi+alfa);
//    P1.z = Offset.z + P0.z;
    //-----Cinematica Inversa
    srv_Cinversa.request.x = P1.x;
    srv_Cinversa.request.y = P1.y;
    srv_Cinversa.request.z = P1.z;

    if (client_Cinversa.call(srv_Cinversa)&&(srv_Cinversa.response.result!=-1))
    {   qMotor.q1 = srv_Cinversa.response.q1;
        qMotor.q2 = srv_Cinversa.response.q2;
        qMotor.q3 = srv_Cinversa.response.q3;
    } else {
        ROS_ERROR("Nodo 2::[%d] servicio de Cinversa no funciona\n",Npata_arg);
//        return;
    }

    fprintf(fp1,"%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",qMotor.q1,qMotor.q2,qMotor.q3,t_Trayectoria,P1.x,P1.y,P1.z);
    //---Publica angulos motores----
    qMotor.Npata = Npata_arg;
	chatter_pub.publish(qMotor);
}

int main(int argc, char **argv){

	if (argc>=8)
	{
		Npata_arg=atoi(argv[1]);    //de 1 a 6
		dh=atof(argv[2]);
        Offset.x=atof(argv[3]);
        Offset.y=atof(argv[4]);
        Offset.z=atof(argv[5]);
        phi=atof(argv[6])*pi/180.0;
        Tripode=atof(argv[7]);
	}
	else
	{
		ROS_ERROR("Nodo 2::[%d] Indique argumentos!\n",Npata_arg);
		sleep(5000);
		return 0;
	}

    /*Inicio nodo de ROS*/
    std::string nodeName("Nodo2_Parametrizacion_pata");
	std::string Id(boost::lexical_cast<std::string>(Npata_arg));
	nodeName+=Id;
	ros::init(argc,argv,nodeName.c_str());
    ros::NodeHandle node;
    //ROS_INFO("Nodo2_Parametrizacion just started\n");

//-- Topicos susbcritos y publicados
    chatter_pub = node.advertise<camina9::AngulosMotor>("DatosDeMotores", 100);
    ros::Subscriber subInfo = node.subscribe("/vrep/info",1,infoCallback);
//-- Recibe topico especifico
    ros::Subscriber sub = node.subscribe("datosTrayectoria", 100, datosCallback);
//-- Clientes y Servicios
    client_Cinversa = node.serviceClient<camina9::CinversaParametros>("Cinversa");
    client_TransHomogenea = node.serviceClient<camina9::TransHomogeneaParametros>("TransHomogenea");

    std::string fileName("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina9/datos/QXEnviada_Pata");
    std::string texto(".txt");
    fileName+=Id;
    fileName+=texto;
    fp1 = fopen(fileName.c_str(),"w+");

    while (ros::ok() && simulationRunning)
    {
	  ros::spinOnce();
    }
    fclose(fp1);
    //ROS_INFO("Adios2!");
    ros::shutdown();
    return 0;
}

//---Caso parte 1 trayectoria----
//-- Periodo A-B
punto3d Trayectoria_FaseApoyo(float t_Trayectoria,punto3d PInicio){

    punto3d salida;
    float velocidadApoyo=0.0;

    velocidadApoyo = lambda_Apoyo/T;

    salida.x = PInicio.x + velocidadApoyo*t_Trayectoria;
    salida.y = PInicio.y;
    salida.z = PInicio.z;

    return(salida);
}

//---Caso parte 2 trayectoria----
//-- Elipsis
punto3d Trayectoria_FaseTrans_Eliptica(float t_Trayectoria,punto3d PInicio){

    punto3d salida;
    float theta=0.0, t_aux=0.0, L=0.0, rotacion=0.0;

    t_aux = t_Trayectoria/T;
    theta = pi*t_aux;
    L = (lambda_Transferencia/2)*cos(theta);

    salida.x = PInicio.x + L;
    salida.y = PInicio.y;
    salida.z = PInicio.z + dh*sin(theta);
//    salida.x = PInicio.x - L*sin(rotacion);
//    salida.y = PInicio.y + L*cos(rotacion);
//    salida.z = PInicio.z + dh*sin(theta);

    return(salida);
}

punto3d TransformacionHomogenea(punto3d Punto_in, punto3d L_traslacion, float ang_rotacion){

    punto3d Punto_out;

    Punto_out.x = L_traslacion.x + Punto_in.x*cos(ang_rotacion) - Punto_in.y*sin(ang_rotacion);
    Punto_out.y = L_traslacion.y + Punto_in.x*sin(ang_rotacion) + Punto_in.y*cos(ang_rotacion);
    Punto_out.z = L_traslacion.z + Punto_in.z;

    return(Punto_out);
}

punto3d TransformacionHomogenea_Inversa(punto3d Punto_in, punto3d L_traslacion, float ang_rotacion){

    punto3d Punto_out;

    Punto_out.x = (-L_traslacion.x*cos(ang_rotacion)-L_traslacion.y*sin(ang_rotacion)) + Punto_in.x*cos(ang_rotacion) + Punto_in.y*sin(ang_rotacion);
    Punto_out.y = (L_traslacion.x*sin(ang_rotacion)-L_traslacion.y*cos(ang_rotacion)) - Punto_in.x*sin(ang_rotacion) + Punto_in.y*cos(ang_rotacion);
    Punto_out.z = (-L_traslacion.z) + Punto_in.z;

    return(Punto_out);
}
