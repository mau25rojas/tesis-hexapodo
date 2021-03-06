#include "ros/ros.h"
#include "math.h"
#include "string.h"
//Librerias propias usadas
#include "camina12/v_repConst.h"
#include "camina12/constantes.hpp"
#include "camina12/vector3d.hpp"
// Used data structures:
#include "camina12/DatosTrayectoriaPata.h"
#include "camina12/AngulosMotor.h"
//#include "camina12/CinversaParametros.h"
// Used API services:
#include "vrep_common/VrepInfo.h"

//-- Global variables (modified by topic subscribers):
bool simulationRunning=true;
bool sensorTrigger=false;
float simulationTime=0.0f;
bool Inicio=true;
//-- Calculo de trayectoria
int tripode[Npatas];
float velocidadApoyo=0.0, dh=0.0,phi[Npatas];
punto3d Offset, P0[Npatas], FinApoyo[Npatas], FinTranfer[Npatas];

camina12::AngulosMotor qMotor;
ros::Publisher chatter_pub;
FILE *fp1,*fp2,*fp3,*fp4,*fp5,*fp6;
//-- Funciones
punto3d Trayectoria_FaseApoyo(float t_Trayectoria,punto3d PInicio);
punto3d Trayectoria_FaseTrans_Eliptica(float t_Trayectoria,punto3d PInicio, punto3d PFin);
void CinematicaInversa(float *Qs,punto3d P_in);

// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}

/* Callback que escucha el topico DatosDeTrayectoria calcula la trayectoria
   deseaday cinematica inversa para de motores y los publica
*/
void datosCallback(const camina12::DatosTrayectoriaPata msg_datoTrayectoria)
{
    int correccion_ID, cambioEstado, Estado;
    float t_Trayectoria,alfa=0.0, T=0.0;
    float correccion_x,correccion_y;
    punto3d P1, PInicio, PFin, InicioApoyo;

    for(int k=0;k<Npatas;k++){
        T = msg_datoTrayectoria.T[tripode[k]-1];
        t_Trayectoria = msg_datoTrayectoria.t_Trayectoria[tripode[k]-1];
        alfa = msg_datoTrayectoria.alfa;
        Estado = msg_datoTrayectoria.vector_estados[tripode[k]-1];
        cambioEstado = msg_datoTrayectoria.cambio_estado[tripode[k]-1];
        correccion_x = msg_datoTrayectoria.correccion_x[k];
        correccion_y = msg_datoTrayectoria.correccion_y[k];
        correccion_ID = msg_datoTrayectoria.correccion_ID[k];

        InicioApoyo.x = (Offset.y-FinEspacioTrabajo_y)-lambda_maximo+correccion_y;
        InicioApoyo.y = 0.0;
        if(correccion_ID==Correccion_menosX){
            InicioApoyo.y = -correccion_x;
        } else if (correccion_ID==Correccion_masX){
            InicioApoyo.y = correccion_x;
        }

        if(cambioEstado==1){
            FinApoyo[k] = P0[k];
            FinTranfer[k] = InicioApoyo;
        }

        if(Inicio){
            if(k==Npatas-1) Inicio=false;
            InicioApoyo.x=(Offset.y-FinEspacioTrabajo_y)-lambda_maximo;
            InicioApoyo.y=0.0;
            FinApoyo[k].x=Offset.y-FinEspacioTrabajo_y;
            FinApoyo[k].y=0.0;
            FinTranfer[k] = InicioApoyo;
        }
        //-----Parametrizacion de trayectoria eliptica en Sistema de Robot

        switch (Estado){
            case Apoyo:
                PInicio=InicioApoyo;
                P0[k] = Trayectoria_FaseApoyo(t_Trayectoria,PInicio);
            break;

            case Transferencia:
                // Elipsis
                PInicio=FinApoyo[k];
                PFin=FinTranfer[k];
                P0[k] = Trayectoria_FaseTrans_Eliptica(t_Trayectoria/T,PInicio,PFin);
            break;

            default:
                ROS_ERROR("Nodo2: Error en estados");
            break;
        }
        //-----Transformacion de trayectoria a Sistema de Pata
        P1 = TransformacionHomogenea(P0[k],Offset,phi[k]+alfa);
        //-----Cinematica Inversa
        float q[Neslabones];
        CinematicaInversa(q,P1);
        qMotor.q1[k] = q[0];
        qMotor.q2[k] = q[1];
        qMotor.q3[k] = q[2];

        switch (k){
            case 0:
                fprintf(fp1,"%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n",qMotor.q1[k],qMotor.q2[k],qMotor.q3[k],t_Trayectoria,P1.x,P1.y,P1.z);
            break;
            case 1:
                fprintf(fp2,"%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n",qMotor.q1[k],qMotor.q2[k],qMotor.q3[k],t_Trayectoria,P1.x,P1.y,P1.z);
            break;
            case 2:
                fprintf(fp3,"%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n",qMotor.q1[k],qMotor.q2[k],qMotor.q3[k],t_Trayectoria,P1.x,P1.y,P1.z);
            break;
            case 3:
                fprintf(fp4,"%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n",qMotor.q1[k],qMotor.q2[k],qMotor.q3[k],t_Trayectoria,P1.x,P1.y,P1.z);
            break;
            case 4:
                fprintf(fp5,"%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n",qMotor.q1[k],qMotor.q2[k],qMotor.q3[k],t_Trayectoria,P1.x,P1.y,P1.z);
            break;
            case 5:
                fprintf(fp6,"%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n",qMotor.q1[k],qMotor.q2[k],qMotor.q3[k],t_Trayectoria,P1.x,P1.y,P1.z);
            break;
        }
    } //fin for Npatas
    //---Publica angulos motores----
	chatter_pub.publish(qMotor);
}

int main(int argc, char **argv){

	if (argc>=17)
	{	dh=atof(argv[1]);
        Offset.x=atof(argv[2]);
        Offset.y=atof(argv[3]);
        Offset.z=atof(argv[4]);
        velocidadApoyo=atof(argv[5]);
        for(int k=0;k<Npatas;k++) phi[k]=atof(argv[6+k])*pi/180;
        for(int k=0;k<Npatas;k++) tripode[k] = atoi(argv[6+Npatas+k]);
	}
	else
	{
		ROS_ERROR("Nodo 2::Indique argumentos!\n");
		sleep(5000);
		return 0;
	}

    /*Inicio nodo de ROS*/
    std::string nodeName("Nodo2_Parametrizacion");
	ros::init(argc,argv,nodeName.c_str());
    ros::NodeHandle node;
    ROS_INFO("Nodo2_Parametrizacion just started\n");

//-- Topicos susbcritos y publicados
    chatter_pub = node.advertise<camina12::AngulosMotor>("DatosDeMotores", 100);
    ros::Subscriber subInfo = node.subscribe("/vrep/info",1,infoCallback);
//-- Recibe topico
    ros::Subscriber sub = node.subscribe("datosTrayectoria", 100, datosCallback);

    fp1 = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina12/datos/QXEnviada_Pata1.txt","w+");
    fp2 = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina12/datos/QXEnviada_Pata2.txt","w+");
    fp3 = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina12/datos/QXEnviada_Pata3.txt","w+");
    fp4 = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina12/datos/QXEnviada_Pata4.txt","w+");
    fp5 = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina12/datos/QXEnviada_Pata5.txt","w+");
    fp6 = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina12/datos/QXEnviada_Pata6.txt","w+");

    //-- Datos de envio
    for(int k=0;k<Npatas;k++){
        qMotor.q1.push_back(0);
        qMotor.q2.push_back(0);
        qMotor.q3.push_back(0);
    }

    while (ros::ok() && simulationRunning)
    {
	  ros::spinOnce();
    }
    fclose(fp1);
	fclose(fp2);
	fclose(fp3);
	fclose(fp4);
	fclose(fp5);
	fclose(fp6);
    //ROS_INFO("Adios2!");
    ros::shutdown();
    return 0;
}

//---Caso parte 1 trayectoria----
//-- Periodo A-B
punto3d Trayectoria_FaseApoyo(float t_Trayectoria,punto3d PInicio){

    punto3d salida;

    salida.x = PInicio.x + velocidadApoyo*t_Trayectoria;
    salida.y = PInicio.y;
    salida.z = PInicio.z;

    return(salida);
}

//---Caso parte 2 trayectoria----
//-- Elipsis
punto3d Trayectoria_FaseTrans_Eliptica(float t_Trayectoria,punto3d PInicio, punto3d PFin){

    punto3d salida, Po;
    float teta, dL;
    float L, Lx, Ly, gamma;

    Lx = PFin.x - PInicio.x;
    Ly = PFin.y - PInicio.y;
    L = sqrt(Lx*Lx + Ly*Ly);
    gamma = atan(Ly/Lx);
    Po.x = PInicio.x + Lx/2;
    Po.y = PInicio.y + Ly/2;
    Po.z = 0.0;

//    t_aux = t_Trayectoria/T_in;
    teta = pi*t_Trayectoria;
    dL = (L/2)*cos(teta);
//    if(Npata_arg==1) ROS_INFO("Pi.x=%.4f\t Pi.y=%.4f\t Pf.x=%.4f\t Pf.y=%.4f\t",PInicio.x,PInicio.y,PFin.x,PFin.y);

    salida.x = Po.x + dL*cos(gamma);
    salida.y = Po.y + dL*sin(gamma);
    salida.z = Po.z + dh*sin(teta);

    return(salida);
}

//---Cinematica inversa---
void CinematicaInversa(float *Qs,punto3d P_in){
    float L23=0.0, L23_aux=0.0, beta=0.0, teta=0.0, gamma1=0.0, arg=0;
    //------calculo q1------
    //p[0] = atan2(px,py);
    Qs[0] = atan2(P_in.y,P_in.x)-pi/2;
    //----------------------

    L23_aux = sqrt(P_in.x*P_in.x + P_in.y*P_in.y) - L1;
    L23 = sqrt(L23_aux*L23_aux + P_in.z*P_in.z);

    //------calculo q3------
	arg = (L2*L2 + L3*L3 - L23*L23)/(2*L2*L3);
	if (fabs(arg)>1) {
        ROS_ERROR("ERROR Cinversa_server acos q3");
        return;
	}
	beta = acos(arg);
    //El ajuste de pi se hace para coincidir con eje de D-H
    //p[2] = pi - beta;
    Qs[2] = beta-pi/2;
    //----------------------

    //------calculo q2------
	arg = (L3/L23)*sin(beta);
	if (fabs(arg)>1) {
        ROS_ERROR("ERROR Cinversa_server asin q2");
        return;
	}
	teta = atan(-P_in.z/L23_aux);
	gamma1 = asin(arg);
	//El ajuste de pi/2 se hace para coincidir con eje de D-H
    //p[1] = pi/2 - (teta-gamma);
    Qs[1] = teta-gamma1;
}
