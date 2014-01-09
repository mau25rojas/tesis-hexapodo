#include "ros/ros.h"
#include "std_msgs/String.h"
#include "math.h"
//Librerias propias usadas
#include "constantes.hpp"
#include "camina/v_repConst.h"
// Used data structures:
#include "camina/DatosTrayectoriaPata.h"
#include "camina/PlanificadorParametros.h"
#include "camina/SenalesCambios.h"
// Used API services:
#include "vrep_common/VrepInfo.h"
//Clientes y Servicios
ros::ServiceClient client_Planificador;
camina::PlanificadorParametros srv_Planificador;

// variables Globales
bool simulationRunning=true;
bool sensorTrigger=false;
float simulationTime=0.0f;
camina::DatosTrayectoriaPata datosTrayectoriaPata;
int Npata_arg=0,Tripode=0;
bool modificacion = false;
float divisionTrayectoriaPata=0.0, T=0.0, divisionTiempo=0.0, desfasaje_t=0.0, beta=0.0,t_aux=0.0;
float modificacion_T = 0.0, modificacion_lambda =0.0;
bool InicioTransf=true, FinTransf=false;
float delta_t=0.0;
FILE *fp1;
ros::Publisher chatter_pub1;
//camina::SenalesCambios senales;

// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}

void relojCallback(camina::SenalesCambios msgSenal)
{
    if (!msgSenal.Stop){
        t_aux=delta_t+desfasaje_t;
        t_aux=fmod(t_aux,T);

        if (t_aux>=(beta*T) && InicioTransf){
//            ROS_INFO("Nodo1::T[%d] Periodo=%.3f, (taux)=%.3f>=%.3f=(beta*T)",Tripode,T,t_aux,beta*T);
            InicioTransf=false;
            FinTransf=true;
//            ROS_INFO("Nodo1::T[%d] tiempo: %.3f, llamo planificacion",Tripode,t_aux);
            srv_Planificador.request.Tripode = Tripode;
            srv_Planificador.request.T = T;
            if (client_Planificador.call(srv_Planificador)){
                datosTrayectoriaPata.lambda_Transferencia[0] = srv_Planificador.response.modificacion_lambda;
                T = datosTrayectoriaPata.T[0] = srv_Planificador.response.modificacion_T;
                ROS_INFO("Nodo1::T[%d]: t_sim=%.3f, t_aux=%.3f, lambda_c=%.3f,t_c=%.3f",Tripode,simulationTime,t_aux,datosTrayectoriaPata.lambda_Transferencia[0],datosTrayectoriaPata.T[0]);
                divisionTrayectoriaPata = T/divisionTiempo;
            } else {
                ROS_ERROR("Nodo1::T[%d] servicio de Planificacion no funciona",Tripode);
                ROS_ERROR("result=%d", srv_Planificador.response.result);
            //        return;
            }
//            delta_t = delta_t + T/divisionTrayectoriaPata;
        }

        if (fabs(delta_t-T)<=(T/divisionTrayectoriaPata)) {
//            ROS_INFO("reinicio trayectoria[%d]",Tripode);
            delta_t = 0.0;
//            t_aux = 0.0;
//            InicioTransf=true;
        }
//        if (fabs(t_aux-T)>=2*(T/divisionTrayectoriaPata)) {
        if (t_aux>=(T*(1-1/divisionTrayectoriaPata)) && FinTransf) {
//            ROS_INFO("Nodo1::T[%d] reinicio trayectoria",Tripode);
            FinTransf=false;
            InicioTransf=true;
        }

        datosTrayectoriaPata.t_Trayectoria = t_aux;
        //-- Esta linea es muy importante
        chatter_pub1.publish(datosTrayectoriaPata);
        delta_t = delta_t + T/divisionTrayectoriaPata;
    }
}

int main(int argc, char **argv)
{
  float lambda_Apoyo=0.0, lambda_Transferencia=0.0, alfa=0.0, f=0.0;
  int i=0, Narg=0;

  Narg=9;
      // (when V-REP launches this executable, V-REP will also provide the argument list)
	//numero de argumentos que mande (se excluye el fantasma que el manda solo)
	if (argc>=Narg)
	{
        Npata_arg=atoi(argv[1]);
        Tripode=atoi(argv[2]);
        T=atof(argv[3]); // Periodo de trayectoria [seg]
		divisionTrayectoriaPata=atof(argv[4]);  //N puntos
		beta=atof(argv[5]);
		lambda_Apoyo=atof(argv[6]);
		lambda_Transferencia=atof(argv[7]);
		alfa=atof(argv[8])*pi/180;
        desfasaje_t=atof(argv[9]);
    } else {
		ROS_ERROR("Nodo1: Indique argumentos!\n");
		return 0;
	}

//    for(i=0;i<Npatas;i++) {
        datosTrayectoriaPata.T.push_back(0);
        datosTrayectoriaPata.lambda_Apoyo.push_back(0);
        datosTrayectoriaPata.lambda_Transferencia.push_back(0);
        datosTrayectoriaPata.desfasaje_t.push_back(0);
//    }

	/*Inicio nodo de ROS*/
    std::string nodeName("Nodo1_datosTrayectoriaPata");
	std::string Id(boost::lexical_cast<std::string>(Tripode));
	nodeName+=Id;
	ros::init(argc,argv,nodeName.c_str());
    ros::NodeHandle node;

//-- Topicos susbcritos y publicados
    ros::Subscriber subInfo1=node.subscribe("/vrep/info",100,infoCallback);
    ros::Subscriber subInfo2=node.subscribe("Reloj",100,relojCallback);
//-- Manda topico especifico para cada pata
    std::string topicName("datosTrayectoria_T");
    topicName+=Id;
    chatter_pub1=node.advertise<camina::DatosTrayectoriaPata>(topicName.c_str(), 100);
//-- Clientes y Servicios
    client_Planificador = node.serviceClient<camina::PlanificadorParametros>("PlanificadorPisada");
//-- Log de datos
//    std::string fileName("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina/datos/SalidaDatos");
//    std::string texto(".txt");
//    fileName+=Id;
//    fileName+=texto;
//    fp1 = fopen(fileName.c_str(),"w+");

//-- Datos de envio
    datosTrayectoriaPata.T[0]=T;
    datosTrayectoriaPata.divisionTrayectoriaPata=divisionTrayectoriaPata;
    datosTrayectoriaPata.lambda_Apoyo[0]=lambda_Apoyo;
    datosTrayectoriaPata.lambda_Transferencia[0]=lambda_Transferencia;
    datosTrayectoriaPata.alfa=alfa;
    datosTrayectoriaPata.desfasaje_t[0]=desfasaje_t;

//-- Prepara variables para calculos de trayectoria de PATA
    //delta_t = 1/divisionTrayectoriaPata;
    divisionTiempo = T/divisionTrayectoriaPata;
    f=1/divisionTiempo;
    /* La velocidad de envío de los datos se encarga de darme el tiempo total de trayectoria deseado */
    /* Velocidad de transmision */
    ros::Rate loop_rate(f);  //Frecuencia [Hz]
//-- Delay inicial para esperar inicio de todos los nodos
    for(i=0;i<10;i++) loop_rate.sleep();

    modificacion_T = T;
    modificacion_lambda = lambda_Transferencia;

    while (ros::ok() && simulationRunning){
        ros::spinOnce();
//        loop_rate.sleep();
    }
        ROS_INFO("Adios1!");
//        fclose(fp1);
        ros::shutdown();
        return 0;
    }
