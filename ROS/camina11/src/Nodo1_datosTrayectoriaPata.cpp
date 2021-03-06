#include <ros/ros.h>
#include <std_msgs/String.h>
#include <math.h>
#include <time.h>
#include "boost/date_time/posix_time/posix_time.hpp"
//Librerias propias usadas
#include "camina11/v_repConst.h"
#include "camina11/constantes.hpp"
#include "camina11/vector3d.hpp"
// Used data structures:
#include "camina11/DatosTrayectoriaPata.h"
#include "camina11/PlanificadorParametros.h"
#include "camina11/UbicacionRobot.h"
// Used API services:
#include "vrep_common/VrepInfo.h"
#define PataPrint 4-1
//Clientes y Servicios
ros::ServiceClient client_Planificador;
camina11::PlanificadorParametros srv_Planificador;

// variables Globales
bool simulationRunning=true;
bool sensorTrigger=false, Inicio=true;
bool InicioApoyo[Npatas]={false,false,false,false,false,false}, FinApoyo[Npatas]={false,false,false,false,false,false};
bool InicioTransf[Npatas]={false,false,false,false,false,false}, FinTransf[Npatas]={false,false,false,false,false,false},mediaTransf[Npatas]={true,true,true,true,true,true};
bool velPata1_1=false,velPata1_2=false;
float simulationTime=0.0f;
float divisionTrayectoriaPata[Npatas],t_Trayectoria[Npatas], divisionTiempo=0.0, desfasaje_t[Npatas], beta=0.0, phi[Npatas],alfa=0.0, dh=0.0, velApoyo=0.0;
float T[Npatas], T_contador[Npatas], T_apoyo[Npatas],T_transf[Npatas], contadores[Npatas],delta_t[Npatas], modificacion_T_apoyo = 0.0, modificacion_lambda =0.0;
float xCuerpo_1=0.0, xCuerpo_2=0.0, yCuerpo_1=0.0, yCuerpo_2=0.0, mod_velocidadCuerpo=0.0;
int pataApoyo[Npatas],divisionTrayectoriaPata_ini;
int cuenta=0, PasosIni=0;
FILE *fp1,*fp2;
punto3d coordenadaCuerpo,velocidadCuerpo,posicionActualPataSistemaPata[Npatas],posCuerpo_1,posCuerpo_2;
punto3d Offset;
boost::posix_time::ptime timer_1,timer_2;
camina11::DatosTrayectoriaPata datosTrayectoriaPata;
ros::Publisher chatter_pub1,chatter_pub2;

// Funciones
void Inicializacion();
bool CambioDeEstado_Apoyo(int nPata);
bool CambioDeEstado_Transf(int nPata);
int VerificacionEstadoPata(int nPata, int estadoActual);
bool LlegadaFinEDT(int nPata);
void ParametrosVelocidad();
float VelocidadCuerpo(boost::posix_time::ptime t1, boost::posix_time::ptime t2, punto3d Pos1, punto3d Pos2);

// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}

void ubicacionRobCallback(camina11::UbicacionRobot msgUbicacionRobot)
{
     coordenadaCuerpo.x = msgUbicacionRobot.coordenadaCuerpo_x;
     coordenadaCuerpo.y = msgUbicacionRobot.coordenadaCuerpo_y;
     for(int k=0; k<Npatas;k++) {
        pataApoyo[k] = msgUbicacionRobot.pataApoyo[k];
        posicionActualPataSistemaPata[k].x = msgUbicacionRobot.coordenadaPataSistemaPata_x[k];
        posicionActualPataSistemaPata[k].y = msgUbicacionRobot.coordenadaPataSistemaPata_y[k];
        posicionActualPataSistemaPata[k].z = msgUbicacionRobot.coordenadaPataSistemaPata_z[k];
    }
}

int main(int argc, char **argv)
{
    float lambda=0.0, f=0.0, vector_estados[Npatas], T_ini;
    int Narg=0;
    boost::posix_time::ptime t_inicio, t_fin;
    float tiempo_ahora=0.0;
    boost::posix_time::time_duration diff_t;

    Narg=23;
	if (argc>=Narg)
	{
        PasosIni=atoi(argv[1]);
        T_ini=atof(argv[2]); // Periodo de trayectoria [seg]
		divisionTrayectoriaPata_ini=atof(argv[3]);  //N puntos
		beta=atof(argv[4]);
		lambda=atof(argv[5]);
		velApoyo=atof(argv[6]);
		alfa=atof(argv[7])*pi/180;
		dh=atof(argv[8]);
        Offset.x=atof(argv[9]);
        Offset.y=atof(argv[10]);
        Offset.z=atof(argv[11]);
        for(int k=0;k<Npatas;k++) desfasaje_t[k]=atof(argv[12+k]);
        for(int k=0;k<Npatas;k++) phi[k]=atof(argv[12+Npatas+k])*pi/180;
    } else {
		ROS_ERROR("Nodo1: Indique argumentos!\n");
		return 0;
	}

	/*Inicio nodo de ROS*/
    std::string nodeName("Nodo1_datosTrayectoriaPata");
	ros::init(argc,argv,nodeName.c_str());
    ros::NodeHandle node;
    ROS_INFO("Nodo1_datosTrayectoriaPata just started\n");

//-- Topicos susbcritos y publicados
    ros::Subscriber subInfo1=node.subscribe("/vrep/info",100,infoCallback);
    ros::Subscriber subInfo3=node.subscribe("UbicacionRobot",100,ubicacionRobCallback);
//-- Manda topico especifico para cada pata
    chatter_pub1=node.advertise<camina11::DatosTrayectoriaPata>("datosTrayectoria", 100);
//-- Clientes y Servicios
    client_Planificador = node.serviceClient<camina11::PlanificadorParametros>("PlanificadorPisada");
//-- Log de datos
    fp1 = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina11/datos/SalidaDatos.txt","w+");
    fp2 = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina11/datos/SalidaTiempo.txt","w+");

//-- Inicializo variables
    for(int k=0;k<Npatas;k++) {
        T[k] = T_ini;
        T_apoyo[k] = beta*T[k];
        T_transf[k] = T[k]-T_apoyo[k];
        if(k==0) ROS_INFO("T_apoyo=%.3f, T_trans=%.3f",T_apoyo[k],T_transf[k]);
        contadores[k] = desfasaje_t[k]*T_ini;
        divisionTrayectoriaPata[k] = divisionTrayectoriaPata_ini;
        if(desfasaje_t[k]>beta){
            vector_estados[k] = 1;
            T_contador[k] = T_transf[k];
        } else {
            vector_estados[k] = 0;
            T_contador[k] = T_apoyo[k];
        }
    }
//-- Datos de envio
    datosTrayectoriaPata.alfa=alfa;
    for(int k=0;k<Npatas;k++) {
        datosTrayectoriaPata.T.push_back(T_transf[k]);
        datosTrayectoriaPata.t_Trayectoria.push_back(0);
        datosTrayectoriaPata.lambda.push_back(lambda);
        datosTrayectoriaPata.desfasaje_t.push_back(desfasaje_t[k]);
        datosTrayectoriaPata.vector_estados.push_back(vector_estados[k]);
        datosTrayectoriaPata.cambio_estado.push_back(0);
        datosTrayectoriaPata.correccion_x.push_back(0);
        datosTrayectoriaPata.correccion_y.push_back(0);
        datosTrayectoriaPata.correccion_ID.push_back(-1);
    }

//-- Prepara variables para calculos de trayectoria de PATA
    //contadores = 1/divisionTrayectoriaPata;
    divisionTiempo = T_ini/divisionTrayectoriaPata_ini;
    f=1/divisionTiempo;
    /* La velocidad de envío de los datos se encarga de darme el tiempo total de trayectoria deseado */
    /* Velocidad de transmision */
    ros::Rate loop_rate(f);  //Frecuencia [Hz]

    modificacion_T_apoyo = T_ini;
    modificacion_lambda = lambda;
    float correccion_x = 0.0;
    t_inicio = boost::posix_time::microsec_clock::local_time();

    while (ros::ok() && simulationRunning){
        ros::spinOnce();
        loop_rate.sleep();

        if (Inicio){
            Inicializacion();
        } else {
            chatter_pub1.publish(datosTrayectoriaPata);

            for(int k=0;k<Npatas;k++){
//                if(k==PataPrint) ROS_INFO("pata[%d] contador=%.3f",k+1,contadores[k]);
                datosTrayectoriaPata.cambio_estado[k]=0;
                delta_t[k]=T[k]/divisionTrayectoriaPata[k];
//                if(k==PataPrint) ROS_INFO("cuenta=%.3f",contadores[k]);
                ///-- Verificacion por posicion actual de pata
                //-- ..Seleccion de estado proximo de la pata
                //-- ..si hay cambio de estado se activa la bandera correspondiente
                datosTrayectoriaPata.vector_estados[k]=VerificacionEstadoPata(k,datosTrayectoriaPata.vector_estados[k]);
                ParametrosVelocidad();
//                if(k==PataPrint) ROS_INFO("pata[%d] estado=%d",k+1,datosTrayectoriaPata.vector_estados[k]);

                if(datosTrayectoriaPata.cambio_estado[k]==1 and datosTrayectoriaPata.vector_estados[k]==Apoyo) contadores[k]=0.0;
                if(datosTrayectoriaPata.cambio_estado[k]==1 and datosTrayectoriaPata.vector_estados[k]==Transferencia){

                    T_apoyo[k]=contadores[k];
                    T_transf[k]=T[k]-T_apoyo[k];

                    mod_velocidadCuerpo = VelocidadCuerpo(timer_1,timer_2,posCuerpo_1,posCuerpo_2);
                    fprintf(fp1,"%.3f\t%.3f\t%.3f\t%.3f\n",simulationTime,T_apoyo[k],T_transf[k],mod_velocidadCuerpo);
//                    ROS_INFO("Nodo1::mod_velocidad=%.4f",mod_velocidadCuerpo);
                    ROS_INFO("Nodo1::pata[%d] t_sim=%.3f,T_a=%.3f,T_t=%.3f,vel=%.3f",k+1,simulationTime,T_apoyo[k],T_transf[k],mod_velocidadCuerpo);
                    if(datosTrayectoriaPata.correccion_ID[k]==Correccion_menosX){
                        correccion_x = -datosTrayectoriaPata.correccion_x[k];
                    } else if (datosTrayectoriaPata.correccion_ID[k]==Correccion_masX){
                        correccion_x = datosTrayectoriaPata.correccion_x[k];
                    } else {
                        correccion_x=0.0;
                    }
                    srv_Planificador.request.nPata = k;
                    srv_Planificador.request.T = T_transf[k];
                    srv_Planificador.request.mod_velApoyo = mod_velocidadCuerpo;
                    srv_Planificador.request.correccion_x = correccion_x;
                    if (client_Planificador.call(srv_Planificador) and srv_Planificador.response.resultado!=-1){
                        modificacion_lambda = srv_Planificador.response.modificacion_lambda;
                        datosTrayectoriaPata.correccion_ID[k] = srv_Planificador.response.correccion_ID;
                        datosTrayectoriaPata.correccion_x[k] = srv_Planificador.response.correccion_x;
                        datosTrayectoriaPata.correccion_y[k] = srv_Planificador.response.correccion_y;
//                        ROS_INFO("Nodo1::t_sim=%.3f, lambda_c=%.3f",simulationTime,modificacion_lambda);
                    } else {
                        ROS_ERROR("Nodo1::servicio de Planificacion no funciona");
                        ROS_ERROR("Parada de emergencia: Adios1!");
//                        fclose(fp1);
                        return 0;
                        ros::shutdown();
                    }
                }// fin de InicioTransferencia

                if(fabs(contadores[k]-T[k])<delta_t[k]){
                    contadores[k] = T[k];
                } else {
                    contadores[k] = contadores[k] + delta_t[k];
                }
                if (datosTrayectoriaPata.vector_estados[k]==Transferencia) {
                    t_Trayectoria[k] = (contadores[k]-T_apoyo[k])/T_transf[k];
                } else {
                    t_Trayectoria[k] = contadores[k];
                }
//                fprintf(fp1,"%.3f\t%d\t",contadores[k],datosTrayectoriaPata.vector_estados[k]);
                t_fin = boost::posix_time::microsec_clock::local_time();
                diff_t = t_inicio - t_fin;
                tiempo_ahora = (float) fabs(diff_t.total_milliseconds())/1000;
                fprintf(fp2,"%.3f\n",tiempo_ahora);

                datosTrayectoriaPata.t_Trayectoria[k]=t_Trayectoria[k];
            }// fin del for
//            fprintf(fp1,"\n");
        }//-- Checkea por inicio
    }
        ROS_INFO("Adios1!");
        fclose(fp1);fclose(fp2);
        ros::shutdown();
        return 0;
}

/* Funciones */
void Inicializacion(){

    cuenta++;
    if (cuenta==PasosIni*divisionTrayectoriaPata_ini){
        Inicio=false;
    } else {

        for(int k=0;k<Npatas;k++){
            datosTrayectoriaPata.cambio_estado[k]=0;
            delta_t[k]=T[k]/divisionTrayectoriaPata[k];

            contadores[k] = contadores[k] + delta_t[k];

            if (contadores[k]>beta*T[k]+0.001) {
                t_Trayectoria[k] = (contadores[k]-T_apoyo[k])/T_transf[k];
                datosTrayectoriaPata.vector_estados[k]=Transferencia;
            } else {
                t_Trayectoria[k] = contadores[k];
                datosTrayectoriaPata.vector_estados[k]=Apoyo;
            }
            if (fabs(contadores[k]-T[k])<delta_t[k]){
                contadores[k] = 0.0;
            }

            CambioDeEstado_Apoyo(k);
            LlegadaFinEDT(k);
            ParametrosVelocidad();
        }

        for(int k=0;k<Npatas;k++) {
            datosTrayectoriaPata.t_Trayectoria[k]=t_Trayectoria[k];
        }
        chatter_pub1.publish(datosTrayectoriaPata);
    }
}

//-- Verificacion por posicion actual de pata recibida
int VerificacionEstadoPata(int nPata, int estadoActual){
    bool cambio_a_Apoyo=false,llegada_FEDT=false;
    int estadoSiguiente;
    estadoSiguiente=estadoActual;

    cambio_a_Apoyo = CambioDeEstado_Apoyo(nPata);
    if(cambio_a_Apoyo){
        cambio_a_Apoyo = false;
        estadoSiguiente = Apoyo;
        datosTrayectoriaPata.cambio_estado[nPata]=1;
    }
    llegada_FEDT = LlegadaFinEDT(nPata);
    if(llegada_FEDT){
        llegada_FEDT = false;
        estadoSiguiente = Transferencia;
        datosTrayectoriaPata.cambio_estado[nPata]=1;
    }

    return(estadoSiguiente);
}

bool CambioDeEstado_Apoyo(int nPata){
    bool cambio = false;
//--- Apoyo de Pata
    if (pataApoyo[nPata]==Apoyo and FinApoyo[nPata]) {
        InicioApoyo[nPata]=true;
        FinApoyo[nPata]=false;
    }
//    if(nPata==PataPrint) ROS_WARN("------Pata[%d]=%d",nPata+1,pataApoyo[nPata]);
//    if (pataApoyo[nPata]==Transferencia and (fabs(contadores[nPata]-(T[nPata]-T_transf[nPata]/2))<=delta_t[nPata])) {
    if (pataApoyo[nPata]==Transferencia and (fabs(contadores[nPata]-(T[nPata]-5*delta_t[nPata]))<=delta_t[nPata])) {
//    if (pataApoyo[nPata]==Transferencia and (fabs(contadores[nPata]-(T_apoyo[nPata]+5*delta_t[nPata]))<=delta_t[nPata])) {
        FinApoyo[nPata]=true;
//        if(nPata==PataPrint) ROS_WARN("****Pata[%d] finApoyo",nPata+1);
//        ROS_WARN("****Pata[%d] finApoyo",nPata+1);
    }
    if (InicioApoyo[nPata]){
        InicioApoyo[nPata]=false;
        cambio=true;
    //-- Para velocidad, pata1
        if(nPata==0) velPata1_1=true;
//            velPata1_1=true;
//        if(nPata==PataPrint) ROS_WARN("***Inicia Apoyo pata[%d]",nPata+1);
    }
    return cambio;
}


bool LlegadaFinEDT(int nPata){

    float paso_y = 0.0;
    bool cambio = false; punto3d P0, Fin_EDT;
    paso_y = velApoyo*divisionTiempo;
//    ROS_WARN("%.3f,%.3f",velApoyo,divisionTiempo);
    if(datosTrayectoriaPata.correccion_ID[nPata]==Correccion_menosX){
        P0.y = -datosTrayectoriaPata.correccion_x[nPata];
    } else if (datosTrayectoriaPata.correccion_ID[nPata]==Correccion_masX){
        P0.y = datosTrayectoriaPata.correccion_x[nPata];
    } else {
        P0.y = 0.0;
    }
    P0.x = Offset.y-FinEspacioTrabajo_y-paso_y;
    //-----Transformacion de trayectoria a Sistema de Pata
    Fin_EDT = TransformacionHomogenea(P0,Offset,phi[nPata]+alfa);

    if (fabs(posicionActualPataSistemaPata[nPata].y-Fin_EDT.y)<=0.002 and FinTransf[nPata]) {
        InicioTransf[nPata]=true;
        FinTransf[nPata]=false;
    }
//    if(nPata==n) ROS_WARN("Pata.z=%.4f",posicionActualPataSistemaPata[nPata].z);
//    if (pataApoyo[nPata]==Apoyo and (fabs(contadores[nPata]-T_apoyo[nPata]/2)<delta_t[nPata])) {
    if (pataApoyo[nPata]==Apoyo and contadores[nPata]==4*delta_t[nPata]) {
        FinTransf[nPata]=true;
//        if(nPata==PataPrint) ROS_WARN("------Pata[%d] preTransferencia",nPata+1);
    }
    if (InicioTransf[nPata]){
        InicioTransf[nPata]=false;
        cambio=true;
    //-- Para velocidad, pata1
        if(nPata==1) velPata1_2=true;
//         velPata1_2=true;
//        if(nPata==PataPrint) ROS_WARN("------Inicia Transferencia pata[%d]",nPata+1);
    }
    return cambio;
}

/* Toma de muestras de tiempo y posicion para calculo de velocidad
.. se toma de muestra de patas escogidas*/
void ParametrosVelocidad(){
//--- Apoyo de Pata 1
    if (velPata1_1){
        velPata1_1=false;
        posCuerpo_1.x = coordenadaCuerpo.x;
        posCuerpo_1.y = coordenadaCuerpo.y;
        timer_1 = boost::posix_time::microsec_clock::local_time();
    }
//--- Transferencia de Pata 1
    if (velPata1_2){
        velPata1_2=false;
        posCuerpo_2.x = coordenadaCuerpo.x;
        posCuerpo_2.y = coordenadaCuerpo.y;
        timer_2 = boost::posix_time::microsec_clock::local_time();
    }
}

/* Retorna el modulo de la velocidad del cuerpo, segun el tiempo y distancia del cuerpo
.. en apoyo y transferencia*/
float VelocidadCuerpo(boost::posix_time::ptime t1, boost::posix_time::ptime t2, punto3d Pos1, punto3d Pos2){
    float delta_x=0.0, delta_y=0.0, tiempo_ahora=0.0;
    boost::posix_time::time_duration dif_t;

    delta_x = fabs(Pos1.x-Pos2.x);
    delta_y = fabs(Pos1.y-Pos2.y);
    dif_t = t1 - t2;
    tiempo_ahora = (float) fabs(dif_t.total_milliseconds())/1000;
    if(tiempo_ahora<0.00001){
    //-- caso que no se mida bien el tiempo
        return(0.0);
    }
    velocidadCuerpo.x = delta_x/tiempo_ahora;
    velocidadCuerpo.y = delta_y/tiempo_ahora;
    return (sqrt(velocidadCuerpo.x*velocidadCuerpo.x + velocidadCuerpo.y*velocidadCuerpo.y));
}
