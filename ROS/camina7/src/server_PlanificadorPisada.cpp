#include "ros/ros.h"
#include "std_msgs/String.h"
#include "math.h"
#include <algorithm>    // std::sort
//Librerias propias usadas
#include "constantes.hpp"
#include "camina7/v_repConst.h"
// Used data structures:
#include "camina7/InfoMapa.h"
#include "camina7/CinversaParametros.h"
#include "camina7/UbicacionRobot.h"
#include "camina7/PlanificadorParametros.h"
#include "camina7/SenalesCambios.h"
// Used API services:
#include "vrep_common/VrepInfo.h"
// Definiciones
#define MAX_CICLOS 5
//Clientes y Servicios
ros::ServiceClient client_Cinversa1;
camina7::CinversaParametros srv_Cinversa1;

//-- Variables Globales
bool simulationRunning=true;
bool sensorTrigger=false;
float simulationTime=0.0f;
//-- Log de planificador
FILE *fp1,*fp2;
//-- Entrada
int Tripode=0, tripode[Npatas], Tripode1[Npatas/2], Tripode2[Npatas/2], cuentaPasos=0, cuentaErrores[Npatas]={0,0,0,0,0,0};
float velocidadApoyo=0.0, beta=0.0, phi[Npatas], alfa=0.0;
//-- Variables de mapa
camina7::InfoMapa infoMapa;
std::vector<int> coordenadaObstaculo_i(1000,0), coordenadaObstaculo_j(1000,0);
bool matrizMapa[100][100];
int nCeldas_i=0, nCeldas_j=0;
float LongitudCeldaY=0, LongitudCeldaX=0;
//-- Variables de ubicacion robot
double tiempo_ahora=0.0, tiempo_anterior=0.0;
float ajuste_Vel=vel_esperada/vel_teorica;
float velocidadCuerpo_y=0.0, delta_x=0.0, delta_y=0.0, x_anterior=0.0, y_anterior=0.0, x_actual=0.0, y_actual=0.0;
float posicionActualPata_y[Npatas], posicionActualPata_x[Npatas], posicionActualPata_z[Npatas];
float posicionActualPataSistemaPata_y[Npatas],posicionActualPataSistemaPata_x[Npatas],posicionActualPataSistemaPata_z[Npatas];
float teta_CuerpoRobot=0.0;
//-- Envio de señal de stop
ros::Publisher chatter_pub1;
camina7::SenalesCambios senales;
//-- Generales
//int k=0;
ros::Publisher chatter_pub2;

//-- Funciones
void transformacion_yxTOij(int *ptr_ij, float y, float x);
void FilePrint_matrizMapa();
void Limpiar_matrizMapa();
int Construye_matrizMapa(std::string fileName);
void print_matrizMapa();

//-- Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}


void ubicacionRobCallback(camina7::UbicacionRobot msgUbicacionRobot)
{
    float vel_aux=0.0;

    teta_CuerpoRobot = msgUbicacionRobot.orientacionCuerpo_yaw;

//    vel_aux = msgUbicacionRobot.velocidadCuerpo_y;
//    if(vel_aux>velocidad_Apoyo){
////-- la velocidad no puede ser mayor a la esperada
//        velocidadCuerpo_y = velocidad_Apoyo;
//    } else {
//        velocidadCuerpo_y = vel_aux;
//    }
////    velocidadCuerpo_y= ajuste_Vel*velocidadCuerpo_y;
//    velocidadCuerpo_y= velocidadCuerpo_y;

    for(int k=0; k<Npatas;k++) {
        posicionActualPata_x[k] = msgUbicacionRobot.coordenadaPata_x[k];
        posicionActualPata_y[k] = msgUbicacionRobot.coordenadaPata_y[k];
        posicionActualPata_z[k] = msgUbicacionRobot.coordenadaPata_z[k];
        posicionActualPataSistemaPata_x[k] = msgUbicacionRobot.coordenadaPataSistemaPata_x[k];
        posicionActualPataSistemaPata_y[k] = msgUbicacionRobot.coordenadaPataSistemaPata_y[k];
        posicionActualPataSistemaPata_z[k] = msgUbicacionRobot.coordenadaPataSistemaPata_z[k];
    }
}

bool PlanificadorPisada(camina7::PlanificadorParametros::Request  &req,
                        camina7::PlanificadorParametros::Response &res)
{
//    ROS_INFO("Llamado a servicio planificador");
//    fprintf(fp2,"\ntiempo de simulacion: %.3f\t",simulationTime);

//-- Datos para envio de mensajes
    res.modificacion_T = 0.0;
    res.modificacion_lambda = 0.0;
    res.result = 0;
//-- Variables locales
    int Tripode_Transferencia[Npatas/2];
    int PisadaProxima_i=0, PisadaProxima_j=0;
    bool PisadaInvalida[Npatas/2], TodasPisadasOk, cinversaOK;
    float PisadaProxima_x=0.0, PisadaProxima_y=0.0, PisadaProxima_z=0.0;
    float delta_x_S0=0.0, delta_y_S0=0.0, delta_x=0.0, delta_y=0.0;
    float modificacion_lambda[Npatas/2];
    float T_actual=0.0,lambda_Apoyo_actual=0.0;
    int ij[2]={0,0}, *p_ij;     //Apuntadores a arreglos de coordenadas e indices
    p_ij = ij;    // Inicialización de apuntador

    Tripode=req.Tripode;
//    T_actual = req.T;
    lambda_Apoyo_actual = req.lambda;
    velocidadCuerpo_y = req.velApoyo_y;

    cuentaPasos++;
    ROS_INFO("INICIO server_PlanificadorPisada::T[%d]::P[%d] ((((v_y=%.3f))))",Tripode,cuentaPasos,velocidadCuerpo_y);
    fprintf(fp2,"\nINICIO T[%d],Paso[%d]\n",Tripode,cuentaPasos);
    fprintf(fp2,"server_PlanificadorPisada::T[%d]: tiempo de simulacion: %.3f ((((v_y=%.3f))))\n",Tripode,simulationTime,velocidadCuerpo_y);


    if (req.Tripode == T1){
    //-- se intercambian los tripodes
        for(int k=0;k<Npatas/2;k++) {
            Tripode_Transferencia[k] = Tripode2[k];
//    //-- se reporta posicion de tripode en apoyo
//            transformacion_yxTOij(p_ij, posicionActualPata_y[Tripode1[k]], posicionActualPata_x[Tripode1[k]]);
//            infoMapa.coordenadaPreAjuste_i[Tripode1[k]] = ij[0];
//            infoMapa.coordenadaPreAjuste_j[Tripode1[k]] = ij[1];
//            if(matrizMapa[PisadaProxima_i][PisadaProxima_j]){
//                fprintf(fp2,"---ERROR--- pata[%d] Coincide con obstaculo[%d][%d]\n",Tripode1[k]+1,PisadaProxima_i,PisadaProxima_j);
//            }
        }
    } else{
        for(int k=0;k<Npatas/2;k++){
            Tripode_Transferencia[k] = Tripode1[k];
//    //-- se reporta posicion de tripode en apoyo
//            transformacion_yxTOij(p_ij, posicionActualPata_y[Tripode2[k]], posicionActualPata_x[Tripode2[k]]);
//            infoMapa.coordenadaPreAjuste_i[Tripode2[k]] = ij[0];
//            infoMapa.coordenadaPreAjuste_j[Tripode2[k]] = ij[1];
//            if(matrizMapa[PisadaProxima_i][PisadaProxima_j]){
//                fprintf(fp2,"---ERROR--- pata[%d] Coincide con obstaculo[%d][%d]\n",Tripode2[k]+1,PisadaProxima_i,PisadaProxima_j);
//            }
        }
    }
    //-- La correccion del tiempo se hace solo para mantener la velocidad al lambda que llevavas
    res.modificacion_T = T_actual = lambda_Apoyo_actual/velocidadApoyo;

    TodasPisadasOk = true;    // Todas las pisadas se asumen bien a la primera
    for(int k=0;k<Npatas/2;k++){
        ros::spinOnce();
//        ROS_INFO("server_Plan: revisando pata [%d]", Tripode_Transferencia[k]);
    //-- Calculamos proximo movimiento en el sistema de pata
        delta_x_S0 = -lambda_maximo*cos(alfa);
        delta_y_S0 = lambda_maximo*sin(alfa);
        delta_x = delta_x_S0*cos(phi[Tripode_Transferencia[k]]+alfa)-delta_y_S0*sin(phi[Tripode_Transferencia[k]]+alfa);
        delta_y = delta_x_S0*sin(phi[Tripode_Transferencia[k]]+alfa)+delta_y_S0*cos(phi[Tripode_Transferencia[k]]+alfa);
        PisadaProxima_x=posicionActualPataSistemaPata_x[Tripode_Transferencia[k]] + delta_x;
        PisadaProxima_y=posicionActualPataSistemaPata_y[Tripode_Transferencia[k]] + delta_y;
        PisadaProxima_z=posicionActualPataSistemaPata_z[Tripode_Transferencia[k]];
    //-- Verificamos que pisada sea factible
//        ROS_INFO("server_Plan: revisando cinversa");
        srv_Cinversa1.request.x = PisadaProxima_x;
        srv_Cinversa1.request.y = PisadaProxima_y;
        srv_Cinversa1.request.z = PisadaProxima_z;
        if (client_Cinversa1.call(srv_Cinversa1)){
        //-- Funciona servicio
            cinversaOK=true;
        } else {
            ROS_ERROR("server_PlanificadorPisada: servicio de Cinversa no funciona\n");
            cinversaOK=false;
        }
        if (cinversaOK){
//            ROS_INFO("server_Plan: cinversaOK");
            ros::spinOnce();
        //-- Calculamos proximo movimiento en el sistema mundo
                PisadaProxima_y=posicionActualPata_y[Tripode_Transferencia[k]] + (lambda_maximo+velocidadCuerpo_y*T_actual)*cos((teta_CuerpoRobot-teta_Offset)+alfa);
                PisadaProxima_x=posicionActualPata_x[Tripode_Transferencia[k]] + (lambda_maximo+velocidadCuerpo_y*T_actual)*sin((teta_CuerpoRobot-teta_Offset)+alfa);
                transformacion_yxTOij(p_ij, PisadaProxima_y, PisadaProxima_x);
                PisadaProxima_i=ij[0];
                PisadaProxima_j=ij[1];
                PisadaInvalida[k] = false;
//                ROS_INFO("Pisada revisar: i=%d,j=%d",PisadaProxima_i,PisadaProxima_j);
                if(matrizMapa[PisadaProxima_i][PisadaProxima_j]){
                //-- La pisada COINCIDE con obstaculo
                    ROS_WARN("server_PlanificadorPisada: pata [%d] coincidira con obstaculo [%d][%d]",Tripode_Transferencia[k]+1,PisadaProxima_i,PisadaProxima_j);
//                    fprintf(fp2,"tiempo de simulacion: %.3f\n",simulationTime);
                    fprintf(fp2,"pata [%d] coincidira con obstaculo[%d][%d]\n",Tripode_Transferencia[k]+1,PisadaProxima_i,PisadaProxima_j);
                    PisadaInvalida[k] = true;
                    TodasPisadasOk = false;
//                    break;
                }
            if(!PisadaInvalida[k]){
//                ROS_INFO("TripodeOK");
            //-- Todo salio bien! Esta proxima_pisada es valida! :D
                modificacion_lambda[k] = lambda_maximo;
            //-- Pisada maxima
                infoMapa.coordenadaAjuste_i[Tripode_Transferencia[k]] = PisadaProxima_i;
                infoMapa.coordenadaAjuste_j[Tripode_Transferencia[k]] = PisadaProxima_j;
                transformacion_yxTOij(p_ij, posicionActualPata_y[Tripode_Transferencia[k]], posicionActualPata_x[Tripode_Transferencia[k]]);
                infoMapa.coordenadaPreAjuste_i[Tripode_Transferencia[k]] = ij[0];
                infoMapa.coordenadaPreAjuste_j[Tripode_Transferencia[k]] = ij[1];
            }
        } else {
            ROS_WARN("server_PlanificadorPisada: pata [%d] error cinversa",k+1);
//            fprintf(fp2,"tiempo de simulacion: %.3f\n",simulationTime);
            fprintf(fp2,"pata [%d] error cinversa\n",Tripode_Transferencia[k]+1);
            PisadaInvalida[k] = true;
            TodasPisadasOk = false;
        }
    } // Fin de revision de pisadas
    infoMapa.correccion=false;

//---> Aqui va codigo para arreglar pisadas invalidas
    float lambda_Correccion=0.0, T_Correccion=0.0, lambda_paso=0.0;
    int ciclosContraccion=0;
    while (ciclosContraccion<MAX_CICLOS && !TodasPisadasOk){
        infoMapa.correccion=true;
        ROS_WARN("server_PlanificadorPisada: Correccion de pisada ciclo: %d",ciclosContraccion);
//        fprintf(fp2,"tiempo de simulacion: %.3f\n",simulationTime);
        fprintf(fp2,"server_PlanificadorPisada: Correccion de pisada ciclo: %d\n",ciclosContraccion);
        ciclosContraccion++;
        TodasPisadasOk = true;
    //-- Se corrige en tamaños de celda
        lambda_paso = lambda_paso+0.025;
        lambda_Correccion = lambda_maximo-lambda_paso;
        ROS_WARN("server_PlanificadorPisada: lambda_Correccion: %.3f",lambda_Correccion);
//        fprintf(fp2,"tiempo de simulacion: %.3f\n",simulationTime);
        fprintf(fp2,"lambda_Correccion: %.3f\n",lambda_Correccion);

//        T_Correccion = lambda_Correccion/velocidad_Apoyo;

        for (int k=0;k<Npatas/2;k++){
            ros::spinOnce();
            if(PisadaInvalida[k]){
                ROS_WARN("server_PlanificadorPisada: revisando pata[%d]",Tripode_Transferencia[k]+1);
//                fprintf(fp2,"tiempo de simulacion: %.3f\n",simulationTime);
                fprintf(fp2,"revisando pata[%d]\n",Tripode_Transferencia[k]+1);
            //-- Calculamos proximo movimiento en el sistema de pata
                delta_x_S0 = -lambda_Correccion*cos(alfa);
                delta_y_S0 = lambda_Correccion*sin(alfa);
                delta_x = delta_x_S0*cos(phi[Tripode_Transferencia[k]]+alfa)-delta_y_S0*sin(phi[Tripode_Transferencia[k]]+alfa);
                delta_y = delta_x_S0*sin(phi[Tripode_Transferencia[k]]+alfa)+delta_y_S0*cos(phi[Tripode_Transferencia[k]]+alfa);
                PisadaProxima_x=posicionActualPataSistemaPata_x[Tripode_Transferencia[k]] + delta_x;
                PisadaProxima_y=posicionActualPataSistemaPata_y[Tripode_Transferencia[k]] + delta_y;
                PisadaProxima_z=posicionActualPataSistemaPata_z[Tripode_Transferencia[k]];
            //-- Verificamos que pisada sea factible
                srv_Cinversa1.request.x = PisadaProxima_x;
                srv_Cinversa1.request.y = PisadaProxima_y;
                srv_Cinversa1.request.z = PisadaProxima_z;
                if (client_Cinversa1.call(srv_Cinversa1)){
                //-- Funciona servicio
                    cinversaOK=true;
                } else {
                    ROS_ERROR("server_PlanificadorPisada: servicio de Cinversa no funciona\n");
                    cinversaOK=false;
                }
                if (cinversaOK){
                    ros::spinOnce();
                //-- Calculamos proximo movimiento en el sistema mundo
//                    PisadaProxima_y=posicionActualPata_y[Tripode_Transferencia[k]] + (lambda_Correccion+velocidadCuerpo_y*T_Correccion)*cos((teta_CuerpoRobot-teta_Offset)+alfa);
//                    PisadaProxima_x=posicionActualPata_x[Tripode_Transferencia[k]] + (lambda_Correccion+velocidadCuerpo_y*T_Correccion)*sin((teta_CuerpoRobot-teta_Offset)+alfa);
                    PisadaProxima_y=posicionActualPata_y[Tripode_Transferencia[k]] + (lambda_Correccion+velocidadCuerpo_y*T_actual)*cos((teta_CuerpoRobot-teta_Offset)+alfa);
                    PisadaProxima_x=posicionActualPata_x[Tripode_Transferencia[k]] + (lambda_Correccion+velocidadCuerpo_y*T_actual)*sin((teta_CuerpoRobot-teta_Offset)+alfa);
                    transformacion_yxTOij(p_ij, PisadaProxima_y, PisadaProxima_x);
                    PisadaProxima_i=ij[0];
                    PisadaProxima_j=ij[1];
                    PisadaInvalida[k] = false;
                    if(matrizMapa[PisadaProxima_i][PisadaProxima_j]){
                    //-- La pisada COINCIDE con obstaculo
                        ROS_WARN("server_PlanificadorPisada: pata [%d] coincidira con obstaculo [%d][%d]",Tripode_Transferencia[k]+1,PisadaProxima_i,PisadaProxima_j);
//                        fprintf(fp2,"tiempo de simulacion: %.3f\n",simulationTime);
                        fprintf(fp2,"pata [%d] coincidira con obstaculo [%d][%d]\n",Tripode_Transferencia[k]+1,PisadaProxima_i,PisadaProxima_j);
                        PisadaInvalida[k] = true;
                        TodasPisadasOk = false;
                        break;
                    }
                    if(!PisadaInvalida[k]){
                    //-- Todo salio bien! Esta proxima_pisada es valida! :D
                        modificacion_lambda[k] = lambda_Correccion;
                    //-- Pisada corregida
                        infoMapa.coordenadaAjuste_i[Tripode_Transferencia[k]] = PisadaProxima_i;
                        infoMapa.coordenadaAjuste_j[Tripode_Transferencia[k]] = PisadaProxima_j;
                        transformacion_yxTOij(p_ij, posicionActualPata_y[Tripode_Transferencia[k]], posicionActualPata_x[Tripode_Transferencia[k]]);
                        infoMapa.coordenadaPreAjuste_i[Tripode_Transferencia[k]] = ij[0];
                        infoMapa.coordenadaPreAjuste_j[Tripode_Transferencia[k]] = ij[1];
                    }
                } else {
                //-- La pisada no es factible
                    ROS_ERROR("server_PlanificadorPisada: pata [%d] error cinversa",Tripode_Transferencia[k]+1);
//                    fprintf(fp2,"tiempo de simulacion: %.3f\n",simulationTime);
                    fprintf(fp2,"pata [%d] error cinversa\n",Tripode_Transferencia[k]+1);
                    PisadaInvalida[k] = true;
                    TodasPisadasOk = false;
                }
            }// Fin if(pisadaInvalida)
        }   // Fin de for
    }   // Fin while ciclos && !TodasPisadasOk
//-- Si hay alguna pisada invalida detengo la planificacion
    for(int k=0;k<Npatas/2;k++) {
        if(PisadaInvalida[k]){
            ROS_ERROR("server_PlanificadorPisada: No se pudo corregir pisada pata[%d]",Tripode_Transferencia[k]+1);
//            fprintf(fp2,"tiempo de simulacion: %.3f\n",simulationTime);
            fprintf(fp2,"No se pudo corregir pisada pata[%d]\n",Tripode_Transferencia[k]+1);
            senales.Stop=true;
            chatter_pub1.publish(senales);
//            fclose(fp2);
            ROS_INFO("Adios_server_PlanificadorPisada!");
//            ros::shutdown();
//            res.result = -1;
//            return -1;
        }
    }
//-- Escojo el largo de pisada mas corto y lo impongo a todas las patas del tripode
//    ROS_INFO("server_Plan: final PlanificadorPisada");
    std::sort (modificacion_lambda, modificacion_lambda+3);
    if(modificacion_lambda[0]<T_minimo*velocidadApoyo){
        res.modificacion_lambda = T_minimo*velocidadApoyo;
    } else {
        res.modificacion_lambda = modificacion_lambda[0];
    }
//-- Envio trayectoria planificada D: chanchanchaaaaaan
//    ROS_INFO("server_PlanificadorPisada: Tripode=%d, lambda_correccion=%.3f, T_correccion=%.3f",req.Tripode,res.modificacion_lambda,res.modificacion_T);
//-- Envio datos de planificacion al mapa
//    fprintf(fp2,"\ntiempo de simulacion: %.3f\n",simulationTime);
    fprintf(fp2,"server_PlanificadorPisada: Tripode=%d, lambda_correccion=%.3f, T_correccion=%.3f\n",req.Tripode,res.modificacion_lambda,res.modificacion_T);

    for(int k=0;k<Npatas;k++){
    //-- se reporta posicion de tripode en apoyo
        transformacion_yxTOij(p_ij, posicionActualPata_y[k], posicionActualPata_x[k]);
        infoMapa.coordenadaPreAjuste_i[k] = ij[0];
        infoMapa.coordenadaPreAjuste_j[k] = ij[1];
        if(matrizMapa[ij[0]][ij[1]]){
            fprintf(fp2,"---ERROR--- pata[%d] Coincide con obstaculo[%d][%d]\n",k+1,ij[0],ij[1]);
            cuentaErrores[k]++;
        }
    }
//    fprintf(fp2,"Envio datos de mapa");
    chatter_pub2.publish(infoMapa);

    return 1;
}

int main(int argc, char **argv)
{
    int Narg=0, cuentaObs=0;
    std::string fileName;

    Narg=20;
    if (argc>=Narg)
	{
        beta = atof(argv[1]);
        velocidadApoyo = atof(argv[2]);
        alfa = atof(argv[3])*pi/180.0;
        fileName = argv[4];
        nCeldas_i = atoi(argv[5]);
        nCeldas_j = atoi(argv[6]);
        LongitudCeldaY = atof(argv[7]);
        LongitudCeldaX = atof(argv[8]);
        for(int k=0;k<Npatas;k++) phi[k] = atof(argv[9+k])*pi/180.0;
        for(int k=0;k<Npatas;k++) tripode[k] = atoi(argv[9+Npatas+k]);
	} else{
        ROS_ERROR("server_PlanificadorPisada: Indique argumentos completos!\n");
        return (0);
    }

    /*Inicio nodo de ROS*/
    ros::init(argc, argv, "server_PlanificadorPisada");
    ros::NodeHandle node;
    ROS_INFO("server_PlanificadorPisada just started\n");

//-- Topicos susbcritos y publicados
    chatter_pub1=node.advertise<camina7::SenalesCambios>("Senal", 100);
    chatter_pub2=node.advertise<camina7::InfoMapa>("Plan", 100);
    ros::Subscriber sub1=node.subscribe("/vrep/info",100,infoCallback);
    ros::Subscriber sub2=node.subscribe("UbicacionRobot",100,ubicacionRobCallback);
//-- Clientes y Servicios
    ros::ServiceServer service = node.advertiseService("PlanificadorPisada", PlanificadorPisada);
    client_Cinversa1=node.serviceClient<camina7::CinversaParametros>("Cinversa");

    /* Log de planificador */
    fp1 = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina7/datos/RegistroCorrida.txt","a+");
    fp2 = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina7/datos/LogPlanificador.txt","w+");

    for(int k=0;k<Npatas;k++) {
        infoMapa.coordenadaPreAjuste_i.push_back(0);
        infoMapa.coordenadaPreAjuste_j.push_back(0);
        infoMapa.coordenadaAjuste_i.push_back(0);
        infoMapa.coordenadaAjuste_j.push_back(0);
    }
//-- Patas de [0-5]
    int cuenta_T1=0, cuenta_T2=0;
    for(int k=0;k<Npatas;k++) {
        if(tripode[k]==1){
            Tripode1[cuenta_T1]=k;
            cuenta_T1++;
        } else {
            Tripode2[cuenta_T2]=k;
            cuenta_T2++;
        }
    }

    Limpiar_matrizMapa();
    cuentaObs = Construye_matrizMapa(fileName);
//    print_matrizMapa(nCeldas_i,nCeldas_j);
//    ROS_INFO("Nobstaculos=%d",cuentaObs);
//    ROS_INFO("variables de mapa: Ni=%d,Nj=%d,LY=%.3f,LX=%.3f",nCeldas_i,nCeldas_j,LongitudCeldaY,LongitudCeldaX);
    ROS_INFO("server_PlanificadorPisada: Tripode1[%d,%d,%d] - Tripode2[%d,%d,%d]",Tripode1[0]+1,Tripode1[1]+1,Tripode1[2]+1,Tripode2[0]+1,Tripode2[1]+1,Tripode2[2]+1);

    while (ros::ok() && simulationRunning){
    ros::spinOnce();
    }
    fprintf(fp1,"%d\t%d\t",cuentaObs,cuentaPasos);
    for(int k=0;k<Npatas;k++) fprintf(fp1,"%d\t",cuentaErrores[k]);
    fprintf(fp1,"\n");
    fclose(fp1);
    fclose(fp2);
    ROS_INFO("Adios_server_PlanificadorPisada!");
    ros::shutdown();
    return 0;
}

/*En matriz de mapa las coordenadas van de i=[0,99], j=[0,19] */
void transformacion_yxTOij(int *ptr_ij, float y, float x){
    ptr_ij[0] = (int) (nCeldas_i/2 - floor(y/LongitudCeldaY)-1);
    ptr_ij[1] = (int) (nCeldas_j/2 + floor(x/LongitudCeldaX));
}


void Limpiar_matrizMapa(){

    int i=0, j=0;

    for(i=0;i<nCeldas_i;i++){
        for(j=0;j<nCeldas_j;j++){
            matrizMapa[i][j]=false;
        }
    }
}

int Construye_matrizMapa(std::string fileName){
    //--- Construccion de mapa mediante lectura de archivo
//    printf ("%s \n", fileName.c_str());
    FILE *fp;
    int int_aux=0, i=0, j=0, cuentaObs=0;

    fp = fopen(fileName.c_str(), "r");
    if(fp!=NULL){
        printf("\n");
        for(i=0;i<nCeldas_i;i++){
            for(j=0;j<nCeldas_j;j++){
                fscanf(fp, "%d", &int_aux);
                if (int_aux==0) matrizMapa[i][j]=false;
                if (int_aux==1) {
                    matrizMapa[i][j]=true;
                    cuentaObs++;
//                    ROS_INFO("obstaculo_i: %d",i);
                }
            }
        }
//        printf("Mapa inicial\n");
//        print_matrizMapa(nCeldas_i,nCeldas_j);
    }
    fclose(fp);
    return (cuentaObs);
}

void print_matrizMapa(){
     printf("\n");
     for(int i=0;i<nCeldas_i;i++){
         for(int j=0;j<nCeldas_j;j++){
            if (matrizMapa[i][j]){
                printf ("o.");
            } else {
                printf("-.");
            }
         }
        printf("\n");
    }
}
