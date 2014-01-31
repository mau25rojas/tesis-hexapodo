#include "ros/ros.h"
#include "std_msgs/String.h"
#include "math.h"
#include <algorithm>    // std::sort
//Librerias propias usadas
#include "constantes.hpp"
#include "camina6/v_repConst.h"
// Used data structures:
#include "camina6/InfoMapa.h"
#include "camina6/CinversaParametros.h"
#include "camina6/UbicacionRobot.h"
#include "camina6/PlanificadorParametros.h"
#include "camina6/SenalesCambios.h"
// Used API services:
#include "vrep_common/VrepInfo.h"
// Definiciones
#define MAX_CICLOS 5
//Clientes y Servicios
ros::ServiceClient client_Cinversa1;
camina6::CinversaParametros srv_Cinversa1;

//-- Variables Globales
bool simulationRunning=true;
bool sensorTrigger=false;
float simulationTime=0.0f;
//-- Log de planificador
FILE *fp2;
//-- Entrada
int Tripode=0, tripode[Npatas], Tripode1[Npatas/2], Tripode2[Npatas/2];
float velocidad_Apoyo=0.0, beta=0.0, phi[Npatas], alfa=0.0;
//-- Variables de mapa
camina6::InfoMapa infoMapa;
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
camina6::SenalesCambios senales;
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


void ubicacionRobCallback(camina6::UbicacionRobot msgUbicacionRobot)
{
    float vel_aux=0.0;

    teta_CuerpoRobot = msgUbicacionRobot.orientacionCuerpo_yaw;

    vel_aux = msgUbicacionRobot.velocidadCuerpo_y;
    if(vel_aux>velocidad_Apoyo){
//-- la velocidad no puede ser mayor a la esperada
        velocidadCuerpo_y = velocidad_Apoyo;
    } else {
        velocidadCuerpo_y = vel_aux;
    }
    velocidadCuerpo_y= ajuste_Vel*velocidadCuerpo_y;

    for(int k=0; k<Npatas;k++) {
        posicionActualPata_x[k] = msgUbicacionRobot.coordenadaPata_x[k];
        posicionActualPata_y[k] = msgUbicacionRobot.coordenadaPata_y[k];
        posicionActualPata_z[k] = msgUbicacionRobot.coordenadaPata_z[k];
        posicionActualPataSistemaPata_x[k] = msgUbicacionRobot.coordenadaPataSistemaPata_x[k];
        posicionActualPataSistemaPata_y[k] = msgUbicacionRobot.coordenadaPataSistemaPata_y[k];
        posicionActualPataSistemaPata_z[k] = msgUbicacionRobot.coordenadaPataSistemaPata_z[k];
    }
}

bool PlanificadorPisada(camina6::PlanificadorParametros::Request  &req,
                        camina6::PlanificadorParametros::Response &res)
{
//    ROS_INFO("Llamado a servicio planificador");
//    fprintf(fp2,"\ntiempo de simulacion: %.3f\t",simulationTime);

//-- Datos para envio de mensajes
    res.modificacion_T = 0.0;
    res.modificacion_lambda = 0.0;
    res.result = 0;
//-- Variables locales
    int Tripode_Apoyo[Npatas/2];
    int PisadaProxima_i=0, PisadaProxima_j=0;
    bool PisadaInvalida[Npatas/2], TodasPisadasOk, cinversaOK;
    float PisadaProxima_x=0.0, PisadaProxima_y=0.0, PisadaProxima_z=0.0;
    float delta_x_S0=0.0, delta_y_S0=0.0, delta_x=0.0, delta_y=0.0;
    float modificacion_lambda[Npatas/2];
    float T_actual=0.0;
    int ij[2]={0,0}, *p_ij;     //Apuntadores a arreglos de coordenadas e indices
    p_ij = ij;    // Inicialización de apuntador

    Tripode=req.Tripode;

    ROS_INFO("server_Plan::T[%d] v_y=%.3f",Tripode,velocidadCuerpo_y);
    fprintf(fp2,"tiempo de simulacion: %.3f\n",simulationTime);
    fprintf(fp2,"server_Plan::T[%d] v_y=%.3f\n",Tripode,velocidadCuerpo_y);


    if (req.Tripode == 1){
        for(int k=0;k<Npatas/2;k++) Tripode_Apoyo[k] = Tripode1[k];
    } else{
        for(int k=0;k<Npatas/2;k++) Tripode_Apoyo[k] = Tripode2[k];
    }
    T_actual = req.T;

    TodasPisadasOk = true;    // Todas las pisadas se asumen bien a la primera
    for(int k=0;k<Npatas/2;k++){
        ros::spinOnce();
//        ROS_INFO("server_Plan: revisando pata [%d]", Tripode_Apoyo[k]);
    //-- Calculamos proximo movimiento en el sistema de pata
        delta_x_S0 = -lambda_maximo*cos(alfa);
        delta_y_S0 = lambda_maximo*sin(alfa);
        delta_x = delta_x_S0*cos(phi[Tripode_Apoyo[k]]+alfa)-delta_y_S0*sin(phi[Tripode_Apoyo[k]]+alfa);
        delta_y = delta_x_S0*sin(phi[Tripode_Apoyo[k]]+alfa)+delta_y_S0*cos(phi[Tripode_Apoyo[k]]+alfa);
        PisadaProxima_x=posicionActualPataSistemaPata_x[Tripode_Apoyo[k]] + delta_x;
        PisadaProxima_y=posicionActualPataSistemaPata_y[Tripode_Apoyo[k]] + delta_y;
        PisadaProxima_z=posicionActualPataSistemaPata_z[Tripode_Apoyo[k]];
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
                PisadaProxima_y=posicionActualPata_y[Tripode_Apoyo[k]] + (lambda_maximo+velocidadCuerpo_y*(1-beta)*T_actual)*cos((teta_CuerpoRobot-teta_Offset)+alfa);
                PisadaProxima_x=posicionActualPata_x[Tripode_Apoyo[k]] + (lambda_maximo+velocidadCuerpo_y*(1-beta)*T_actual)*sin((teta_CuerpoRobot-teta_Offset)+alfa);
                transformacion_yxTOij(p_ij, PisadaProxima_y, PisadaProxima_x);
                PisadaProxima_i=ij[0];
                PisadaProxima_j=ij[1];
                PisadaInvalida[k] = false;
//                ROS_INFO("Pisada revisar: i=%d,j=%d",PisadaProxima_i,PisadaProxima_j);
                if(matrizMapa[PisadaProxima_i][PisadaProxima_j]){
                //-- La pisada COINCIDE con obstaculo
                    ROS_WARN("server_PlanificadorPisada: pata [%d] coincide con obstaculo [%d][%d]",Tripode_Apoyo[k]+1,PisadaProxima_i,PisadaProxima_j);
                    fprintf(fp2,"tiempo de simulacion: %.3f\n",simulationTime);
                    fprintf(fp2,"pata [%d] coincide con obstaculo[%d][%d]\n",Tripode_Apoyo[k]+1,PisadaProxima_i,PisadaProxima_j);
                    PisadaInvalida[k] = true;
                    TodasPisadasOk = false;
//                    break;
                }
            if(!PisadaInvalida[k]){
//                ROS_INFO("TripodeOK");
            //-- Todo salio bien! Esta proxima_pisada es valida! :D
                modificacion_lambda[k] = lambda_maximo;
            //-- Pisada maxima
                infoMapa.coordenadaAjuste_i[Tripode_Apoyo[k]] = PisadaProxima_i;
                infoMapa.coordenadaAjuste_j[Tripode_Apoyo[k]] = PisadaProxima_j;
                transformacion_yxTOij(p_ij, posicionActualPata_y[Tripode_Apoyo[k]], posicionActualPata_x[Tripode_Apoyo[k]]);
                infoMapa.coordenadaPreAjuste_i[Tripode_Apoyo[k]] = ij[0];
                infoMapa.coordenadaPreAjuste_j[Tripode_Apoyo[k]] = ij[1];
            }
        } else {
            ROS_WARN("server_PlanificadorPisada: pata [%d] error cinversa",k+1);
            fprintf(fp2,"tiempo de simulacion: %.3f\n",simulationTime);
            fprintf(fp2,"pata [%d] error cinversa\n",Tripode_Apoyo[k]+1);
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
        fprintf(fp2,"tiempo de simulacion: %.3f\n",simulationTime);
        fprintf(fp2,"server_PlanificadorPisada: Correccion de pisada ciclo: %d\n",ciclosContraccion);
        ciclosContraccion++;
        TodasPisadasOk = true;
    //-- Se corrige en tamaños de celda
        lambda_paso = lambda_paso+0.025;
        lambda_Correccion = lambda_maximo-lambda_paso;
        ROS_WARN("server_PlanificadorPisada: lambda_Correccion: %.3f",lambda_Correccion);
        fprintf(fp2,"tiempo de simulacion: %.3f\n",simulationTime);
        fprintf(fp2,"lambda_Correccion: %.3f\n",lambda_Correccion);
        T_Correccion = lambda_Correccion/(beta*velocidad_Apoyo);
        for (int k=0;k<Npatas/2;k++){
            ros::spinOnce();
            if(PisadaInvalida[k]){
                ROS_WARN("server_PlanificadorPisada: revisando pata[%d]",Tripode_Apoyo[k]+1);
                fprintf(fp2,"tiempo de simulacion: %.3f\n",simulationTime);
                fprintf(fp2,"revisando pata[%d]\n",Tripode_Apoyo[k]+1);
            //-- Calculamos proximo movimiento en el sistema de pata
                delta_x_S0 = -lambda_Correccion*cos(alfa);
                delta_y_S0 = lambda_Correccion*sin(alfa);
                delta_x = delta_x_S0*cos(phi[Tripode_Apoyo[k]]+alfa)-delta_y_S0*sin(phi[Tripode_Apoyo[k]]+alfa);
                delta_y = delta_x_S0*sin(phi[Tripode_Apoyo[k]]+alfa)+delta_y_S0*cos(phi[Tripode_Apoyo[k]]+alfa);
                PisadaProxima_x=posicionActualPataSistemaPata_x[Tripode_Apoyo[k]] + delta_x;
                PisadaProxima_y=posicionActualPataSistemaPata_y[Tripode_Apoyo[k]] + delta_y;
                PisadaProxima_z=posicionActualPataSistemaPata_z[Tripode_Apoyo[k]];
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
                    PisadaProxima_y=posicionActualPata_y[Tripode_Apoyo[k]] + (lambda_Correccion+velocidadCuerpo_y*(1-beta)*T_Correccion)*cos((teta_CuerpoRobot-teta_Offset)+alfa);
                    PisadaProxima_x=posicionActualPata_x[Tripode_Apoyo[k]] + (lambda_Correccion+velocidadCuerpo_y*(1-beta)*T_Correccion)*sin((teta_CuerpoRobot-teta_Offset)+alfa);
                    transformacion_yxTOij(p_ij, PisadaProxima_y, PisadaProxima_x);
                    PisadaProxima_i=ij[0];
                    PisadaProxima_j=ij[1];
                    PisadaInvalida[k] = false;
                    if(matrizMapa[PisadaProxima_i][PisadaProxima_j]){
                    //-- La pisada COINCIDE con obstaculo
                        ROS_WARN("server_PlanificadorPisada: pata [%d] coincide con obstaculo [%d][%d]",Tripode_Apoyo[k]+1,PisadaProxima_i,PisadaProxima_j);
                        fprintf(fp2,"tiempo de simulacion: %.3f\n",simulationTime);
                        fprintf(fp2,"pata [%d] coincide con obstaculo [%d][%d]\n",Tripode_Apoyo[k]+1,PisadaProxima_i,PisadaProxima_j);
                        PisadaInvalida[k] = true;
                        TodasPisadasOk = false;
                        break;
                    }
                    if(!PisadaInvalida[k]){
                    //-- Todo salio bien! Esta proxima_pisada es valida! :D
                        modificacion_lambda[k] = lambda_Correccion;
                    //-- Pisada corregida
                        infoMapa.coordenadaAjuste_i[Tripode_Apoyo[k]] = PisadaProxima_i;
                        infoMapa.coordenadaAjuste_j[Tripode_Apoyo[k]] = PisadaProxima_j;
                        transformacion_yxTOij(p_ij, posicionActualPata_y[Tripode_Apoyo[k]], posicionActualPata_x[Tripode_Apoyo[k]]);
                        infoMapa.coordenadaPreAjuste_i[Tripode_Apoyo[k]] = ij[0];
                        infoMapa.coordenadaPreAjuste_j[Tripode_Apoyo[k]] = ij[1];
                    }
                } else {
                //-- La pisada no es factible
                    ROS_ERROR("server_PlanificadorPisada: pata [%d] error cinversa",Tripode_Apoyo[k]+1);
                    fprintf(fp2,"tiempo de simulacion: %.3f\n",simulationTime);
                    fprintf(fp2,"pata [%d] error cinversa\n",Tripode_Apoyo[k]+1);
                    PisadaInvalida[k] = true;
                    TodasPisadasOk = false;
                }
            }// Fin if(pisadaInvalida)
        }   // Fin de for
    }   // Fin while ciclos && !TodasPisadasOk
//-- Si hay alguna pisada invalida detengo la planificacion
    for(int k=0;k<Npatas/2;k++) {
        if(PisadaInvalida[k]){
            ROS_ERROR("server_PlanificadorPisada: No se pudo corregir pisada pata[%d]",Tripode_Apoyo[k]+1);
            fprintf(fp2,"tiempo de simulacion: %.3f\n",simulationTime);
            fprintf(fp2,"No se pudo corregir pisada pata[%d]\n",Tripode_Apoyo[k]+1);
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
    if(modificacion_lambda[0]<lambda_minimo){
        res.modificacion_lambda = lambda_minimo;
    } else {
        res.modificacion_lambda = modificacion_lambda[0];
    }

    res.modificacion_T = res.modificacion_lambda/(beta*velocidad_Apoyo);
//-- Envio trayectoria planificada D: chanchanchaaaaaan
//    ROS_INFO("server_PlanificadorPisada: Tripode=%d, landa_correccion=%.3f, T_correccion=%.3f",req.Tripode,res.modificacion_lambda,res.modificacion_T);
//-- Envio datos de planificacion al mapa
    fprintf(fp2,"\ntiempo de simulacion: %.3f\n",simulationTime);
    fprintf(fp2,"server_PlanificadorPisada: Tripode=%d, landa_correccion=%.3f, T_correccion=%.3f",req.Tripode,res.modificacion_lambda,res.modificacion_T);
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
        velocidad_Apoyo = atof(argv[2]);
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
    chatter_pub1=node.advertise<camina6::SenalesCambios>("Senal", 100);
    chatter_pub2=node.advertise<camina6::InfoMapa>("Plan", 100);
    ros::Subscriber sub1=node.subscribe("/vrep/info",100,infoCallback);
    ros::Subscriber sub2=node.subscribe("UbicacionRobot",100,ubicacionRobCallback);
//-- Clientes y Servicios
    ros::ServiceServer service = node.advertiseService("PlanificadorPisada", PlanificadorPisada);
    client_Cinversa1=node.serviceClient<camina6::CinversaParametros>("Cinversa");

    /* Log de planificador */
    fp2 = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina6/datos/LogPlanificador.txt","w+");

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
