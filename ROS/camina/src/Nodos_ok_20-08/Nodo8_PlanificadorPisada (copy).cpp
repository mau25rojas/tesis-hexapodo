#include "ros/ros.h"
#include "std_msgs/String.h"
#include "math.h"
//Librerias propias usadas
#include "constantes.hpp"
#include "camina/v_repConst.h"
// Used data structures:
#include "../msg_gen/cpp/include/camina/DatosTrayectoriaPata.h"
#include "../msg_gen/cpp/include/camina/InfoMapa.h"
#include "../srv_gen/cpp/include/camina/CinversaParametros.h"
#include "../msg_gen/cpp/include/camina/UbicacionRobot.h"
// Used API services:
#include "vrep_common/VrepInfo.h"
// Definiciones
#define MAX_CICLOS 10
//Clientes y Servicios
ros::ServiceClient client_Cinversa;
camina::CinversaParametros srv_Cinversa;

// variables Globales
//--Variables de simulacion
bool simulationRunning=true;
bool sensorTrigger=false;
float simulationTime=0.0f;
//-- Variables de datos trayectoria
float x_Offset=0.0, y_Offset=0.0, z_Offset=0.0;   //matriz de transformacion homogenea
float alfa_rad[Npatas],phi_rad=0.0; //matriz de transformacion homogenea
//-- Variables de mapa
camina::InfoMapa infoMapa;
std::vector<int> coordenadaObstaculo_i(1000,0), coordenadaObstaculo_j(1000,0);
int cantidadObstaculos=0, nCeldas_i=0, nCeldas_j=0;
float LongitudCeldaY=0, LongitudCeldaX=0;
//-- Variables de ubicacion robot
float posicionActualPata_y[Npatas], posicionActualPata_x[Npatas], posicionActualPata_z[Npatas];
float posicionActualPataSistemaPata_y[Npatas],posicionActualPataSistemaPata_x[Npatas],posicionActualPataSistemaPata_z[Npatas];
int pataApoyo[Npatas]={0,0,0,0,0,0};
float teta_CuerpoRobot=0.0;
//-- Log de planificador
FILE *fp2;
//-- Generales
//int k=0;

// Funciones
void transformacion_yxTOij(int *ptr_ij, float y, float x);
void FilePrint_matrizMapa(int nFilas, int nColumnas);

// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}

void infoMapaCallback(camina::InfoMapa msgInfoMapa)
{
	nCeldas_i = msgInfoMapa.nCeldas_i;
	nCeldas_j = msgInfoMapa.nCeldas_j;
	LongitudCeldaY=msgInfoMapa.tamanoCelda_i;
	LongitudCeldaX=msgInfoMapa.tamanoCelda_j;
    cantidadObstaculos = msgInfoMapa.cantidadObstaculos;
	for(int k=0; k<cantidadObstaculos; k++){
	    coordenadaObstaculo_i[k]=msgInfoMapa.coordenadaObstaculo_i[k];
        coordenadaObstaculo_j[k]=msgInfoMapa.coordenadaObstaculo_j[k];
	}
}

void ubicacionRobCallback(camina::UbicacionRobot msgUbicacionRobot)
{
    for(int k=0; k<Npatas;k++) {
        posicionActualPata_x[k] = msgUbicacionRobot.coordenadaPata_x[k];
        posicionActualPata_y[k] = msgUbicacionRobot.coordenadaPata_y[k];
        posicionActualPata_z[k] = msgUbicacionRobot.coordenadaPata_z[k];
        posicionActualPataSistemaPata_x[k] = msgUbicacionRobot.coordenadaPataSistemaPata_x[k];
        posicionActualPataSistemaPata_y[k] = msgUbicacionRobot.coordenadaPataSistemaPata_y[k];
        posicionActualPataSistemaPata_z[k] = msgUbicacionRobot.coordenadaPataSistemaPata_z[k];
        pataApoyo[k] = msgUbicacionRobot.pataApoyo[k];
//        printf("Nodo8: Pata apoyo[%d]=%d\n",k,pataApoyo[k]);
    }
    teta_CuerpoRobot = msgUbicacionRobot.orientacionCuerpo_yaw;
}

void datosCallback(const camina::DatosTrayectoriaPata msg_datoTrayectoria)
{
    x_Offset=msg_datoTrayectoria.p_Offset[0];
	y_Offset=msg_datoTrayectoria.p_Offset[1];
	z_Offset=msg_datoTrayectoria.p_Offset[2];
	for(int k=0;k<Npatas;k++) {
        alfa_rad[k] = msg_datoTrayectoria.alfa[k]*pi/180.0;
	}
    phi_rad = msg_datoTrayectoria.phi*pi/180.0;
}

int main(int argc, char **argv)
{
    float VelocidadTransferencia=0.0, teta_Offset_rad=0.0, lambda_Transferencia_arg=0.0, beta_arg=0.0;
    camina::DatosTrayectoriaPata datosTrayectoriaModificada;
    bool PisadaInvalida[Npatas], TodasPisadasOk, cinversaOK;
    int PisadaProxima_i=0, PisadaProxima_j=0;
    float PisadaProxima_x=0.0, PisadaProxima_y=0.0, PisadaProxima_z=0.0;
//    float lambda_Transferencia_maximo=0.0, lambda_Transferencia_paso=0.0;
    float delta_x_S0=0.0, delta_y_S0=0.0, delta_x=0.0, delta_y=0.0;
    int Narg=0;
    float Periodo=0.0, f=0.0;
    int ij[2]={0,0}, *p_ij;     //Apuntadores a arreglos de coordenadas e indices
    Narg=2;
    if (argc>=Narg)
	{
//        cantidadObstaculos = atoi(argv[1]);
        lambda_Transferencia_arg = atof(argv[1]);
        beta_arg = atof(argv[2]);
	} else{
        ROS_ERROR("Nodo 8: Indique argumentos completos!\n");
        return (0);
    }
//    for(k=0;k<cantidadObstaculos;k++) {
//        coordenadaObstaculo_i.push_back(0);
//        coordenadaObstaculo_j.push_back(0);
//    }
//    VelocidadApoyo=lambda_Transferencia/beta_arg;
        VelocidadTransferencia=lambda_Transferencia_arg/beta_arg;

    for(int k=0;k<Npatas;k++) {
        datosTrayectoriaModificada.beta.push_back(beta_arg);
        datosTrayectoriaModificada.lambda_Transferencia.push_back(lambda_Transferencia_arg);
        infoMapa.coordenadaPreAjuste_i.push_back(0);
        infoMapa.coordenadaPreAjuste_j.push_back(0);
        infoMapa.coordenadaAjuste_i.push_back(0);
        infoMapa.coordenadaAjuste_j.push_back(0);
    }
    p_ij = ij;    // Inicialización de apuntador

    /*Inicio nodo de ROS*/
    ros::init(argc, argv, "Nodo8_PlanificacionPisada");
    ros::NodeHandle node;
    ROS_INFO("Nodo8_PlanificacionPisada just started\n");

    //Topicos susbcritos y publicados
    ros::Publisher chatter_pub=node.advertise<camina::DatosTrayectoriaPata>("PlanificacionDePisada", 100);
    ros::Publisher chatter_pub1=node.advertise<camina::InfoMapa>("Plan", 100);
    ros::Subscriber sub1=node.subscribe("/vrep/info",100,infoCallback);
    ros::Subscriber sub2=node.subscribe("UbicacionRobot",100,ubicacionRobCallback);
    ros::Subscriber sub3=node.subscribe("GraficaMapa",100,infoMapaCallback);
    ros::Subscriber sub4=node.subscribe("datosTrayectoriaPataEntrada",100,datosCallback);
    //Clientes y Servicios
    client_Cinversa=node.serviceClient<camina::CinversaParametros>("Cinversa");

    //Variables internas
    teta_Offset_rad = teta_Offset*pi/180.0;
//    lambda_Transferencia_maximo = lambda_Transferencia_max;
//    lambda_Transferencia_paso=lambda_Transferencia_p;
    /* Log de planificador */
    fp2 = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina/datos/LogPlanificador.txt","w+");
    /* Velocidad de transmision */
    Periodo = 0.5;
    f=1/Periodo;
    ros::Rate loop_rate(f);  //Frecuencia [Hz]
    while (ros::ok() && simulationRunning){
        ros::spinOnce();
        loop_rate.sleep();
        TodasPisadasOk = true;    // Todas las pisadas se asumen bien a la primera
        // Revision de pisadas validas a la primera
//        ROS_WARN("Revision pisadas");
        for(int k=0;k<Npatas;k++){
            ros::spinOnce();
//            printf("Nodo8: Pata apoyo[%d]=%d\n",k,pataApoyo[k]);
            if (pataApoyo[k]==1){
            // Calculamos proximo movimiento en el sistema de pata
                delta_x_S0 = -lambda_maximo*cos(phi_rad);
                delta_y_S0 = lambda_maximo*sin(phi_rad);
                delta_x = delta_x_S0*cos(alfa_rad[k]+phi_rad)-delta_y_S0*sin(alfa_rad[k]+phi_rad);
                delta_y = delta_x_S0*sin(alfa_rad[k]+phi_rad)+delta_y_S0*cos(alfa_rad[k]+phi_rad);
                PisadaProxima_x=posicionActualPataSistemaPata_x[k] + delta_x;
                PisadaProxima_y=posicionActualPataSistemaPata_y[k] + delta_y;
                PisadaProxima_z=posicionActualPataSistemaPata_z[k];
            // Verificamos que pisada sea factible
                srv_Cinversa.request.x = PisadaProxima_x;
                srv_Cinversa.request.y = PisadaProxima_y;
                srv_Cinversa.request.z = PisadaProxima_z;
                if (client_Cinversa.call(srv_Cinversa)){
                // Funciona servicio
                    cinversaOK=true;
                } else {
                    ROS_ERROR("Nodo 8: servicio de Cinversa no funciona\n");
                    cinversaOK=false;
                }
                if (cinversaOK){
                    for(int l=0;l<cantidadObstaculos;l++){
                        ros::spinOnce();
                    // Calculamos proximo movimiento en el sistema mundo
                        PisadaProxima_y=posicionActualPata_y[k] + lambda_maximo*cos((teta_CuerpoRobot-teta_Offset_rad)+phi_rad);
                        PisadaProxima_x=posicionActualPata_x[k] + lambda_maximo*sin((teta_CuerpoRobot-teta_Offset_rad)+phi_rad);
                        transformacion_yxTOij(p_ij, PisadaProxima_y, PisadaProxima_x);
                        PisadaProxima_i=ij[0];
                        PisadaProxima_j=ij[1];
                        PisadaInvalida[k] = false;
                        if(PisadaProxima_i==coordenadaObstaculo_i[l] && PisadaProxima_j==coordenadaObstaculo_j[l]){
                        // La pisada COINCIDE con obstaculo
//                            ROS_WARN("pata [%d] coincide con obstaculo",k+1);
                            PisadaInvalida[k] = true;
                            TodasPisadasOk = false;
                            break;
                        }
                    }
                    if(!PisadaInvalida[k]){
                    // Todo salio bien! Esta proxima_pisada es valida! :D
                        datosTrayectoriaModificada.lambda_Transferencia[k] = lambda_maximo;
                        datosTrayectoriaModificada.beta[k] = lambda_maximo/VelocidadTransferencia;
                        transformacion_yxTOij(p_ij, posicionActualPata_y[k], posicionActualPata_x[k]);
                        infoMapa.coordenadaPreAjuste_i[k] = ij[0];
                        infoMapa.coordenadaPreAjuste_j[k] = ij[1];
//                        ROS_INFO("Posicion actual:[%d] y:%.3f\t x:%.3f",k+1,posicionActualPata_y[k],posicionActualPata_x[k]);
//                        ROS_INFO("Posicion actual:[%d] i:%d\t j:%d",k+1,ij[0],ij[1]);
                        infoMapa.coordenadaAjuste_i[k] = PisadaProxima_i;
                        infoMapa.coordenadaAjuste_j[k] = PisadaProxima_j;
//                        chatter_pub1.publish(infoMapa);
//                      ROS_INFO("Bien:[%d] beta:%.3f\t lambda_Transferencia:%.3f\n",k,datosTrayectoriaModificada.beta[k],datosTrayectoriaModificada.lambda_Transferencia[k]);
                        chatter_pub.publish(datosTrayectoriaModificada);
                        }
                } else {
//                    ROS_WARN("pata [%d] error cinversa",k+1);
//                    printf("x: %.3f, y:%.3f, z:%.3f, alfa+phi:%.3f\n",PisadaProxima_x,PisadaProxima_y,PisadaProxima_z,alfa_rad[k]+phi_rad);
                // La pisada no es factible
                    PisadaInvalida[k] = true;
                    TodasPisadasOk = false;
                }
            } else {
            // Las pisadas al aire las ignoramos
                PisadaInvalida[k] = false;
                datosTrayectoriaModificada.lambda_Transferencia[k] = lambda_maximo;
                datosTrayectoriaModificada.beta[k] = lambda_maximo/VelocidadTransferencia;
            }
        } // Fin de revision de pisadas

        //---> Aqui va codigo para arreglar pisadas invalidas
        float lambda_Correccion = 0.0;
        int ciclosContraccion = 0;
        while (ciclosContraccion<MAX_CICLOS && !TodasPisadasOk){
            ros::spinOnce();
            fprintf(fp2,"\ntiempo de simulacion: %.3f\n",simulationTime);
            fprintf(fp2,"Correccion de pisada ciclo: %d\n",ciclosContraccion);
            ROS_WARN("Correccion de pisada ciclo: %d",ciclosContraccion);
            ciclosContraccion++;
            TodasPisadasOk = true;
            lambda_Correccion = lambda_Correccion+lambda_paso;
            for (int k=0;k<Npatas;k++){
                ros::spinOnce();
                if(PisadaInvalida[k]){
                // Calculamos proximo movimiento en el sistema de pata
                    delta_x_S0 = -(lambda_maximo-lambda_Correccion)*cos(phi_rad);
                    delta_y_S0 = (lambda_maximo-lambda_Correccion)*sin(phi_rad);
                    delta_x = delta_x_S0*cos(alfa_rad[k]+phi_rad)-delta_y_S0*sin(alfa_rad[k]+phi_rad);
                    delta_y = delta_x_S0*sin(alfa_rad[k]+phi_rad)+delta_y_S0*cos(alfa_rad[k]+phi_rad);
                    PisadaProxima_x=posicionActualPataSistemaPata_x[k] + delta_x;
                    PisadaProxima_y=posicionActualPataSistemaPata_y[k] + delta_y;
                    PisadaProxima_z=posicionActualPataSistemaPata_z[k];
                // Verificamos que pisada sea factible
                    srv_Cinversa.request.x = PisadaProxima_x;
                    srv_Cinversa.request.y = PisadaProxima_y;
                    srv_Cinversa.request.z = PisadaProxima_z;
                    if (client_Cinversa.call(srv_Cinversa)){
                    // Funciona servicio
                        cinversaOK=true;
                    } else {
                        ROS_ERROR("Nodo 8: servicio de Cinversa no funciona\n");
                        cinversaOK=false;
                    }
                    if (cinversaOK){
                    for(int l=0;l<cantidadObstaculos;l++){
                        ros::spinOnce();
                    // Calculamos proximo movimiento en el sistema mundo
                        PisadaProxima_y=posicionActualPata_y[k] + (lambda_maximo-lambda_Correccion)*cos((teta_CuerpoRobot-teta_Offset_rad)+phi_rad);
                        PisadaProxima_x=posicionActualPata_x[k] + (lambda_maximo-lambda_Correccion)*sin((teta_CuerpoRobot-teta_Offset_rad)+phi_rad);
                        transformacion_yxTOij(p_ij, PisadaProxima_y, PisadaProxima_x);
                        PisadaProxima_i=ij[0];
                        PisadaProxima_j=ij[1];
                        PisadaInvalida[k] = false;
                        if(PisadaProxima_i==coordenadaObstaculo_i[l] && PisadaProxima_j==coordenadaObstaculo_j[l]){
                        // La pisada COINCIDE con obstaculo
                            ROS_WARN("pata [%d] coincide con obstaculo",k+1);
                            fprintf(fp2,"\ntiempo de simulacion: %.3f\n",simulationTime);
                            fprintf(fp2,"pata [%d] coincide con obstaculo\n",k+1);
                            PisadaInvalida[k] = true;
                            TodasPisadasOk = false;
                            break;
                        }
                    }
                        if(!PisadaInvalida[k]){
                        // Todo salio bien! Esta proxima_pisada es valida! :D
                            datosTrayectoriaModificada.lambda_Transferencia[k] = (lambda_maximo-lambda_Correccion);
                            datosTrayectoriaModificada.beta[k] = (lambda_maximo-lambda_Correccion)/VelocidadTransferencia;
                            transformacion_yxTOij(p_ij, posicionActualPata_y[k], posicionActualPata_x[k]);
                            infoMapa.coordenadaPreAjuste_i[k] = ij[0];
                            infoMapa.coordenadaPreAjuste_j[k] = ij[1];
                            infoMapa.coordenadaAjuste_i[k] = PisadaProxima_i;
                            infoMapa.coordenadaAjuste_j[k] = PisadaProxima_j;
//                            chatter_pub1.publish(infoMapa);
                            ROS_INFO("Correccion:[%d] beta:%.3f\t lambda_Transferencia:%.3f",k+1,datosTrayectoriaModificada.beta[k],datosTrayectoriaModificada.lambda_Transferencia[k]);
                            fprintf(fp2,"\ntiempo de simulacion: %.3f\n",simulationTime);
                            fprintf(fp2,"Correccion:[%d] beta:%.3f\t lambda_Transferencia:%.3f\n",k+1,datosTrayectoriaModificada.beta[k],datosTrayectoriaModificada.lambda_Transferencia[k]);
                            chatter_pub.publish(datosTrayectoriaModificada);
                        }
                    } else {
                    // La pisada no es factible
                        ROS_WARN("pata [%d] error cinversa",k+1);
                        fprintf(fp2,"\ntiempo de simulacion: %.3f\n",simulationTime);
                        fprintf(fp2,"pata [%d] error cinversa\n",k+1);
//                        printf("Actuales: x: %.3f, y:%.3f, z:%.3f\n",posicionActualPataSistemaPata_x[k],posicionActualPataSistemaPata_y[k],posicionActualPataSistemaPata_z[k]);
//                        printf("Deltas_S0: delta_x_S0: %.3f, delta_y_S0:%.3f\n",delta_x_S0,delta_y_S0);
//                        printf("Deltas: delta_x: %.3f, delta_y:%.3f\n",delta_x,delta_y);
//                        printf("Proximas: x: %.3f, y:%.3f, z:%.3f, alfa+phi:%.3f\n",PisadaProxima_x,PisadaProxima_y,PisadaProxima_z,alfa_rad[k]+phi_rad);
                        PisadaInvalida[k] = true;
                        TodasPisadasOk = false;
                    }
                }
            }   // Fin de for
        }   // Fin while ciclos && !TodasPisadasOk
//        ROS_INFO("Fin ciclo Nodo 8\n");
        chatter_pub.publish(datosTrayectoriaModificada);
        chatter_pub1.publish(infoMapa);
//        ROS_INFO("Correccion:\n");
//        for(k=0; k<Npatas;k++) ROS_INFO("Nodo 8: Pata[%d] beta:%.3f\t lambda_Transferencia:%.3f\n",k+1,datosTrayectoriaModificada.beta[k],datosTrayectoriaModificada.lambda_Transferencia[k]);
    }
    fclose(fp2);
    ROS_INFO("Adios8!");
    ros::shutdown();
    return 0;
}


void transformacion_yxTOij(int *ptr_ij, float y, float x){
    if (y>=0){
        ptr_ij[0] = (int) (nCeldas_i/2 - floor(y/LongitudCeldaY)-1);
    }else{
        ptr_ij[0] = (int) (nCeldas_i/2 - floor(y/LongitudCeldaY));
    }

    if (x>=0){
        ptr_ij[1] = (int) (nCeldas_j/2 + floor(x/LongitudCeldaX));
    }else{
        ptr_ij[1] = (int) (nCeldas_j/2 + floor(x/LongitudCeldaX)-1);
    }
}
