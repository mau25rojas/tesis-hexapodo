#include "ros/ros.h"
#include "std_msgs/String.h"
#include "math.h"
#include <algorithm>    // std::sort
//Librerias propias usadas
#include "constantes.hpp"
#include "camina9/v_repConst.h"
#include "../../Convexhull/vector3d.hpp"
#include "../../Convexhull/convexhull.cpp"
#include "../../Convexhull/analisis.cpp"
// Used data structures:
#include "camina9/InfoMapa.h"
#include "camina9/CinversaParametros.h"
#include "camina9/UbicacionRobot.h"
#include "camina9/PlanificadorParametros.h"
#include "camina9/SenalesCambios.h"
// Used API services:
#include "vrep_common/VrepInfo.h"
// Definiciones
#define delta_correccion 0.005
//Clientes y Servicios
ros::ServiceClient client_Cinversa1;
camina9::CinversaParametros srv_Cinversa1;

//-- Variables Globales
bool simulationRunning=true;
bool sensorTrigger=false;
float simulationTime=0.0f;
//-- Log de planificador
FILE *fp1,*fp2;
//-- Entrada
int Tripode=0, tripode[Npatas], Tripode1[Npatas/2], Tripode2[Npatas/2], cuentaPasos=0, cuentaErrores[Npatas]={0,0,0,0,0,0},errorPata[Npatas][2];
float velocidadApoyo=0.0, beta=0.0, phi[Npatas], alfa=0.0;
//-- Variables de mapa
camina9::InfoMapa infoMapa;
bool matrizMapa[100][20];
int nCeldas_i=0, nCeldas_j=0;
float LongitudCeldaY=0, LongitudCeldaX=0;
Obstaculo obstaculo[100][20];
//-- Variables de ubicacion robot
double tiempo_ahora=0.0, tiempo_anterior=0.0;
//float ajuste_Vel=vel_esperada/vel_teorica;
float mod_velocidadCuerpo=0.0;
punto3d posicionActualPata[Npatas], posicionActualPataSistemaPata[Npatas];
float teta_CuerpoRobot=0.0;
//-- Envio de señal de stop
camina9::SenalesCambios senales;
//-- Correccion
int correccion_ID[Npatas]; // #ID de la correccion: (-1)no hay corr;(0)corr_izq;(1)corr_der;(2)corr_aba;
float di_prueba=0.0, correccion_y[Npatas];
float di=0.0;
//-- Publishers
ros::Publisher chatter_pub1;
ros::Publisher chatter_pub2;

//-- Funciones
void transformacion_yxTOij(int *ptr_ij, float y, float x);
void FilePrint_matrizMapa();
void Limpiar_matrizMapa();
int Construye_matrizMapa(std::string fileName);
void print_matrizMapa();
void Info_Obstaculos(std::string fileName, int N_Obstaculos);
punto3d TransformacionHomogenea(punto3d Punto_in, punto3d L_traslacion, float ang_rotacion);
punto3d TransportaPunto(punto3d Punto_in,float L_traslacion, float ang_rotacion);
bool VerificacionCinematica(int Pata, float lambda);


//-- Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
        simulationTime=info->simulationTime.data;
        simulationRunning=(info->simulatorState.data&1)!=0;
}

void ubicacionRobCallback(camina9::UbicacionRobot msgUbicacionRobot)
{
    teta_CuerpoRobot = msgUbicacionRobot.orientacionCuerpo_yaw;
    for(int k=0; k<Npatas;k++) {
        posicionActualPata[k].x = msgUbicacionRobot.coordenadaPata_x[k];
        posicionActualPata[k].y = msgUbicacionRobot.coordenadaPata_y[k];
        posicionActualPata[k].z = msgUbicacionRobot.coordenadaPata_z[k];
        posicionActualPataSistemaPata[k].x = msgUbicacionRobot.coordenadaPataSistemaPata_x[k];
        posicionActualPataSistemaPata[k].y = msgUbicacionRobot.coordenadaPataSistemaPata_y[k];
        posicionActualPataSistemaPata[k].z = msgUbicacionRobot.coordenadaPataSistemaPata_z[k];
    }
}

bool PlanificadorPisada(camina9::PlanificadorParametros::Request  &req,
                        camina9::PlanificadorParametros::Response &res)
{
//    ROS_INFO("Llamado a servicio planificador");
//    fprintf(fp2,"\ntiempo de simulacion: %.3f\t",simulationTime);

//-- Datos para envio de mensajes
    res.modificacion_T = 0.0;
    res.modificacion_lambda = 0.0;
    res.result = 0;
    for(int k=0;k<2*Npatas;k++) {
        res.correccion_ID.push_back(-1);
        res.correccion_y.push_back(0);
        res.correccion_x.push_back(0);
    }
//-- Variables locales
    int Tripode_Transferencia[Npatas/2];
    int PisadaProxima_i=0, PisadaProxima_j=0;
    int ij[2]={0,0}, *p_ij;     //Apuntadores a arreglos de coordenadas e indices
    bool cinversaOK;
    float modificacion_lambda[Npatas/2], lambda_posible;
    float T_actual=0.0,lambda_Apoyo_actual=0.0;
    punto3d PisadaProxima;

    p_ij = ij;    // Inicialización de apuntador
    Tripode=req.Tripode;
//    T_actual = req.T;
    lambda_Apoyo_actual = req.lambda;
    mod_velocidadCuerpo = req.mod_velApoyo;

    cuentaPasos++;
    ROS_INFO("INICIO server_PlanificadorPisada::T[%d]::P[%d] ((((mod_v=%.3f))))",Tripode,cuentaPasos,mod_velocidadCuerpo);
    fprintf(fp2,"\nINICIO T[%d],Paso[%d]\n",Tripode,cuentaPasos);
    fprintf(fp2,"server_PlanificadorPisada::T[%d]: tiempo de simulacion: %.3f ((((mod_v=%.3f))))\n",Tripode,simulationTime,mod_velocidadCuerpo);

    ros::spinOnce();
    if (req.Tripode == T1){
    //-- se intercambian los tripodes
        for(int k=0;k<Npatas/2;k++) {
            Tripode_Transferencia[k] = Tripode2[k];
            correccion_ID[Tripode_Transferencia[k]] = -1;
            res.correccion_y[Tripode_Transferencia[k]] = 0.0;
        }
    } else{
        for(int k=0;k<Npatas/2;k++){
            Tripode_Transferencia[k] = Tripode1[k];
            correccion_ID[Tripode_Transferencia[k]] = -1;
            res.correccion_y[Tripode_Transferencia[k]] = 0.0;
        }
    }
    //-- La correccion del tiempo se hace solo para mantener la velocidad al lambda que llevavas
    res.modificacion_T = T_actual = lambda_Apoyo_actual/velocidadApoyo;

    for(int k=0;k<Npatas/2;k++){
        ros::spinOnce();
        //-- Calculamos proximo movimiento en el sistema mundo
        lambda_posible = lambda_maximo;
        PisadaProxima = TransportaPunto(posicionActualPata[Tripode_Transferencia[k]],lambda_posible+mod_velocidadCuerpo*T_actual,(teta_CuerpoRobot-teta_Offset)+alfa);
        transformacion_yxTOij(p_ij, PisadaProxima.y, PisadaProxima.x);
        PisadaProxima_i=ij[0];
        PisadaProxima_j=ij[1];
        if(matrizMapa[PisadaProxima_i][PisadaProxima_j]){
        //-- La pisada COINCIDE con obstaculo
            infoMapa.correccion=false;
            ROS_WARN("server_PlanificadorPisada: pata [%d] coincidira con obstaculo [%d][%d]",Tripode_Transferencia[k]+1,PisadaProxima_i,PisadaProxima_j);
            fprintf(fp2,"pata [%d] coincidira con obstaculo[%d][%d]\n",Tripode_Transferencia[k]+1,PisadaProxima_i,PisadaProxima_j);
//                lambda_posible = CorreccioObstaculos(Tripode_Transferencia[k],PisadaProxima_y,PisadaProxima_x);
        }
            modificacion_lambda[k] = lambda_posible;
        //-- Pisada maxima
            infoMapa.coordenadaAjuste_i[Tripode_Transferencia[k]] = PisadaProxima_i;
            infoMapa.coordenadaAjuste_j[Tripode_Transferencia[k]] = PisadaProxima_j;

///////////////////CUANDO HACEMOS ESTA VERIFICACION???/////////////////
//        cinversaOK = VerificacionCinematica(Tripode_Transferencia[k],lambda_posible);
//        if (cinversaOK){
//        //-- Todo salio bien! Esta proxima_pisada es valida! :D
//
//        } else {
//            ROS_WARN("server_PlanificadorPisada: pata [%d] error cinversa",k+1);
//            fprintf(fp2,"pata [%d] error cinversa\n",Tripode_Transferencia[k]+1);
//        }
    } // Fin de revision de pisadas

//-- Escojo el largo de pisada mas corto y lo impongo a todas las patas del tripode
    std::sort (modificacion_lambda, modificacion_lambda+3);
    if(modificacion_lambda[0]<lambda_minimo){
        res.modificacion_lambda = lambda_minimo;
    } else {
        res.modificacion_lambda = modificacion_lambda[0];
    }

//-- Envio trayectoria planificada D: chanchanchaaaaaan
    fprintf(fp2,"server_PlanificadorPisada: Tripode=%d, lambda_correccion=%.3f, T_correccion=%.3f\n",req.Tripode,res.modificacion_lambda,res.modificacion_T);

//-- Envio y guardo datos
    for(int k=0;k<Npatas;k++){
        infoMapa.coordenadaPata_x[k] = posicionActualPata[k].x;
        infoMapa.coordenadaPata_y[k] = posicionActualPata[k].y;
        transformacion_yxTOij(p_ij, posicionActualPata[k].y, posicionActualPata[k].x);
        infoMapa.coordenadaPata_i[k] = ij[0];
        infoMapa.coordenadaPata_j[k] = ij[1];

        if(errorPata[k][0]!=ij[0] && errorPata[k][1]!=ij[1]){
            if(matrizMapa[infoMapa.coordenadaPata_i[k]][infoMapa.coordenadaPata_j[k]]){
    //            fprintf(fp2,"---ERROR--- pata[%d] Coincide con obstaculo[%d][%d]\n",k+1,ij[0],ij[1]);
                errorPata[k][0]=ij[0];
                errorPata[k][1]=ij[1];
                ROS_WARN("---ERROR--- pata[%d] Coincide con obstaculo[%d][%d]",k+1,ij[0],ij[1]);
                cuentaErrores[k]++;
            }
        } else {
            infoMapa.coordenadaPata_i[k] = nCeldas_i-1;
            infoMapa.coordenadaPata_j[k] = nCeldas_j-1;
        }
    }
    chatter_pub2.publish(infoMapa);

    return 1;
}

int main(int argc, char **argv)
{
    int Narg=0, cuentaObs=0;
    std::string fileName,M_fileName,O_fileName;

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
    chatter_pub1=node.advertise<camina9::SenalesCambios>("Senal", 100);
    chatter_pub2=node.advertise<camina9::InfoMapa>("Plan", 100);
    ros::Subscriber sub1=node.subscribe("/vrep/info",100,infoCallback);
    ros::Subscriber sub2=node.subscribe("UbicacionRobot",100,ubicacionRobCallback);
//-- Clientes y Servicios
    ros::ServiceServer service = node.advertiseService("PlanificadorPisada", PlanificadorPisada);
    client_Cinversa1=node.serviceClient<camina9::CinversaParametros>("Cinversa");

    /* Log de planificador */
    fp1 = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina9/datos/RegistroCorridas.txt","a+");
    fp2 = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina9/datos/LogPlanificador.txt","w+");

    for(int k=0;k<Npatas;k++) {
        infoMapa.coordenadaPata_x.push_back(0);
        infoMapa.coordenadaPata_y.push_back(0);
        infoMapa.coordenadaPata_i.push_back(0);
        infoMapa.coordenadaPata_j.push_back(0);
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
    ROS_INFO("server_PlanificadorPisada: Tripode1[%d,%d,%d] - Tripode2[%d,%d,%d]",Tripode1[0]+1,Tripode1[1]+1,Tripode1[2]+1,Tripode2[0]+1,Tripode2[1]+1,Tripode2[2]+1);
    for(int i=0;i<Npatas;i++){
        for(int j=0;j<2;j++){
            errorPata[i][j]=0;
        }
    }
    for(int i=0;i<100;i++){
        for(int j=0;j<20;j++){
            obstaculo[i][j].P1.x=-100;  obstaculo[i][j].P1.y=-100;
            obstaculo[i][j].P2.x=-100;  obstaculo[i][j].P2.y=-100;
            obstaculo[i][j].P3.x=-100;  obstaculo[i][j].P3.y=-100;
            obstaculo[i][j].P4.x=-100;  obstaculo[i][j].P4.y=-100;
        }
    }

    Limpiar_matrizMapa();
    M_fileName = O_fileName = fileName;
    std::string txt(".txt");
    std::string obs("_o");
    M_fileName+=txt;
    cuentaObs = Construye_matrizMapa(M_fileName);
    O_fileName+=obs;
    O_fileName+=txt;
    Info_Obstaculos(O_fileName,cuentaObs);
//    print_matrizMapa(nCeldas_i,nCeldas_j);
//    ROS_INFO("Nobstaculos=%d",cuentaObs);
//    ROS_INFO("variables de mapa: Ni=%d,Nj=%d,LY=%.3f,LX=%.3f",nCeldas_i,nCeldas_j,LongitudCeldaY,LongitudCeldaX);

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
        }punto3d TransformacionHomogenea(punto3d Punto_in, punto3d L_traslacion, float ang_rotacion);
punto3d TransportaPunto(punto3d Punto_in,float L_traslacion, float ang_rotacion);

    }
}

int Construye_matrizMapa(std::string fileName){
    //--- Construccion de mapa mediante lectura de archivo
//    printf ("%s \n", fileName.c_str());
    FILE *fp;
    int int_aux=0, i=0, j=0, cuentaObs=0, aux=0;

    fp = fopen(fileName.c_str(), "r");
    if(fp!=NULL){
        printf("\n");
        for(i=0;i<nCeldas_i;i++){
            for(j=0;j<nCeldas_j;j++){
                aux=fscanf(fp, "%d", &int_aux);
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

void Info_Obstaculos(std::string fileName, int N_Obstaculos){
    //--- Construccion de mapa mediante lectura de archivo
//    printf ("%s \n", fileName.c_str());
    FILE *fp;
    int int_aux=0, i=0, j=0, aux;
    float f_aux=0.0;

    fp = fopen(fileName.c_str(), "r");
    if(fp!=NULL){
        for(int k=0;k<N_Obstaculos;k++){
            aux=fscanf(fp, "%d", &int_aux);
            i=int_aux;
            aux=fscanf(fp, "%d", &int_aux);
            j=int_aux;
            aux=fscanf(fp, "%f", &f_aux);
            obstaculo[i][j].O.x=f_aux;
            aux=fscanf(fp, "%f", &f_aux);
            obstaculo[i][j].O.y=f_aux;
            aux=fscanf(fp, "%f", &f_aux);
            obstaculo[i][j].P1.x=f_aux;
            aux=fscanf(fp, "%f", &f_aux);
            obstaculo[i][j].P1.y=f_aux;
            aux=fscanf(fp, "%f", &f_aux);
            obstaculo[i][j].P2.x=f_aux;
            aux=fscanf(fp, "%f", &f_aux);
            obstaculo[i][j].P2.y=f_aux;
            aux=fscanf(fp, "%f", &f_aux);
            obstaculo[i][j].P3.x=f_aux;
            aux=fscanf(fp, "%f", &f_aux);
            obstaculo[i][j].P3.y=f_aux;
            aux=fscanf(fp, "%f", &f_aux);
            obstaculo[i][j].P4.x=f_aux;
            aux=fscanf(fp, "%f", &f_aux);
            obstaculo[i][j].P4.y=f_aux;
        }
    } else{
        ROS_ERROR("Error al leer archivo %s",fileName.c_str());
    }
    fclose(fp);
}


punto3d TransportaPunto(punto3d Punto_in,float L_traslacion, float ang_rotacion){

    punto3d Punto_out;

    Punto_out.x=Punto_in.x - L_traslacion*sin(ang_rotacion);
    Punto_out.y=Punto_in.y + L_traslacion*cos(ang_rotacion);

    return(Punto_out);
}

punto3d TransformacionHomogenea(punto3d Punto_in, punto3d L_traslacion, float ang_rotacion){

    punto3d Punto_out;

    Punto_out.x = L_traslacion.x + Punto_in.x*cos(ang_rotacion) - Punto_in.y*sin(ang_rotacion);
    Punto_out.y = L_traslacion.y + Punto_in.x*sin(ang_rotacion) + Punto_in.y*cos(ang_rotacion);
    Punto_out.z = L_traslacion.z + Punto_in.z;

    return(Punto_out);
}

bool VerificacionCinematica(int Pata, float lambda){
        punto3d O, PisadaProxima, delta_S0, delta_S1;
        O.x=0.0; O.y=0.0, O.z=0.0;
//-- Calculamos proximo movimiento en el sistema de pata
        delta_S0.x = -lambda*cos(alfa);
        delta_S0.y = lambda*sin(alfa);
        delta_S1 = TransformacionHomogenea(delta_S0,O,phi[Pata]+alfa);
        PisadaProxima.x = posicionActualPataSistemaPata[Pata].x + delta_S1.x;
        PisadaProxima.y = posicionActualPataSistemaPata[Pata].y + delta_S1.y;
        PisadaProxima.z = posicionActualPataSistemaPata[Pata].z;
    //-- Verificamos que pisada sea factible
//        ROS_INFO("server_Plan: revisando cinversa");
        srv_Cinversa1.request.x = PisadaProxima.x;
        srv_Cinversa1.request.y = PisadaProxima.y;
        srv_Cinversa1.request.z = PisadaProxima.z;
        if (client_Cinversa1.call(srv_Cinversa1)){
        //-- Funciona servicio
            return (true);
        } else {
            ROS_ERROR("server_PlanificadorPisada: servicio de Cinversa no funciona\n");
            return (false);
        }
}

//void CorreccioObstaculos(int Pata, float PisadaProxima_y, float PisadaProxima_x){
//    punto3d Pata, puntosObstaculo[4];
//    recta3d recta_inf_o;
//    float correccion=0.0;
//
//    Pata.x = PisadaProxima.x;
//    Pata.y = PisadaProxima.y;
//    puntosObstaculo[0].x=obstaculo[PisadaProxima_i][PisadaProxima_j].P1.x;
//    puntosObstaculo[0].y=obstaculo[PisadaProxima_i][PisadaProxima_j].P1.y;
//    puntosObstaculo[1].x=obstaculo[PisadaProxima_i][PisadaProxima_j].P2.x;
//    puntosObstaculo[1].y=obstaculo[PisadaProxima_i][PisadaProxima_j].P2.y;
//    puntosObstaculo[2].x=obstaculo[PisadaProxima_i][PisadaProxima_j].P3.x;
//    puntosObstaculo[2].y=obstaculo[PisadaProxima_i][PisadaProxima_j].P3.y;
//    puntosObstaculo[3].x=obstaculo[PisadaProxima_i][PisadaProxima_j].P4.x;
//    puntosObstaculo[3].y=obstaculo[PisadaProxima_i][PisadaProxima_j].P4.y;
//    ROS_WARN("Puntos: Pata:%.3f,%.3f; Recta:%.3f,%.3f;%.3f,%.3f",Pata.x,Pata.y,puntosObstaculo[2].x,puntosObstaculo[2].y,puntosObstaculo[3].x,puntosObstaculo[3].y);
//    recta_inf_o = recta3d(puntosObstaculo[3],puntosObstaculo[2]);
//
//    correccion = recta_inf_o.distancia(Pata);
//    ROS_WARN("correccion:%.3f",correccion);
//    res.correccion_y[Tripode_Transferencia[k]] = correccion;
//    modificacion_lambda[k] = lambda_maximo-correccion-delta_correccion;
//}
