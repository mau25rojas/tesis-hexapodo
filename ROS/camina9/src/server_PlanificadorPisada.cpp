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
#include "camina9/EspacioTrabajoParametros.h"
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
ros::ServiceClient client_EspacioTrabajo;
camina9::EspacioTrabajoParametros srv_EspacioTrabajo;

//-- Variables Globales
bool simulationRunning=true;
bool sensorTrigger=false;
float simulationTime=0.0f;
//-- Log de planificador
FILE *fp1,*fp2,*fp3;
//-- Entrada
int Tripode=0, tripode[Npatas], Tripode1[Npatas/2], Tripode2[Npatas/2], cuentaPasos=0, cuentaErrores[Npatas]={0,0,0,0,0,0},errorPata[Npatas][2];
float velocidadApoyo=0.0, beta=0.0, phi[Npatas], alfa=0.0;
//-- Variables de mapa
camina9::InfoMapa infoMapa;
bool matrizMapa[100][20];
int nCeldas_i=0, nCeldas_j=0;
int ij[2]={0,0}, *p_ij;     //Apuntadores a arreglos de coordenadas e indices
float LongitudCeldaY=0, LongitudCeldaX=0;
Obstaculo obstaculo[100][20];
punto3d EspacioTrabajo[Npatas][Npuntos];
//-- Variables de ubicacion robot
double tiempo_ahora=0.0, tiempo_anterior=0.0;
//float ajuste_Vel=vel_esperada/vel_teorica;
float mod_velocidadCuerpo=0.0;
punto3d posicionActualCuerpo, posicionActualPata[Npatas], posicionActualPataSistemaPata[Npatas];
float teta_CuerpoRobot=0.0;
//-- Envio de señal de stop
camina9::SenalesCambios senales;
//-- Correccion
int correccion_ID[Npatas]; // #ID de la correccion: (-1)no hay corr;(0)corr_izq;(1)corr_der
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
punto3d CorreccioObstaculos(int nPata,punto3d PisadaProxima);
bool Revision_PisadaObstaculos_X (int nPata, punto3d PisadaProxima, segmento3d seg_prueba, float *correccion);
bool Revision_PisadaObstaculos_Y (int nPata, punto3d PisadaProxima, float *correccion, float EDT_LongY);


//-- Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
        simulationTime=info->simulationTime.data;
        simulationRunning=(info->simulatorState.data&1)!=0;
}

void ubicacionRobCallback(camina9::UbicacionRobot msgUbicacionRobot)
{
    posicionActualCuerpo.x = msgUbicacionRobot.coordenadaCuerpo_x;
    posicionActualCuerpo.y = msgUbicacionRobot.coordenadaCuerpo_y;
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
        res.correccion_x.push_back(0);
        res.correccion_y.push_back(0);
    }
//-- Variables locales
    int Tripode_Transferencia[Npatas/2];
    int PisadaProxima_i=0, PisadaProxima_j=0;
    bool cinversaOK;
    float modificacion_lambda[Npatas/2], lambda_posible;
    float T_actual=0.0,lambda_Apoyo_actual=0.0;
    punto3d PisadaProxima, correccion;

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
            res.correccion_ID[Tripode_Transferencia[k]] = -1;
            res.correccion_x[Tripode_Transferencia[k]] = 0.0;
            res.correccion_y[Tripode_Transferencia[k]] = 0.0;
        }
    } else{
        for(int k=0;k<Npatas/2;k++){
            Tripode_Transferencia[k] = Tripode1[k];
            res.correccion_ID[Tripode_Transferencia[k]] = -1;
            res.correccion_x[Tripode_Transferencia[k]] = 0.0;
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
            ROS_WARN("server_PlanificadorPisada::pata[%d] coincidira con obstaculo [%d][%d]",Tripode_Transferencia[k]+1,PisadaProxima_i,PisadaProxima_j);
            fprintf(fp2,"pata[%d] coincidira con obstaculo[%d][%d]\n",Tripode_Transferencia[k]+1,PisadaProxima_i,PisadaProxima_j);
        //-- Revision de pisada para correccion
            correccion = CorreccioObstaculos(Tripode_Transferencia[k],PisadaProxima);

//        //-- Prueba con pata 1
//            if(Tripode_Transferencia[k]==1){
//                res.correccion_ID[Tripode_Transferencia[k]]=correccion_ID[Tripode_Transferencia[k]];
//                res.correccion_x[Tripode_Transferencia[k]]=correccion.x;
//                res.correccion_y[Tripode_Transferencia[k]]=correccion.y;
//                lambda_posible = lambda_maximo-correccion.y;
//                ROS_WARN("server_PlanificadorPisada::Correccion[%.4f][%.4f], lambda[%.4f]",correccion.x,correccion.y,lambda_posible);
//            } else {
//                res.correccion_ID[Tripode_Transferencia[k]]=-1;
//                res.correccion_x[Tripode_Transferencia[k]]=0.0;
//                res.correccion_y[Tripode_Transferencia[k]]=0.0;
//                lambda_posible = lambda_maximo;
//            }
//        } else{
            res.correccion_ID[Tripode_Transferencia[k]]=-1;
            res.correccion_x[Tripode_Transferencia[k]]=0.0;
            res.correccion_y[Tripode_Transferencia[k]]=0.0;
            lambda_posible = lambda_maximo;
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
    client_EspacioTrabajo=node.serviceClient<camina9::EspacioTrabajoParametros>("EspacioTrabajo");
    /* Log de planificador */
    fp1 = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina9/datos/RegistroCorridas.txt","a+");
    fp2 = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina9/datos/LogPlanificador.txt","w+");
    fp3 = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina9/datos/LogCorreccion.txt","w+");

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

//-- puntos de EDT para cada pata
    for(int k=0;k<Npatas;k++) {
        srv_EspacioTrabajo.request.Pata = k;
        if (client_EspacioTrabajo.call(srv_EspacioTrabajo)){
        //-- Funciona servicio
            EspacioTrabajo[k][0].x = srv_EspacioTrabajo.response.EspacioTrabajoP1_x;
            EspacioTrabajo[k][0].y = srv_EspacioTrabajo.response.EspacioTrabajoP1_y;
            EspacioTrabajo[k][1].x = srv_EspacioTrabajo.response.EspacioTrabajoP2_x;
            EspacioTrabajo[k][1].y = srv_EspacioTrabajo.response.EspacioTrabajoP2_y;
            EspacioTrabajo[k][2].x = srv_EspacioTrabajo.response.EspacioTrabajoP3_x;
            EspacioTrabajo[k][2].y = srv_EspacioTrabajo.response.EspacioTrabajoP3_y;
            EspacioTrabajo[k][3].x = srv_EspacioTrabajo.response.EspacioTrabajoP4_x;
            EspacioTrabajo[k][3].y = srv_EspacioTrabajo.response.EspacioTrabajoP4_y;
        } else {
            ROS_ERROR("server_PlanificadorPisada: servicio de EDT no funciona, Pata[%d]\n",k+1);
//            return (false);
        }
    }

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
            ROS_ERROR("server_PlanificadorPisada::servicio de Cinversa no funciona\n");
            return (false);
        }
}

punto3d CorreccioObstaculos(int nPata,punto3d PisadaProxima){

    bool PisadaOk=false;
    float correccion_x=0.0, correccion_y=0.0;
    punto3d Pata, aux_PisadaProxima, EDT[4], P_aux, correccion;
    recta3d recta_borde;
    segmento3d seg_prueba;

    correccion.x = 0.0;
    correccion.y = 0.0;

//-- transformacion del EDT hacia las coordenadas del mundo (por ubicacion de robot)
    for(int k=0;k<Npuntos;k++) {
        EDT[k] = TransformacionHomogenea(EspacioTrabajo[nPata][k],posicionActualCuerpo,teta_CuerpoRobot);
//        ROS_INFO("server_PlanificadorPisada::EDT[%d]:%.3f,%.3f",k,EspacioTrabajo[nPata][k].x,EspacioTrabajo[nPata][k].y);
//        ROS_INFO("server_PlanificadorPisada::TransformadoEDT[%d]:%.3f,%.3f",k,EDT[k].x,EDT[k].y);
    }
    for(int k=0;k<Npuntos;k++) fprintf(fp3,"%.3f\t%.3f\t",EspacioTrabajo[nPata][k].x,EspacioTrabajo[nPata][k].y);
    for(int k=0;k<Npuntos;k++) fprintf(fp3,"%.3f\t%.3f\t",EDT[k].x,EDT[k].y);
//-- creo recta del espacio de trabajo para comparar
    if((nPata+1)==Pata1 or (nPata+1)==Pata3 or (nPata+1)==Pata5){
    //-- Pata del lado izquierdo -- reviso hacia la izquierda
        recta_borde = recta3d(EDT[0],EDT[3]);
        correccion_ID[nPata]=Correccion_menosX;
    } else{
    //-- Pata del lado derecho -- reviso hacia la derecha
        recta_borde = recta3d(EDT[1],EDT[2]);
        correccion_ID[nPata]=Correccion_masX;
    }
    P_aux = recta_borde.proyeccion(PisadaProxima);
    seg_prueba = segmento3d(PisadaProxima,P_aux);
    ROS_INFO("server_PlanificadorPisada::SegPuerba[ini:%.3f,%.3f][fin:%.3f,%.3f]",seg_prueba.ini.x,seg_prueba.ini.y,seg_prueba.fin.x,seg_prueba.fin.y);
    fprintf(fp3,"%.3f\t%.3f\t%.3f\t%.3f\t",seg_prueba.ini.x,seg_prueba.ini.y,seg_prueba.fin.x,seg_prueba.fin.y);
///--- Revision estandar
//-- Revision de pisada hacia ESTE LADO
    ROS_WARN("server_PlanificadorPisada::Pata[%d] correccion lado",nPata+1);
    PisadaOk = Revision_PisadaObstaculos_X(nPata,PisadaProxima,seg_prueba,&correccion_x);

/////--- Si la pisada corregida no esta bien sigo revisando
//    if(!PisadaOk){
//    //-- AHORA REVISO EL OTRO LADO
//    ROS_WARN("server_PlanificadorPisada::Pata[%d] correccion OTRO lado",nPata+1);
//        if((nPata+1)==Pata1 or (nPata+1)==Pata3 or (nPata+1)==Pata5){
//        //-- Pata del lado izquierdo -- reviso hacia la derecha
//            recta_borde = recta3d(EDT[1],EDT[2]);
//            correccion_ID[nPata]=Correccion_masX;
//        } else{
//        //-- Pata del lado derecho -- reviso hacia la izquierda
//            recta_borde = recta3d(EDT[0],EDT[3]);
//            correccion_ID[nPata]=Correccion_menosX;
//        }
//        P_aux = recta_borde.proyeccion(PisadaProxima);
//        seg_prueba = segmento3d(PisadaProxima,P_aux);
//    //-- Revision de pisada hacia este lado
//        PisadaOk = Revision_PisadaObstaculos_X(nPata,PisadaProxima,seg_prueba,&correccion_x);
//    }
//
/////--- Si la pisada corregida no esta bien sigo revisando
//    if(!PisadaOk){
//    //-- AHORA REVISO HACIA ABAJO
//        ROS_WARN("server_PlanificadorPisada::Pata[%d] correccion abajo",nPata+1);
//        correccion_ID[nPata]=-1;
//        float EDT_LongY; segmento3d aux_seg;
//        aux_seg = segmento3d(EDT[1],EDT[2]);
//        EDT_LongY = aux_seg.longitud();
//        PisadaOk = Revision_PisadaObstaculos_Y(nPata,PisadaProxima,&correccion_y,EDT_LongY);
//    }
//
//    if(PisadaOk){
//        ROS_WARN("server_PlanificadorPisada::Pisada Pata[%d] Ok", nPata+1);
//        correccion.x = correccion_x;
//        correccion.y = correccion_y;
//    } else {
//        ROS_ERROR("server_PlanificadorPisada::NO SE PUDO CONSEGUIR CORRECCION PARA PISADA, PATA[%d]", nPata);
//        correccion_ID[nPata]=-1;
//        correccion.x = 0.0;
//        correccion.y = 0.0;
//        return correccion;
//    }
    fprintf(fp3,"\n");
    return(correccion);
}


bool Revision_PisadaObstaculos_X (int nPata, punto3d PisadaProxima, segmento3d seg_prueba, float *correccion){
    bool PisadaOk=false, interseccion=false;
    int PisadaProxima_i=0, PisadaProxima_j=0;
    punto3d P_interseccion, aux_PisadaProxima, puntosObstaculo[4];
    segmento3d segObstaculos[4];

    transformacion_yxTOij(p_ij, PisadaProxima.y, PisadaProxima.x);
    PisadaProxima_i=ij[0];
    PisadaProxima_j=ij[1];
//    while(!PisadaOk){
    //-- puntos de obstaculo
        puntosObstaculo[0].x=obstaculo[PisadaProxima_i][PisadaProxima_j].P1.x;
        puntosObstaculo[0].y=obstaculo[PisadaProxima_i][PisadaProxima_j].P1.y;
        fprintf(fp3,"%.3f\t%.3f\t",puntosObstaculo[0].x,puntosObstaculo[0].y);
        puntosObstaculo[1].x=obstaculo[PisadaProxima_i][PisadaProxima_j].P2.x;
        puntosObstaculo[1].y=obstaculo[PisadaProxima_i][PisadaProxima_j].P2.y;
        fprintf(fp3,"%.3f\t%.3f\t",puntosObstaculo[1].x,puntosObstaculo[1].y);
        puntosObstaculo[2].x=obstaculo[PisadaProxima_i][PisadaProxima_j].P3.x;
        puntosObstaculo[2].y=obstaculo[PisadaProxima_i][PisadaProxima_j].P3.y;
        fprintf(fp3,"%.3f\t%.3f\t",puntosObstaculo[2].x,puntosObstaculo[2].y);
        puntosObstaculo[3].x=obstaculo[PisadaProxima_i][PisadaProxima_j].P4.x;
        puntosObstaculo[3].y=obstaculo[PisadaProxima_i][PisadaProxima_j].P4.y;
        fprintf(fp3,"%.3f\t%.3f\t",puntosObstaculo[3].x,puntosObstaculo[3].y);
    //-- segmentos de obstaculo
        segObstaculos[0] = segmento3d(puntosObstaculo[0],puntosObstaculo[1]);
        segObstaculos[1] = segmento3d(puntosObstaculo[1],puntosObstaculo[2]);
        segObstaculos[2] = segmento3d(puntosObstaculo[2],puntosObstaculo[3]);
        segObstaculos[3] = segmento3d(puntosObstaculo[3],puntosObstaculo[0]);

        for(int k=0;k<Npuntos;k++){
            interseccion = seg_prueba.interseccion(segObstaculos[k],&P_interseccion);
            if (interseccion) break;
        }
        if (!interseccion){
            ROS_ERROR("server_PlanificadorPisada::No hubo interseccion entre obstaculo[%d][%d] con seg de pata[%d]",PisadaProxima_i, PisadaProxima_j,nPata+1);
            return -1;
        }
        interseccion = false;

        *correccion = P_interseccion.distancia(PisadaProxima);
        if(*correccion > seg_prueba.longitud()){
        //-- la correccion hallada sale del espacio de trabajo
        //-- FINALIZA LA FUNCION SI NO SE HALLA PUNTO ADECUADO
            ROS_WARN("server_PlanificadorPisada::correccion %d sale del EDT", correccion_ID[nPata]);
//            break;
        }
        ROS_WARN("correccion:%.4f",*correccion);
//        aux_PisadaProxima.y = PisadaProxima.y;
//        if(correccion_ID[nPata]==Correccion_menosX){
//            aux_PisadaProxima.x = PisadaProxima.x-(*correccion+delta_correccion);
//        } else {
//            aux_PisadaProxima.x = PisadaProxima.x+(*correccion+delta_correccion);
//        }
////        ROS_WARN("server_PlanificadorPisada::Correccion Pata[%d] aux_PisadaProxima[%d][%d]",nPata+1,PisadaProxima_i,PisadaProxima_j);
//        transformacion_yxTOij(p_ij, aux_PisadaProxima.y, aux_PisadaProxima.x);
//        PisadaProxima_i=ij[0];
//        PisadaProxima_j=ij[1];
////        ROS_WARN("server_PlanificadorPisada::Correccion Pata[%d] PisadaProxima_ij[%d][%d]",nPata+1,PisadaProxima_i,PisadaProxima_j);
//        if(matrizMapa[PisadaProxima_i][PisadaProxima_j]){
//            PisadaOk = false;
//        } else {
//            PisadaOk = true;
//        }
//    }
        return (PisadaOk);
}

bool Revision_PisadaObstaculos_Y(int nPata,punto3d PisadaProxima, float *correccion, float EDT_LongY){
    bool PisadaOk=false;
    int PisadaProxima_i=0, PisadaProxima_j=0;
    punto3d aux_PisadaProxima, puntosObstaculo[4];
    recta3d recta_obstaculo;

    transformacion_yxTOij(p_ij, PisadaProxima.y, PisadaProxima.x);
    PisadaProxima_i=ij[0];
    PisadaProxima_j=ij[1];
    while(!PisadaOk){
    //-- puntos de obstaculo
        puntosObstaculo[0].x=obstaculo[PisadaProxima_i][PisadaProxima_j].P1.x;
        puntosObstaculo[0].y=obstaculo[PisadaProxima_i][PisadaProxima_j].P1.y;
        puntosObstaculo[1].x=obstaculo[PisadaProxima_i][PisadaProxima_j].P2.x;
        puntosObstaculo[1].y=obstaculo[PisadaProxima_i][PisadaProxima_j].P2.y;
        puntosObstaculo[2].x=obstaculo[PisadaProxima_i][PisadaProxima_j].P3.x;
        puntosObstaculo[2].y=obstaculo[PisadaProxima_i][PisadaProxima_j].P3.y;
        puntosObstaculo[3].x=obstaculo[PisadaProxima_i][PisadaProxima_j].P4.x;
        puntosObstaculo[3].y=obstaculo[PisadaProxima_i][PisadaProxima_j].P4.y;
    //    ROS_WARN("Puntos: Pata:%.3f,%.3f; Recta:%.3f,%.3f;%.3f,%.3f",Pata.x,Pata.y,puntosObstaculo[2].x,puntosObstaculo[2].y,puntosObstaculo[3].x,puntosObstaculo[3].y);
        recta_obstaculo = recta3d(puntosObstaculo[3],puntosObstaculo[2]);
        *correccion = recta_obstaculo.distancia(PisadaProxima);

        if(*correccion > EDT_LongY){
        //-- la correccion hallada sale del espacio de trabajo
        //-- FINALIZA LA FUNCION SI NO SE HALLA PUNTO ADECUADO
            ROS_WARN("server_PlanificadorPisada::correccion Y sale del EDT");
            break;
        }

        aux_PisadaProxima.y = PisadaProxima.y-(*correccion+delta_correccion);

        transformacion_yxTOij(p_ij, aux_PisadaProxima.y, aux_PisadaProxima.x);
        PisadaProxima_i=ij[0];
        PisadaProxima_j=ij[1];
        ROS_WARN("server_PlanificadorPisada::Correccion Pata[%d] aux_PisadaProxima[%d][%d]",nPata+1,PisadaProxima_i,PisadaProxima_j);
        if(matrizMapa[PisadaProxima_i][PisadaProxima_j]){
//            ROS_WARN("server_PlanificadorPisada::Correccion Pata[%d] coincidira con obstaculo [%d][%d]",nPata+1,PisadaProxima_i,PisadaProxima_j);
            PisadaOk = false;
        } else {
            PisadaOk = true;
        }
    }
        return (PisadaOk);
}
