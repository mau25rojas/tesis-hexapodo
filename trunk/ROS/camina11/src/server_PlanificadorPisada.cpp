#include "ros/ros.h"
#include "std_msgs/String.h"
#include <math.h>
#include <algorithm>    // std::sort
//Librerias propias usadas
#include "camina11/v_repConst.h"
#include "camina11/constantes.hpp"
#include "camina11/vector3d.hpp"
#include "camina11/analisis.hpp"
#include "camina11/convexhull.hpp"
// Used data structures:
#include "camina11/InfoMapa.h"
#include "camina11/UbicacionRobot.h"
#include "camina11/PlanificadorParametros.h"
// Used API services:
#include "vrep_common/VrepInfo.h"
// Definiciones
#define delta_correccion_x 0.008
#define delta_correccion_y 0.01

//-- Variables Globales
bool simulationRunning=true;
bool sensorTrigger=false;
float simulationTime=0.0f;
//-- Log de planificador
FILE *fp1,*fp2,*fp3,*fp4;
//-- Entrada
int cuentaPasos=0, cuentaErrores[Npatas]={0,0,0,0,0,0},errorPata[Npatas][2];
float velocidadApoyo=0.0, beta=0.0, phi[Npatas], alfa=0.0;
//-- Variables de mapa
camina11::InfoMapa infoMapa;
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
//-- Correccion
bool ExisteCorreccion=true;
int correccion_ID; // #ID de la correccion: (-1)no hay corr;(0)corr_izq;(1)corr_der
float correccion_x, correccion_y;
//-- Publishers
ros::Publisher chatter_pub2;
//-- Funciones
void transformacion_yxTOij(int *ptr_ij, float y, float x);
void FilePrint_matrizMapa();
void Limpiar_matrizMapa();
int Construye_matrizMapa(std::string fileName);
void print_matrizMapa();
void Info_Obstaculos(std::string fileName, int N_Obstaculos);
//bool VerificacionCinematica(int Pata, float lambda);
punto3d CorreccionObstaculos(int nPata,punto3d PisadaProxima,float transferenciaActual);
bool Revision_PisadaObstaculos_X (int nPata, punto3d PisadaProxima, segmento3d seg_prueba, float *correccion);
bool Revision_PisadaObstaculos_Y (int nPata, punto3d PisadaProxima, float *correccion, float EDT_LongY);

//-- Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
        simulationTime=info->simulationTime.data;
        simulationRunning=(info->simulatorState.data&1)!=0;
}

void ubicacionRobCallback(camina11::UbicacionRobot msgUbicacionRobot)
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

bool PlanificadorPisada(camina11::PlanificadorParametros::Request  &req,
                        camina11::PlanificadorParametros::Response &res)
{
    int nPata=req.nPata;
//-- Datos para envio de mensajes
    res.modificacion_lambda = 0.0;
    res.resultado = 1;
    res.correccion_ID=-1;
    res.correccion_x=0;
    res.correccion_y=0;
//-- Variables locales
    int PisadaProxima_i=0, PisadaProxima_j=0;
    float lambda_posible; //lambda_posible: lo que se va a trasladar la pata en transferencia
    float T_transfer=0.0,lambda_Apoyo_actual=0.0;
    punto3d PisadaProxima, correccion;
    p_ij = ij;    // Inicialización de apuntador
//    lambda_Apoyo_actual = req.lambda;   //lambda_Apoyo_actual: lo que se esta desplazando el robot en apoyo
    mod_velocidadCuerpo = req.mod_velApoyo;

    cuentaPasos++;
//    ROS_INFO("INICIO server_PlanificadorPisada::P[%d] ((((mod_v=%.3f))))",nPata+1,cuentaPasos,mod_velocidadCuerpo);
//    fprintf(fp2,"\nINICIO T[%d],Paso[%d]\n",Tripode,cuentaPasos);
//    fprintf(fp2,"server_PlanificadorPisada::T[%d]: tiempo de simulacion: %.3f ((((mod_v=%.3f))))\n",Tripode,simulationTime,mod_velocidadCuerpo);
    infoMapa.correccion=false;
    ros::spinOnce();

    punto3d Pata, puntosObstaculo[4], *Q;
    recta3d recta_di[2], *S;
    float di=0.0;
    transformacion_yxTOij(p_ij, posicionActualPata[nPata].y, posicionActualPata[nPata].x);
    ROS_INFO("server_PlanificadorPisada::pata[%d] CAYO en [x=%.3f;y=%.3f]",nPata+1,posicionActualPata[nPata].x,posicionActualPata[nPata].y);
    PisadaProxima_i=ij[0];
    PisadaProxima_j=ij[1];
    if(matrizMapa[PisadaProxima_i][PisadaProxima_j]){
        puntosObstaculo[0].x=obstaculo[PisadaProxima_i][PisadaProxima_j].P1.x;
        puntosObstaculo[0].y=obstaculo[PisadaProxima_i][PisadaProxima_j].P1.y;
        puntosObstaculo[1].x=obstaculo[PisadaProxima_i][PisadaProxima_j].P2.x;
        puntosObstaculo[1].y=obstaculo[PisadaProxima_i][PisadaProxima_j].P2.y;
        puntosObstaculo[2].x=obstaculo[PisadaProxima_i][PisadaProxima_j].P3.x;
        puntosObstaculo[2].y=obstaculo[PisadaProxima_i][PisadaProxima_j].P3.y;
        puntosObstaculo[3].x=obstaculo[PisadaProxima_i][PisadaProxima_j].P4.x;
        puntosObstaculo[3].y=obstaculo[PisadaProxima_i][PisadaProxima_j].P4.y;
        Q = puntosObstaculo;

        S = recta_di;
        Pata.x=posicionActualPata[nPata].x; Pata.y=posicionActualPata[nPata].y;
        di = margen_est (Pata,Q,4,S);
//            ROS_WARN("Pata[%d]:%.3f,%.3f; obstaculo:%.3f,%.3f",k+1,Pata.x,Pata.y,obstaculo[PisadaProxima_i[k]][PisadaProxima_j[k]].O_x,obstaculo[PisadaProxima_i[k]][PisadaProxima_j[k]].O_y);
//            ROS_WARN("P1:%.3f,%.3f;P2:%.3f,%.3f,P3:%.3f,%.3f,P4:%.3f,%.3f",puntosObstaculo[0].x,puntosObstaculo[0].y,puntosObstaculo[1].x,puntosObstaculo[1].y,puntosObstaculo[2].x,puntosObstaculo[2].y,puntosObstaculo[3].x,puntosObstaculo[3].y);
        fprintf(fp2,"**ERROR** Pata[%d]:%.3f,%.3f->Coincide Obstaculo [%d][%d]; distancia_min:%.3f\n",nPata+1,Pata.x,Pata.y,PisadaProxima_i,PisadaProxima_j,di);
        ROS_ERROR("Mapa::paso[%d]:Pata[%d]:%.3f,%.3f->coincide con obstaculo [%d][%d]; di=%.4f",cuentaPasos,nPata+1,Pata.x,Pata.y,PisadaProxima_i,PisadaProxima_j,di);
        fprintf(fp4,"%d\t%.5f\n",nPata+1,di);
        cuentaErrores[nPata]++;
    }
//    //-- La correccion del tiempo se hace solo para mantener la velocidad al lambda que llevavas
//    res.modificacion_T_apoyo = T_transfer = lambda_Apoyo_actual/velocidadApoyo;
    T_transfer = req.T;
    //-- Calculamos proximo movimiento en el sistema mundo
    lambda_posible = lambda_maximo;
    //-- Hay que eliminar la correccion de la estimacion, porque se supone la pata vuelve a su estado default
    punto3d posicionPata = posicionActualPata[nPata];
    posicionPata.x = posicionActualPata[nPata].x - req.correccion_x;
    PisadaProxima = TransportaPunto(posicionPata,lambda_posible+mod_velocidadCuerpo*T_transfer,(teta_CuerpoRobot-teta_Offset)+alfa);
    transformacion_yxTOij(p_ij, PisadaProxima.y, PisadaProxima.x);
    PisadaProxima_i=ij[0];
    PisadaProxima_j=ij[1];
    ROS_INFO("server_PlanificadorPisada::pata[%d] va a caer en [x=%.3f;y=%.3f]",nPata+1,PisadaProxima.x,PisadaProxima.y);
    if(0<PisadaProxima_i<nCeldas_i and 0<PisadaProxima_j<nCeldas_j){

        if(matrizMapa[PisadaProxima_i][PisadaProxima_j]){
        //-- La pisada COINCIDE con obstaculo
            infoMapa.correccion=true;
            ROS_WARN("server_PlanificadorPisada::pata[%d] coincidira con obstaculo [%d][%d]",nPata+1,PisadaProxima_i,PisadaProxima_j);
            fprintf(fp2,"pata[%d] coincidira con obstaculo[%d][%d]\n",nPata+1,PisadaProxima_i,PisadaProxima_j);
        //-- Revision de pisada para correccion
            correccion = CorreccionObstaculos(nPata,PisadaProxima,mod_velocidadCuerpo*T_transfer);
            if(!ExisteCorreccion){
                res.resultado=-1;
                return -1;
            }
        //-- Prueba con pata 1
    //            if(Tripode_Transferencia[k]==0){
                correccion_x=correccion.x;
                correccion_y=correccion.y;
                lambda_posible = lambda_maximo-correccion.y;
                ROS_WARN("server_PlanificadorPisada::Pata[%d]Correccion[%.4f][%.4f], lambda[%.4f]",nPata+1,correccion.x,correccion.y,lambda_posible);
    //            } else {
    //                res.correccion_ID[Tripode_Transferencia[k]]=-1;
    //                res.correccion_x[Tripode_Transferencia[k]]=0.0;
    //                res.correccion_y[Tripode_Transferencia[k]]=0.0;
    //                lambda_posible = lambda_maximo;
    //            }
        } else {
            correccion_ID=-1;
            correccion_x=0.0;
            correccion_y=0.0;
            lambda_posible = lambda_maximo;
        }
    } else {
    //-- Out of bondary
        res.resultado=-1;
        return -1;
    }
//-- datos enviados - correccion
    infoMapa.nPata = nPata;
    infoMapa.coordenadaAjuste_i = PisadaProxima_i;
    infoMapa.coordenadaAjuste_j = PisadaProxima_j;
    infoMapa.coordenadaPata_x = posicionActualPata[nPata].x;
    infoMapa.coordenadaPata_y = posicionActualPata[nPata].y;
    transformacion_yxTOij(p_ij, posicionActualPata[nPata].y, posicionActualPata[nPata].x);
    infoMapa.coordenadaPata_i = ij[0];
    infoMapa.coordenadaPata_j = ij[1];
    res.modificacion_lambda = lambda_posible;
    res.correccion_ID = correccion_ID;
    res.correccion_x = correccion_x;
    res.correccion_y = correccion_y;

    fprintf(fp2,"server_PlanificadorPisada::Pata[%d]Correccion[%.4f][%.4f], lambda[%.4f]\n",nPata+1,correccion.x,correccion.y,lambda_posible);

    chatter_pub2.publish(infoMapa);
    res.resultado=1;
    return 1;
}

int main(int argc, char **argv)
{
    int Narg=0, cuentaObs=0;
    punto3d Offset;
    std::string fileName,M_fileName,O_fileName;

    Narg=17;
    if (argc>=Narg){
        beta = atof(argv[1]);
        velocidadApoyo = atof(argv[2]);
        alfa = atof(argv[3])*pi/180.0;
        fileName = argv[4];
        nCeldas_i = atoi(argv[5]);
        nCeldas_j = atoi(argv[6]);
        LongitudCeldaY = atof(argv[7]);
        LongitudCeldaX = atof(argv[8]);
        Offset.x = atof(argv[9]);
        Offset.y = atof(argv[10]);
        Offset.z = atof(argv[11]);
        for(int k=0;k<Npatas;k++) phi[k] = atof(argv[12+k])*pi/180.0;
    } else{
        ROS_ERROR("server_PlanificadorPisada: Indique argumentos completos!\n");
        return (0);
    }

    /*Inicio nodo de ROS*/
    ros::init(argc, argv, "server_PlanificadorPisada");
    ros::NodeHandle node;
    ROS_INFO("server_PlanificadorPisada just started");

//-- Topicos susbcritos y publicados
    ros::Subscriber sub1=node.subscribe("/vrep/info",100,infoCallback);
    ros::Subscriber sub2=node.subscribe("UbicacionRobot",100,ubicacionRobCallback);
    chatter_pub2=node.advertise<camina11::InfoMapa>("Plan", 100);
//-- Clientes y Servicios
    ros::ServiceServer service = node.advertiseService("PlanificadorPisada", PlanificadorPisada);
    /* Log de planificador */
    fp1 = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina11/datos/RegistroCorridas.txt","a+");
    fp2 = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina11/datos/LogPlanificador.txt","w+");
    fp3 = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina11/datos/LogCorreccion.txt","w+");
    fp4 = fopen("../fuerte_workspace/sandbox/TesisMaureen/ROS/camina11/datos/Errores.txt","a+");

    for(int i=0;i<Npatas;i++){
        for(int j=0;j<2;j++){
            errorPata[i][j]=0;
        }
    }
    for(int i=0;i<nCeldas_i;i++){
        for(int j=0;j<nCeldas_j;j++){
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
    punto3d aux_puntos[Npuntos];
    for(int k=0;k<Npatas;k++) {
        EspacioDeTrabajo_Robot(k,aux_puntos,phi[k],Offset);
        EspacioTrabajo[k][0] = aux_puntos[0];
        EspacioTrabajo[k][1] = aux_puntos[1];
        EspacioTrabajo[k][2] = aux_puntos[2];
        EspacioTrabajo[k][3] = aux_puntos[3];
    }

    while (ros::ok() && simulationRunning){
        ros::spinOnce();
    }
    fprintf(fp1,"%d\t%d\t",cuentaObs,cuentaPasos);
    for(int k=0;k<Npatas;k++) fprintf(fp1,"%d\t",cuentaErrores[k]);
    fprintf(fp1,"\n");
    fclose(fp1);fclose(fp2);fclose(fp3);fclose(fp4);
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

//bool VerificacionCinematica(int Pata, float lambda){
//        punto3d O, PisadaProxima, delta_S0, delta_S1;
//        O.x=0.0; O.y=0.0, O.z=0.0;
////-- Calculamos proximo movimiento en el sistema de pata
//        delta_S0.x = -lambda*cos(alfa);
//        delta_S0.y = lambda*sin(alfa);
//        delta_S1 = TransformacionHomogenea(delta_S0,O,phi[Pata]+alfa);
//        PisadaProxima.x = posicionActualPataSistemaPata[Pata].x + delta_S1.x;
//        PisadaProxima.y = posicionActualPataSistemaPata[Pata].y + delta_S1.y;
//        PisadaProxima.z = posicionActualPataSistemaPata[Pata].z;
//    //-- Verificamos que pisada sea factible
////        ROS_INFO("server_Plan: revisando cinversa");
////        srv_Cinversa1.request.x = PisadaProxima.x;
////        srv_Cinversa1.request.y = PisadaProxima.y;
////        srv_Cinversa1.request.z = PisadaProxima.z;
////        if (client_Cinversa1.call(srv_Cinversa1)){
////        //-- Funciona servicio
////            return (true);
////        } else {
////            ROS_ERROR("server_PlanificadorPisada::servicio de Cinversa no funciona\n");
////            return (false);
////        }
//}

punto3d CorreccionObstaculos(int nPata,punto3d PisadaProxima,float transferenciaActual){

    bool PisadaOk=false;
    float correccionX=0.0, correccionY=0.0;
    punto3d Pata, aux_PisadaProxima, aux_posicionActualCuerpo, EDT[4], P_aux, correccion;
    recta3d recta_borde;
    segmento3d seg_prueba;

    correccion.x = 0.0;
    correccion.y = 0.0;

//-- transformacion del EDT hacia las coordenadas del mundo (por ubicacion de robot), incluyendo la traslacion en curso
    aux_posicionActualCuerpo = TransportaPunto(posicionActualCuerpo,transferenciaActual,(teta_CuerpoRobot-teta_Offset)+alfa);
    fprintf(fp3,"%d\t",0);
    fprintf(fp3,"%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n",posicionActualCuerpo.x,posicionActualCuerpo.y,aux_posicionActualCuerpo.x,aux_posicionActualCuerpo.y,posicionActualPata[nPata].x,posicionActualPata[nPata].y,0.0,0.0);
    for(int k=0;k<Npuntos;k++) {
        EDT[k] = TransformacionHomogenea(EspacioTrabajo[nPata][k],aux_posicionActualCuerpo,teta_CuerpoRobot);
//        ROS_INFO("server_PlanificadorPisada::EDT[%d]:%.3f,%.3f",k,EspacioTrabajo[nPata][k].x,EspacioTrabajo[nPata][k].y);
//        ROS_INFO("server_PlanificadorPisada::TransformadoEDT[%d]:%.3f,%.3f",k,EDT[k].x,EDT[k].y);
    }
    fprintf(fp3,"%d\t",1);
    for(int k=0;k<Npuntos;k++) fprintf(fp3,"%.3f\t%.3f\t",EspacioTrabajo[nPata][k].x,EspacioTrabajo[nPata][k].y);
    fprintf(fp3,"\n");
    fprintf(fp3,"%d\t",2);
    for(int k=0;k<Npuntos;k++) fprintf(fp3,"%.3f\t%.3f\t",EDT[k].x,EDT[k].y);
    fprintf(fp3,"\n");
//-- creo recta del espacio de trabajo para comparar
    if((nPata+1)==Pata1 or (nPata+1)==Pata3 or (nPata+1)==Pata5){
    //-- Pata del lado izquierdo -- reviso hacia la izquierda
        recta_borde = recta3d(EDT[0],EDT[3]);
        correccion_ID=Correccion_menosX;
    } else{
    //-- Pata del lado derecho -- reviso hacia la derecha
        recta_borde = recta3d(EDT[1],EDT[2]);
        correccion_ID=Correccion_masX;
    }
    P_aux = recta_borde.proyeccion(PisadaProxima);
    seg_prueba = segmento3d(PisadaProxima,P_aux);
//    ROS_INFO("server_PlanificadorPisada::SegPuerba[ini:%.3f,%.3f][fin:%.3f,%.3f]",seg_prueba.ini.x,seg_prueba.ini.y,seg_prueba.fin.x,seg_prueba.fin.y);
    fprintf(fp3,"%d\t",3);
    fprintf(fp3,"%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t",seg_prueba.ini.x,seg_prueba.ini.y,seg_prueba.fin.x,seg_prueba.fin.y,0.0,0.0,0.0,0.0);
    fprintf(fp3,"\n");
///--- Revision estandar
//-- Revision de pisada hacia ESTE LADO
    ROS_WARN("server_PlanificadorPisada::Pata[%d] correccion LADO",nPata+1);
    PisadaOk = Revision_PisadaObstaculos_X(nPata,PisadaProxima,seg_prueba,&correccionX);
//    ROS_WARN("PisadaOk:%s", PisadaOk ? "true" : "false");

///--- Si la pisada corregida no esta bien sigo revisando
    if(!PisadaOk){
    //-- AHORA REVISO EL OTRO LADO
    ROS_WARN("server_PlanificadorPisada::Pata[%d] correccion OTRO LADO",nPata+1);
        if((nPata+1)==Pata1 or (nPata+1)==Pata3 or (nPata+1)==Pata5){
        //-- Pata del lado izquierdo -- reviso hacia la derecha
            recta_borde = recta3d(EDT[1],EDT[2]);
            correccion_ID=Correccion_masX;
        } else{
        //-- Pata del lado derecho -- reviso hacia la izquierda
            recta_borde = recta3d(EDT[0],EDT[3]);
            correccion_ID=Correccion_menosX;
        }
        P_aux = recta_borde.proyeccion(PisadaProxima);
        seg_prueba = segmento3d(PisadaProxima,P_aux);
        fprintf(fp3,"%d\t",3);
        fprintf(fp3,"%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t",seg_prueba.ini.x,seg_prueba.ini.y,seg_prueba.fin.x,seg_prueba.fin.y,0.0,0.0,0.0,0.0);
        fprintf(fp3,"\n");
    //-- Revision de pisada hacia este lado
        PisadaOk = Revision_PisadaObstaculos_X(nPata,PisadaProxima,seg_prueba,&correccionX);
//        ROS_WARN("PisadaOk:%s", PisadaOk ? "true" : "false");
    }
///--- Si la pisada corregida no esta bien sigo revisando
    if(!PisadaOk){
    //-- La correccion en X no funcionó
        correccionX = 0.0;
    //-- AHORA REVISO HACIA ABAJO
        ROS_WARN("server_PlanificadorPisada::Pata[%d] correccion ABAJO",nPata+1);
        correccion_ID=-1;
        float EDT_LongY; segmento3d aux_seg;
        aux_seg = segmento3d(EDT[1],EDT[2]);
        EDT_LongY = aux_seg.longitud();
        PisadaOk = Revision_PisadaObstaculos_Y(nPata,PisadaProxima,&correccionY,EDT_LongY);
//        ROS_WARN("PisadaOk:%s", PisadaOk ? "true" : "false");
    }
///--- Correccion final o mensaje de error
    if(PisadaOk){
//        ROS_WARN("server_PlanificadorPisada::Pisada Pata[%d] Ok: correccionX=%.3f;correccionY=%.3f", nPata+1,correccion_x,correccion_y);
        correccion.x = correccionX;
        correccion.y = correccionY;
    } else {
        ROS_ERROR("server_PlanificadorPisada::NO SE PUDO CONSEGUIR CORRECCION PARA PISADA, PATA[%d]", nPata);
        correccion_ID=-1;
        correccion.x = 0.0;
        correccion.y = 0.0;
        ExisteCorreccion = false;
        return correccion;
    }
    fprintf(fp3,"\n");
    return(correccion);
}


bool Revision_PisadaObstaculos_X (int nPata, punto3d PisadaProxima, segmento3d seg_prueba, float *correccion){
    bool PisadaOk=false, interseccion=false;
    int PisadaProxima_i=0, PisadaProxima_j=0, Parada=0;
    punto3d P_interseccion, prev_P_interseccion, aux_PisadaProxima, puntosObstaculo[4];
    segmento3d segObstaculos[4];

    transformacion_yxTOij(p_ij, PisadaProxima.y, PisadaProxima.x);
    PisadaProxima_i=ij[0];
    PisadaProxima_j=ij[1];
//    for(int k=0;k<Npuntos;k++){
    while(!PisadaOk and Parada<100){
        Parada++;
    //-- puntos de obstaculo
        fprintf(fp3,"%d\t",4);
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
        fprintf(fp3,"\n");
    //-- segmentos de obstaculo
        segObstaculos[0] = segmento3d(puntosObstaculo[0],puntosObstaculo[1]);
        segObstaculos[1] = segmento3d(puntosObstaculo[1],puntosObstaculo[2]);
        segObstaculos[2] = segmento3d(puntosObstaculo[2],puntosObstaculo[3]);
        segObstaculos[3] = segmento3d(puntosObstaculo[3],puntosObstaculo[0]);

        for(int k=0;k<Npuntos;k++){
            interseccion = seg_prueba.interseccion(segObstaculos[k],&P_interseccion);
            if (interseccion){
                if(P_interseccion==prev_P_interseccion){
//                    ROS_WARN("server_PlanificadorPisada::Se repite punto de interseccion");
                //-- Existe interseccion, pero es igual a la anterior (coincide con obst previo)
                    interseccion = false;
                } else {
                    interseccion = true;
                    prev_P_interseccion = P_interseccion;
                    break;
                }
            }
        }
        if (!interseccion){
//            ROS_WARN("server_PlanificadorPisada::No hubo interseccion entre obstaculo[%d][%d] con seg de pata[%d]",PisadaProxima_i, PisadaProxima_j,nPata+1);
            PisadaOk = false;
            return (PisadaOk);
        } else {
            interseccion = false;

            *correccion = P_interseccion.distancia(PisadaProxima)+delta_correccion_x;
            if(*correccion > seg_prueba.longitud()){
            //-- la correccion hallada sale del espacio de trabajo
            //-- FINALIZA LA FUNCION SI NO SE HALLA PUNTO ADECUADO
                ROS_WARN("server_PlanificadorPisada::correccion %d sale del EDT", correccion_ID);
                PisadaOk = false;
                return (PisadaOk);
            }
//            ROS_WARN("server_PlanificadorPisada::correccionX:%.4f",*correccion);
        //-- Se calcula la posible correccion
            aux_PisadaProxima.y = PisadaProxima.y;
            if(correccion_ID==Correccion_menosX){
                aux_PisadaProxima.x = PisadaProxima.x-(*correccion);
            } else {
                aux_PisadaProxima.x = PisadaProxima.x+(*correccion);
            }
            fprintf(fp3,"%d\t",5);
            fprintf(fp3,"%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n",aux_PisadaProxima.x,aux_PisadaProxima.y,0.0,0.0,0.0,0.0,0.0,0.0);
            transformacion_yxTOij(p_ij, aux_PisadaProxima.y, aux_PisadaProxima.x);
            PisadaProxima_i=ij[0];
            PisadaProxima_j=ij[1];
    //        ROS_WARN("server_PlanificadorPisada::Correccion Pata[%d] PisadaProxima_ij[%d][%d]",nPata+1,PisadaProxima_i,PisadaProxima_j);
            if(matrizMapa[PisadaProxima_i][PisadaProxima_j]){
                PisadaOk = false;
            } else {
                PisadaOk = true;
            }
        }
    }
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
        fprintf(fp3,"%d\t",4);
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
        fprintf(fp3,"\n");
    //    ROS_WARN("Puntos: Pata:%.3f,%.3f; Recta:%.3f,%.3f;%.3f,%.3f",Pata.x,Pata.y,puntosObstaculo[2].x,puntosObstaculo[2].y,puntosObstaculo[3].x,puntosObstaculo[3].y);
        recta_obstaculo = recta3d(puntosObstaculo[3],puntosObstaculo[2]);
        *correccion = recta_obstaculo.distancia(PisadaProxima)+delta_correccion_y;
//        ROS_WARN("server_PlanificadorPisada::correccionY:%.4f",*correccion);

        if(*correccion > lambda_maximo-0.01){
        //-- la correccion hallada sale del espacio de trabajo
        //-- FINALIZA LA FUNCION SI NO SE HALLA PUNTO ADECUADO
            ROS_WARN("server_PlanificadorPisada::correccion Y sale del EDT");
            PisadaOk = false;
            return (PisadaOk);
        }
        aux_PisadaProxima.x = PisadaProxima.x;
        aux_PisadaProxima.y = PisadaProxima.y-(*correccion);
        fprintf(fp3,"%d\t",5);
        fprintf(fp3,"%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n",aux_PisadaProxima.x,aux_PisadaProxima.y,0.0,0.0,0.0,0.0,0.0,0.0);
        transformacion_yxTOij(p_ij, aux_PisadaProxima.y, aux_PisadaProxima.x);
        PisadaProxima_i=ij[0];
        PisadaProxima_j=ij[1];
//        ROS_WARN("server_PlanificadorPisada::Correccion Pata[%d] aux_PisadaProxima[%d][%d]",nPata+1,PisadaProxima_i,PisadaProxima_j);
        if(matrizMapa[PisadaProxima_i][PisadaProxima_j]){
//            ROS_WARN("server_PlanificadorPisada::Correccion Pata[%d] coincidira con obstaculo [%d][%d]",nPata+1,PisadaProxima_i,PisadaProxima_j);
            PisadaOk = false;
        } else {
            PisadaOk = true;
        }
    }
        return (PisadaOk);
}
