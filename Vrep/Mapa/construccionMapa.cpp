#include "math.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <algorithm>
#include <ctime>
//Variables Globales
#define nFilas 100
#define nColumnas 20
#define caso 1
FILE *fp2;
int matrizMapa[100][100];
int arregloID[1200];
//-- Funciones
void Limpiar_matrizMapa();
void print_matrizMapa();
void FilePrint_matrizMapa();
void MapaLleno();
void hLineas(int nLineas, int *lineas);
void vLineas(int nLineas, int *lineas);
void hBloques(int nBloques, int *bloques, int *lado);
void MapaMedioLleno();
int myrandom (int i);

int main(){

int *L,*op, hlineas[nFilas],vlineas[nColumnas], bloques[nFilas], lado[nFilas];

//-- Inicializacion de semilla
    std::srand ( unsigned ( std::time(0) ) );
//-- Inicializacion de archivo de salida
    fp2=fopen("Mapa.txt","w+");
    if (fp2==NULL){
        printf ("Error opening file unexist.ent: %s\n",strerror(errno));
        return -1;
    }
//-- Creamos matriz vacia
    Limpiar_matrizMapa();

    switch (9){
        case 0:
            MapaLleno();
            FilePrint_matrizMapa();
        break;
        case 1:
            hlineas[0]=nFilas/2;
            L=hlineas;
            hLineas(1,L);
            FilePrint_matrizMapa();
        break;
        case 2:
            hlineas[0]=nFilas/3;
            hlineas[1]=2*nFilas/3;
            L=hlineas;
            hLineas(2,L);
            FilePrint_matrizMapa();

        break;
        case 3:
            hlineas[0]=nFilas/3;
            hlineas[1]=nFilas/3 + 6;
            L=hlineas;
            hLineas(2,L);
            FilePrint_matrizMapa();
        break;
        case 4:
            vlineas[0]=nColumnas/2;
            vlineas[1]=nColumnas/2-1;
            L=vlineas;
            vLineas(2,L);
            FilePrint_matrizMapa();

        break;
        case 5:
            hlineas[0]=nFilas/4;
            hlineas[1]=2*nFilas/4;
            hlineas[2]=3*nFilas/4;
            lado[0]=0;
            lado[1]=0;
            lado[2]=0;
            L=hlineas; op=lado;
            hBloques(3,L,op);
            FilePrint_matrizMapa();
        break;
        case 6:
            hlineas[0]=nFilas/3+5;
            hlineas[1]=nFilas/3+10;
            hlineas[2]=nFilas/3+15;
            hlineas[3]=nFilas/3+20;
            lado[0]=0;
            lado[1]=0;
            lado[2]=0;
            lado[3]=0;
            L=hlineas; op=lado;
            hBloques(4,L,op);
            FilePrint_matrizMapa();
        break;
        case 7:
            hlineas[0]=nFilas/3;
            hlineas[1]=2*nFilas/3;
            lado[0]=0;
            lado[1]=1;
            L=hlineas; op=lado;
            hBloques(2,L,op);
            FilePrint_matrizMapa();
        break;
        case 8:
            hlineas[0]=nFilas/2;
            hlineas[1]=nFilas/2 + 6;
            lado[0]=0;
            lado[1]=1;
            L=hlineas; op=lado;
            hBloques(2,L,op);
            FilePrint_matrizMapa();
        break;
        case 9:
            MapaMedioLleno();
            FilePrint_matrizMapa();
        break;

        default:

        break;
    }

    fclose(fp2);
    return 0;
}

void Limpiar_matrizMapa(){
int i=0, j=0;

    for(i=0;i<nFilas;i++){
        for(j=0;j<nColumnas;j++){
            matrizMapa[i][j]=0;
        }
    }
}

void print_matrizMapa(){
     printf("\n");
     for(int i=0;i<nFilas;i++){
         for(int j=0;j<nColumnas;j++){
            printf(" %d.",matrizMapa[i][j]);
            }
         }
        printf("\n");
    }

void FilePrint_matrizMapa(){
int i=0, j=0;

     for(i=0;i<nFilas;i++){
         for(j=0;j<nColumnas;j++){
            fprintf(fp2,"%d\t",matrizMapa[i][j]);
            }
        fprintf(fp2,"\n");
     }
}
//-- Funciones de casos de obstaculos

/*Caso0: mapa lleno
*/
void MapaLleno(){
int i, j;
    for(i=0;i<nFilas;i++){
        for(j=0;j<nColumnas;j++){
            matrizMapa[i][j]=1;
        }
    }
}

/* Caso1: crea lineas horizontales en el mapa:
parametros:
..nlineas - cuantas lineas horizontales se crean
..lineas - apuntador a arreglo con posiciones (i) de las lineas
*/
void hLineas(int nLineas, int *lineas){
    int j, k;

    for(k=0;k<nLineas;k++){
        for(j=0;j<nColumnas;j++){
            matrizMapa[lineas[k]][j]=1;
        }
    }
}

/* Caso2: crea lineas verticales en el mapa:
parametros:
..nlineas - cuantas lineas verticales se crean
..lineas - apuntador a arreglo con posiciones (j) de las lineas
*/
void vLineas(int nLineas, int *lineas){
    int i, k;

    for(k=0;k<nLineas;k++){
        for(i=0;i<nFilas;i++){
            matrizMapa[i][lineas[k]]=1;
        }
    }
}

/* Caso3: crea bloques horizontales de longitud nColumnas/2 en el mapa:
parametros:
..nBloques - cuantos bloques horizontales se crean
..bloques - apuntador a matriz con posicion (i) de los bloques
..lado - señala lado para creacion de bloque: opcion de izquierda (0) o derecha (1)
.. ejemplo:  i    op
            [4     0] -->bloque en i=4, lado izquierdo
            [10    1] -->bloque en i=10, lado derecho
*/
void hBloques(int nBloques, int *bloques, int *lado){
    int j, k;

    for(k=0;k<nBloques;k++){
        if(lado[k]==0){
            for(j=0;j<nColumnas/2;j++) matrizMapa[bloques[k]][j]=1;
        } else {
            for(j=nColumnas/2;j<nColumnas;j++) matrizMapa[bloques[k]][j]=1;
        }
    }
}

/*Caso0: mapa medio-lleno (vertical)
*/
void MapaMedioLleno(){
int i, j;
    for(i=0;i<nFilas;i++){
        for(j=0;j<nColumnas/2;j++){
            matrizMapa[i][j]=1;
        }
    }
}

// random generator function:
int myrandom (int i) {
    return std::rand()%i;
}
