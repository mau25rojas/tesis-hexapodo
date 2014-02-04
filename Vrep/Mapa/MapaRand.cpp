#include "math.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <algorithm>
#include <ctime>
#include <vector>
//Variables Globales
#define nFilas 100
#define nColumnas 20
#define caso 1
FILE *fp2;
int matrizMapa[100][100];
int matrizID[2000][2];
std::vector<int> arregloID;
//-- Funciones
void Limpiar_matrizMapa();
void Limpiar_matrizMapa_Robot();
void print_matrizMapa();
void FilePrint_matrizMapa();
int myrandom (int i);

int main(){
//-- Inicializacion de semilla
    std::srand ( unsigned ( std::time(0) ) );
//-- Inicializacion de archivo de salida
    fp2=fopen("Mapa.txt","w+");
    if (fp2==NULL){
        printf ("Error opening file unexist.ent: %s\n",strerror(errno));
        return -1;
    }
    int cuenta=0;
    for(int i=0;i<nFilas;i++){
        for(int j=0;j<nColumnas;j++){
            arregloID.push_back(cuenta);
            matrizID[cuenta][0]=i;
            matrizID[cuenta][1]=j;
            cuenta++;
        }
    }
//-- using built-in random generator:
    std::random_shuffle(arregloID.begin(),arregloID.end());
//-- using myrandom:
    std::random_shuffle(arregloID.begin(),arregloID.end(),myrandom);

//    printf("permutacion arreglo:\n");
    for(int j=0;j<(0.5)*(nFilas*nColumnas);j++){
//        printf("%d: i:%d, j:%d\n",arregloID[j],matrizID[arregloID[j]][0],matrizID[arregloID[j]][1]);
        matrizMapa[matrizID[arregloID[j]][0]][matrizID[arregloID[j]][1]]=1;
    }
//-- limpiar area en donde inicia el robot
    Limpiar_matrizMapa_Robot();
    FilePrint_matrizMapa();

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

// random generator function:
int myrandom (int i) {
    return std::rand()%i;
}

void Limpiar_matrizMapa_Robot(){
int i=0, j=0;

    for(i=77;i<nFilas;i++){
        for(j=0;j<nColumnas;j++){
            matrizMapa[i][j]=0;
        }
    }
}
