#include <stdio.h>
#include "vector3d.cpp"


int main()
{
    punto3d a(0,0,0), b(10,10,10); //creo dos puntos de prueba

    segmento3d s(a,b);

    float l = s.longitud();

    printf ("Longitud de segmento: %f",l);
}
