#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include "constantes.hpp"

#define pi 3.141592


int main()
{
    float q[3]={0,0,0};
    float ang_rot = pi/3;
    float x=0, y=0, z=0, px=0, py=0, pz=0, p1=0, p2=0, L23=0, beta=0, teta=0, gamma=0, arg=0, arg1=0;
    float xoff=0.1299, yoff=0.075, zoff=0.05;

//    x=0.2271;
//    y=0.1311;
//    z=-0.0475;
    x=-0.01;
    y= 0.3;
    z=-0.095;

    px = cos(-ang_rot)*(x+xoff) + sin(-ang_rot)*(y+yoff);
    py = -sin(-ang_rot)*(x+xoff) + cos(-ang_rot)*(y+yoff);
    pz = z+zoff;

    printf("\n px:%.3f, py:%.3f, pz:%.3f", px, py, pz);

    //calculo q1
	q[0] = atan2(px,py);

	p1 = sqrt(px*px + py*py) - L1;
	//p1 = sqrt(((px-L1*sin(q[0]))*(px-L1*sin(q[0]))) + ((py-L1*cos(q[0]))*(py-L1*cos(q[0]))));
	p2 = pz;
    printf("\n p1:%.3f, p2:%.3f", p1, p2);

	L23 = sqrt(p1*p1 + p2*p2);
	printf("\n L23:%.3f", L23);

    //calculo q3
	arg = (L2*L2 + L3*L3 - L23*L23)/(2*L2*L3);
	if (fabs(arg)>=1) {
        printf("\n ERROR acos");
        return 0;
	}
	beta = acos(arg);
	printf("\n beta:%.3f", beta);
    q[2] = pi - beta;

	//calculo q2
	arg1= (L3/L23)*sin(beta);
	if (abs(arg1)>=1) {
        printf("\n ERROR asin");
        return 0;
	}
	teta = atan2(-p2,p1);
	printf("\n teta: %.3f", teta);
	gamma = asin(arg1);
	printf("\n gamma: %.3f", gamma);
	q[1] = pi/2 - (teta - gamma);

	printf("\n\n q1=%.3f, q2=%.3f, q3=%.3f", q[0], q[1], q[2]);

    return 0;
}
