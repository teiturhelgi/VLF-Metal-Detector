#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include "math.h"

void make_dft(double *magnitude, double *phase, volatile short *samp){
    //Code for calculating DFT of sample array
    double ReX = 0, ImX = 0;

    for (int n=0; n<64; n+=4){

        ReX += samp[n];
        ReX -= samp[n+2];

        ImX += samp[n+1];
        ImX -= samp[n+3];
        
        //printf("%d: ReX: %.2f ImX: %.2f\n", i, ReX, ImX);
    }

    //Calculate magnitude in percentage
    *magnitude = sqrt((ReX*ReX)+(ImX*ImX))/16/4095*100/141;
    //Calculate phase in degrees
    *phase = ((atan2(-ImX,ReX)*180)/3.141592); 
    // if (*phase >= 90){
    //     *phase = 270 - *phase;
    // }
    // else {
    //     *phase = -90 - *phase;
    // }
    if (*phase < 0)
    {
        *phase += 360;
    }
}