/**
 * @file pll.h
 * @author Eduardo Eller Behr (eellerbehr@gmail.com)
 * @brief 
 * @date 2022-10-24
 * 
 * @copyright (c) 2022
 * 
 */
#pragma once

void pll_task();

#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#define FG 	50
#define PI 		3.141592653589793
#define PI2 	9.869604401089358

#define FSAMP 10000.0
#define WAVE_TABLE_LEN ((uint32_t) FSAMP/FG)
const uint32_t TSAMP_US = 1e6/(FSAMP);

const double Ts = 0.0001;

static double wave_table[WAVE_TABLE_LEN];



void make_wave(double* buffer, size_t len, double freq){
    const double dt = 1/((double)FSAMP);
    for(int i=0; i<len; i++){
        const double t = i*dt;
        
        buffer[i] = sin(2*PI*freq*t)
        +(1./4.)*sin(2*PI*freq*3*t+20*PI/180)
        +(1./20.)*sin(2*PI*freq*5*t-40*PI/180);
    }
}

double sogi_pll(const double* input, float K){
	// definir buffers de entrada e saida
	//                 Z-index:  0      -1      -2
	static double out_buff[] = {0.0,    0.0,    0.0};
	static double in_buff[] =  {0.0,    0.0,    0.0};
	
	// atualizar variável de entrada
	in_buff[0] =  *input;

	const double A0 = 1 + PI*K*Ts*FG + pow(PI*Ts*FG, 2);
	const double A1 = 2*(pow(PI*Ts*FG, 2)-1);
	const double A2 = 1 - PI*K*Ts*FG + pow(PI*Ts*FG, 2);
	const double B0 = PI*K*Ts*FG;
	const double B2 = -PI*K*Ts*FG;

	// calcular resultado
	out_buff[0] = (B0*in_buff[0] + B2*in_buff[2] - A2*out_buff[2] -A1*out_buff[1])/A0;

	// atualizar variáveis passadas
	in_buff[2] = in_buff[1];
	in_buff[1] = in_buff[0];
	
	out_buff[2] = out_buff[1];
	out_buff[1] = out_buff[0];

	return out_buff[0];
}


float amplitude_step(int count_to, float gain){
	static int counter = 0;
	static bool active = false;
	float ret;

	if (counter >= count_to){
		active = !active;
		counter = 0;
	}
	
	if(active){
		ret = gain;
	} else{
		ret = 1.0f;
	}
	counter++;
	return ret;
}

uint8_t clamp_8_bits(float value){
    uint8_t ret;
    if(value > 255.0){
        ret = 255;
    } else if(value < 0){
        ret = 0;
    } else{
        ret = value;
    }
    return ret;
}