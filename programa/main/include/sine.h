/**
 * @file sine.h
 * @author Eduardo Eller Behr (eellerbehr@gmail.com)
 * @brief 
 * @date 2022-10-01
 * 
 * @copyright (c) 2022
 * 
 */
#pragma once
#include <stdint.h>

double rectified_sine_table[] = {
    0,
    0.037690182669935,
    0.075326805527933,
    0.112856384873482,
    0.150225589120757,
    0.187381314585725,
    0.224270760949381,
    0.260841506289897,
    0.297041581577035,
    0.332819544522987,
    0.368124552684678,
    0.402906435713663,
    0.437115766650933,
    0.470703932165333,
    0.503623201635761,
    0.535826794978997,
    0.567268949126757,
    0.597904983057519,
    0.627691361290701,
    0.656585755752957,
    0.684547105928689,
    0.711535677209285,
    0.737513117358174,
    0.762442511011448,
    0.786288432136619,
    0.809016994374947,
    0.830595899195812,
    0.850994481794692,
    0.870183754669525,
    0.888136448813544,
    0.904827052466019,
    0.92023184736587,
    0.934328942456612,
    0.947098304994744,
    0.958521789017376,
    0.968583161128631,
    0.977268123568193,
    0.984564334529205,
    0.990461425696651,
    0.9949510169813,
    0.998026728428271,
    0.9996841892833,
    0.999921044203816,
    0.998736956606018,
    0.996133609143172,
    0.992114701314478,
    0.986685944207868,
    0.979855052384247,
    0.971631732914674,
    0.962027671586086,
    0.951056516295154,
    0.938733857653874,
    0.925077206834458,
    0.910105970684996,
    0.893841424151264,
    0.876306680043864,
    0.857526656193652,
    0.837528040042142,
    0.816339250717184,
    0.793990398647835,
    0.770513242775789,
    0.745941145424182,
    0.720309024887906,
    0.693653305812804,
    0.666011867434251,
    0.637423989748689,
    0.607930297694604,
    0.577572703422266,
    0.546394346734268,
    0.514439533781505,
    0.481753674101714,
    0.448383216090031,
    0.414375580993282,
    0.379779095521799,
    0.344642923174515,
    0.309016994374945,
    0.272951935517323,
    0.236498997023722,
    0.199709980514404,
    0.162637165194881,
    0.125333233564302,
    0.087851196550741,
    0.050244318179768,
};




const uint16_t cArrayLength = sizeof(rectified_sine_table)/sizeof(double);

double get_duty_of_rectified_sine(uint32_t period, uint16_t index){
    
    index = index % cArrayLength; // limitar índice
    return ((double)period)*rectified_sine_table[index];
}