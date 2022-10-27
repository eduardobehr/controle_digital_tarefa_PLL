/**
 * @file pll.c
 * @author Eduardo Eller Behr (eellerbehr@gmail.com)
 * @brief 
 * @date 2022-10-24
 * 
 * @copyright (c) 2022
 * 
 */
#include <pll.h>

#include <driver/dac.h>

#include "esp_system.h"
#include <esp_log.h>
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* PLL_TAG = "PLL Task";
static const char* MAIN_TAG = "Main Task";

static uint64_t counter = 0;

void output_update_callback(void*){
    static int index = 0; // índice da tabela do sinal com harmônicas
    
    // sinal de entrada com degraus repetidos e ruído branco
    double input_signal = amplitude_step(20000, 0)*wave_table[index] + 0.3*rand()/RAND_MAX;

    // cálculo do sinal de saída
    double pll_output = sogi_pll(&input_signal, 0.2);

    // escrever sinais de entrada e saída no conversor digital para analógico
    const uint8_t offset = 100;
    const float gain = 80;
    dac_output_voltage(DAC_CHANNEL_1, clamp_8_bits(offset+gain*(input_signal))); // GPIO25
    dac_output_voltage(DAC_CHANNEL_2, clamp_8_bits(offset+gain*pll_output)); // GPIO26

    // incrementar índice da tabela do seno
    index = (index + 1) % WAVE_TABLE_LEN;
    counter++;
}

void pll_task(){
    ESP_LOGI(PLL_TAG, "Iniciando");

    dac_output_enable(DAC_CHANNEL_1);
    dac_output_enable(DAC_CHANNEL_2);

    make_wave(wave_table, WAVE_TABLE_LEN, FG);

    esp_timer_handle_t output_update_timer;

    const esp_timer_create_args_t output_update_timer_args = {
        .callback = &output_update_callback,
        .name = "Output Update Callback Timer",
        .arg = NULL
    };

    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_create(
        &output_update_timer_args, 
        &output_update_timer
    ));

    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_start_periodic(output_update_timer, TSAMP_US));


    while(true){
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(PLL_TAG, "Contagem de amostras: %llu", counter);
    }
    
}

void app_main(){
    ESP_LOGI(MAIN_TAG, "Iniciando");

    xTaskCreate(
        pll_task,   // função
        "pll",      // nome
        4096,       // memória
        NULL,       // ponteiro para parametros
        5,          // prioridade
        NULL        // handle
    );
}