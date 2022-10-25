/**
 * @file pwm_task.c
 * @author Eduardo Eller Behr (eduardo.behr@hotmail.com)
 * @brief 
 * - Timer de 333µs para atualizar a senoide 
 * @date 2022-09-07
 * 
 */

/// user includes
#include <sine.h>
#include <pwm_task.h>

/// system
#include "esp_log.h"
#include "esp_timer.h"

/// drivers
#include <driver/gpio.h>

/// registers
#include <soc/mcpwm_reg.h>
#include <soc/dport_reg.h>
#include <soc/gpio_reg.h>
#include <soc/gpio_sig_map.h>

/// freertos
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

/// stdlib
#include <stdio.h>
#include <math.h>

static const char* TAG = "PWM";

static const uint32_t cSineMicroSeconds = 100;
static const uint16_t cPwmPeriod = 1000;

static const uint8_t PWM0 = 0;

// Configurações dos pinos
static const uint8_t cGpioS1 = 13;
static const uint8_t cGpioS2 = 12;
static const uint8_t cGpioS3 = 14;
static const uint8_t cGpioS4 = 27;
static const uint8_t cGpioS5 = 26;
static const uint8_t cGpioS6 = 25;
static const uint8_t cGpioS10 = 33;


static const uint32_t cGpioSelectionMask = 
      BIT(cGpioS1) | BIT(cGpioS2)| BIT(cGpioS3)
    | BIT(cGpioS4) | BIT(cGpioS5)| BIT(cGpioS6)
    | BIT(cGpioS10);


/**
 * @brief Encapsula pinos usados para realizar o PWM por meio do PwmData
 * 
 */
typedef struct PwmPin {
    uint8_t S1;
    uint8_t S2;
    uint8_t S3;
    uint8_t S4;
    uint8_t S5;
    uint8_t S6;
} PwmPin;

/**
 * @brief Encapsula os dados usados pelo PWM
 * 
 */
typedef struct PwmData {
    uint64_t time_us;   /// tempo em microsegundos
    double modulator;   /// senoide
    uint8_t vt1;        /// portadora 1
    uint8_t vt2;        /// portadora 2
    uint8_t vt3;        /// portadora 3
    bool vt1_upward;    /// portadora 1 crescente
    bool vt2_upward;    /// portadora 2 crescente
    bool vt3_upward;    /// portadora 3 crescente
    PwmPin pin;

} PwmData;

static void pwm_item_1();
static void pwm_item_2();

static int current_item = eNone;

void choose_item(eItem item){
    current_item = eNone;

    switch(item){
        case eNone: {
            return;
        }
        case ePwmItem_1: {
            pwm_item_1();
            break;
        }

        case ePwmItem_2: {
            pwm_item_2();
            break;
        }
    }

}

static volatile float sine_amplitude_scale = 1.0f; /// Para não saturar, deve ser menor ou igual a cPwmPeriod

static void sine_duty_update_callback(void* pPwmData){

    static uint16_t index = 0;
    static bool positive = true;

    const uint32_t current_period = cPwmPeriod;
    switch(current_item){
    
    case eNone: {
        return;
    }
    
    case ePwmItem_1:{
        const uint32_t new_duty = get_duty_of_rectified_sine(current_period, index);
        index = (index+1) % cArrayLength;
    
        REG_WRITE(MCPWM_GEN0_TSTMP_A_REG(PWM0), new_duty);
        REG_WRITE(MCPWM_GEN0_TSTMP_B_REG(PWM0), new_duty);
        REG_WRITE(MCPWM_GEN1_TSTMP_A_REG(PWM0), new_duty);
        REG_WRITE(MCPWM_GEN2_TSTMP_B_REG(PWM0), new_duty);
        

        break;
    }
    case ePwmItem_2:{
        const float new_duty = get_duty_of_rectified_sine(current_period, index);
        index = (index+1) % cArrayLength;

        uint32_t duty_1 = 0;
        uint32_t duty_2 = 0;
        uint32_t duty_3 = 0;

        uint32_t scaled_duty = new_duty*sine_amplitude_scale;

        if(scaled_duty <= ((float) current_period)/3.0f ){
            duty_1 = 3.0*scaled_duty; // el/(X0*1/3)       el é do x = X0*np.abs(np.sin(wg*t))
        }
        else if(scaled_duty <= ((float) current_period)*(2.0/3.0) ){
            duty_1 = current_period;
            duty_2 = 3.0*(scaled_duty-current_period/3.0);
        }
        else if(scaled_duty <= current_period ){
            duty_1 = current_period;
            duty_2 = current_period;
            duty_3 = 3.0*(scaled_duty-2.0*current_period/3.0);
        }
        else if(scaled_duty > current_period ){
            duty_1 = current_period;
            duty_2 = current_period;
            duty_3 = current_period;
        }

        // Aplicar novas razões cíclicas nos registradores
        REG_WRITE(MCPWM_GEN0_TSTMP_A_REG(PWM0), duty_1);
        REG_WRITE(MCPWM_GEN1_TSTMP_A_REG(PWM0), duty_2);
        REG_WRITE(MCPWM_GEN2_TSTMP_A_REG(PWM0), duty_3);

        const uint8_t cSyncAtTEP = 0b10;
        const uint8_t cSyncAtTEZ = 0b01;

        // remoção dos artefatos de update (dependendo se está subindo ou descendo o seno)
        if(index <= cArrayLength/2){
        // if(0 < index && index <= cArrayLength/2){
            // rising of sine
            REG_WRITE(MCPWM_GEN0_STMP_CFG_REG(PWM0), cSyncAtTEZ << MCPWM_GEN0_A_UPMETHOD_S | cSyncAtTEZ << MCPWM_GEN0_B_UPMETHOD_S);
            REG_WRITE(MCPWM_GEN1_STMP_CFG_REG(PWM0), cSyncAtTEZ << MCPWM_GEN1_A_UPMETHOD_S | cSyncAtTEZ << MCPWM_GEN1_B_UPMETHOD_S);
        } else {
            // falling of sine
            REG_WRITE(MCPWM_GEN0_STMP_CFG_REG(PWM0), cSyncAtTEP << MCPWM_GEN0_A_UPMETHOD_S | cSyncAtTEP << MCPWM_GEN0_B_UPMETHOD_S);
            REG_WRITE(MCPWM_GEN1_STMP_CFG_REG(PWM0), cSyncAtTEP << MCPWM_GEN1_A_UPMETHOD_S | cSyncAtTEP << MCPWM_GEN1_B_UPMETHOD_S);
        }

        break;
    }
    }
    
    // Detectores de ciclo (úteis para sincronizar trigger do osciloscópio)
    if(index == 0){
        positive = !positive;
    }


    if(positive){

        REG_SET_BIT(GPIO_OUT_W1TS_REG, 1 << cGpioS5);
        REG_SET_BIT(GPIO_OUT_W1TC_REG, 1 << cGpioS6);
    }
    else{
        REG_SET_BIT(GPIO_OUT_W1TS_REG, 1 << cGpioS6);
        REG_SET_BIT(GPIO_OUT_W1TC_REG, 1 << cGpioS5);
    }
        
}

static inline void config_pins(){
    gpio_config_t gpio_driver_config = {};

    gpio_driver_config.mode = GPIO_MODE_DEF_OUTPUT;
    gpio_driver_config.pin_bit_mask = cGpioSelectionMask;

    gpio_config(&gpio_driver_config);

    // Set GPIOs as output
    REG_WRITE(
        GPIO_OUT_W1TS_REG,
        cGpioSelectionMask
    );

    // Connect GPIO to PWM (GPIO_FUNCn_OUT_SEL_CFG_REG)
    // connect PWM signals
    REG_WRITE(DR_REG_GPIO_BASE + 0x530+0x4*cGpioS1, PWM0_OUT0A_IDX);
    REG_WRITE(DR_REG_GPIO_BASE + 0x530+0x4*cGpioS2, PWM0_OUT0B_IDX);
    REG_WRITE(DR_REG_GPIO_BASE + 0x530+0x4*cGpioS3, PWM0_OUT1A_IDX);
    REG_WRITE(DR_REG_GPIO_BASE + 0x530+0x4*cGpioS4, PWM0_OUT1B_IDX);
    REG_WRITE(DR_REG_GPIO_BASE + 0x530+0x4*cGpioS10, PWM0_OUT2A_IDX);
    // ...
 

    // Configure the IO_MUX to select the GPIO Matrix.

    *((volatile uint32_t*) DR_REG_IO_MUX_BASE + 0x10+4*cGpioS1) = (
          (2U << 12U) // MCU_SEL: Function 2 (GPIO)
        | (2U << 10U) // FUN_DRV: 2 (default)
    );
}

static inline void config_pwm(PwmData* pPwmData){
    const esp_timer_create_args_t sine_100us_timer_args = {
        .callback = &sine_duty_update_callback,
        .name = "Pwm modulator timer",
        .arg = pPwmData
    };


    esp_timer_handle_t sine_100us_periodic_timer;

    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_create(
        &sine_100us_timer_args, 
        &sine_100us_periodic_timer
    ));

    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_start_periodic(sine_100us_periodic_timer, cSineMicroSeconds));
}



void pwm_item_2(){
    current_item = ePwmItem_2;
    // Habilitar clock para o periférico PWM0:
    DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_PWM0_CLK_EN);

    DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_PWM0_RST);
    // desabilitar protecao de memoria
    
    const uint8_t cPwmWorkinMode = 3; // 0: freeze, 1: increase mode, 2: decrease mode, 3: up-down mode
    uint16_t duty = cPwmPeriod/2;

    // pré-escala: Period of PWM_clk = 6.25ns * (PWM_CLK_PRESCALE + 1).
    REG_WRITE(MCPWM_CLK_CFG_REG(PWM0), 7<<0); // 50ns
    
    // Portadora vt1 (PWM0, timer 0)
    {
        // Timer period and update method
        REG_WRITE(MCPWM_TIMER0_CFG0_REG(PWM0), 
                (1 << 24)   // PWM_TIMER0_PERIOD_UPMETHOD. 0: immediately, 1: update at TEZ, 2: update at sync, 3: update at TEZ or sync.
            |   (cPwmPeriod << 8) // Period shadow register of PWM timer0 1000 = 500µs/50ns/2  (up e down divide por dois)
            |   (0 << 0)    // timer0 pre-scale
        );

        // Working mode and start/stop control
        REG_WRITE(MCPWM_TIMER0_CFG1_REG(PWM0), 
                (cPwmWorkinMode << 3)  //  PWM timer0 working mode. 0: freeze, 1: increase mode, 2: decrease mode, 3: up-down mode.
            |   (2 << 0)  //  PWM timer0 start and stop control. 2: PWM timer0 starts and runs on
        );
    
        // Synchronization settings
        // Fazer o Timer 0 emitir um sinal de sincronização (para o Timer 1 receber)
        REG_WRITE(MCPWM_TIMER0_SYNC_REG(PWM0), 
                (0 << MCPWM_TIMER0_PHASE_DIRECTION_S)   // 0: increase; 1: decrease.
            |   (0 << MCPWM_TIMER0_PHASE_S)             // Phase for timer reload at sync event. Don't care
            |   (2 << MCPWM_TIMER0_SYNCO_SEL_S)         // PWM timer0 sync_out selection. 0: sync_in; 1: TEZ; 2: TEP; other-wise: sync_out is always 0.
            |   (0 << MCPWM_TIMER0_SYNC_SW_S)           // Toggling this bit will trigger a software sync.
            |   (0 << MCPWM_TIMER0_SYNCI_EN_S)          // When set, timer reloading with phase on sync input event is enabled.
        );

    }

    // Portadora vt2 (PWM0, timer 1)
    {
        // Timer period and update method
        REG_WRITE(MCPWM_TIMER1_CFG0_REG(PWM0), 
                (1 << 24)   // update at TEZ
            |   (cPwmPeriod << 8) // Period shadow register of PWM timer1 2000 = 500µs/50ns
            |   (0 << 0)    // timer1 pre-scale
        );

        // Working mode and start/stop control
        REG_WRITE(MCPWM_TIMER1_CFG1_REG(PWM0), 
                (cPwmWorkinMode << 3)  //  PWM timer1 working mode. 0: freeze, 1: increase mode, 2: decrease mode, 3: up-down mode.
            |   (2 << 0)  //  PWM timer1 start and stop control. 2: PWM timer1 starts and runs on
        );
        
        // Synchronization settings
        REG_WRITE(MCPWM_TIMER1_SYNC_REG(PWM0), 
                (0 << MCPWM_TIMER1_PHASE_DIRECTION_S)   // 0: increase; 1: decrease.
            |   (0 << MCPWM_TIMER1_PHASE_S)             // Phase for timer reload at sync event. (manter em fase no problema 2)
            |   (0 << MCPWM_TIMER1_SYNCO_SEL_S)         // PWM timer1 sync_out selection. 0: sync_in; 1: TEZ; 2: TEP; other-wise: sync_out is always 0.
            |   (0 << MCPWM_TIMER1_SYNC_SW_S)           // Toggling this bit will trigger a software sync.
            |   (1 << MCPWM_TIMER1_SYNCI_EN_S)          // When set, timer reloading with phase on sync input event is enabled.
        );
    }

    // Portadora vt3 (PWM0, timer 2)
    {
        // Timer period and update method
        REG_WRITE(MCPWM_TIMER2_CFG0_REG(PWM0), 
                (1 << 24)   // update at TEZ
            |   (cPwmPeriod << 8) // Period shadow register of PWM timer1 2000 = 500µs/50ns
            |   (0 << 0)    // timer1 pre-scale
        );

        // Working mode and start/stop control
        REG_WRITE(MCPWM_TIMER2_CFG1_REG(PWM0), 
                (cPwmWorkinMode << 3)  //  PWM timer1 working mode. 0: freeze, 1: increase mode, 2: decrease mode, 3: up-down mode.
            |   (2 << 0)  //  PWM timer1 start and stop control. 2: PWM timer1 starts and runs on
        );
        
        // Synchronization settings
        REG_WRITE(MCPWM_TIMER2_SYNC_REG(PWM0), 
                (0 << MCPWM_TIMER2_PHASE_DIRECTION_S)   // 0: increase; 1: decrease.
            |   (0 << MCPWM_TIMER2_PHASE_S)             // Phase for timer reload at sync event. (manter em fase no problema 2)
            |   (0 << MCPWM_TIMER2_SYNCO_SEL_S)         // PWM timer1 sync_out selection. 0: sync_in; 1: TEZ; 2: TEP; other-wise: sync_out is always 0.
            |   (0 << MCPWM_TIMER2_SYNC_SW_S)           // Toggling this bit will trigger a software sync.
            |   (1 << MCPWM_TIMER2_SYNCI_EN_S)          // When set, timer reloading with phase on sync input event is enabled.
        );
    }

    // Conectar a saída de sincronização do Timer 0 na entrada de sincronização do Timer 1 e Timer 2
    REG_WRITE(MCPWM_TIMER_SYNCI_CFG_REG(PWM0), 
        1 << MCPWM_TIMER1_SYNCISEL_S
        | 1 << MCPWM_TIMER2_SYNCISEL_S
    ); // Select sync input for PWM timer1. 1: PWM timer0 sync_out, 2: PWM timer1 sync_out, 3: PWM timer2 sync_out, ...


    const uint8_t cActionSetLow = 1;
    const uint8_t cActionSetHigh = 2;
    // const uint8_t cActionToggle = 3;

    const uint8_t cSyncAtTEP = 0b10;
    const uint8_t cSyncAtTEZ = 0b01;
    
    // PWM Operator 0
    {
        // Selecionar Timer 0 para o operador 0
        REG_SET_BITS(
            MCPWM_OPERATOR_TIMERSEL_REG(PWM0), 
            0 << MCPWM_OPERATOR0_TIMERSEL_S,    //  0: timer0, 1: timer1, 2: timer2.
            MCPWM_OPERATOR0_TIMERSEL_M          // máscara binária
        );


        // Configurar update method para TEZ
        // When all bits are set to 0: immediately; when bit0 is set to 1: TEZ; 
        // when bit1 is set to 1: TEP; when bit2 is set to 1: sync; 
        // when bit3 is set to 1: disable the update.
        REG_WRITE(
            MCPWM_GEN0_STMP_CFG_REG(PWM0), 
              cSyncAtTEZ << MCPWM_GEN0_A_UPMETHOD_S 
            | cSyncAtTEZ << MCPWM_GEN0_B_UPMETHOD_S
        );

        // Canal A PWM (Operator 0)
        {
            // Configurar razão cíclica
            REG_WRITE(MCPWM_GEN0_TSTMP_A_REG(PWM0), duty);
    
            // Ações a partir de eventos no canal A
            REG_WRITE(
                MCPWM_GEN0_A_REG(PWM0),
                (cActionSetHigh << MCPWM_GEN0_A_DTEA_S)  // LOW quando T=A (decreasing)
                | (cActionSetLow << MCPWM_GEN0_A_UTEA_S) // HIGH quando T=A (increasing)
            );
        }


        // Canal B (PWM Operator 0)
        {
            // Configurar razão cíclica
            REG_WRITE(MCPWM_GEN0_TSTMP_B_REG(PWM0), duty);

            // Ações a partir de eventos no canal B
            REG_WRITE(
                MCPWM_GEN0_B_REG(PWM0),
                (cActionSetHigh << MCPWM_GEN0_B_UTEA_S)  // LOW quando T=B (decreasing)
                | (cActionSetLow << MCPWM_GEN0_B_DTEA_S) // HIGH quando T=B (increasing)
            );
        }
    }

    // PWM Operator 1
    {
        // Selecionar Timer 1 para o operador 1
        REG_SET_BITS(
            MCPWM_OPERATOR_TIMERSEL_REG(PWM0), 
            1 << MCPWM_OPERATOR1_TIMERSEL_S,    //  0: timer0, 1: timer1, 2: timer2.
            MCPWM_OPERATOR1_TIMERSEL_M          // máscara binária
        );

        // Configurar update method para TEZ
        // When all bits are set to 0: immediately; when bit0 is set to 1: TEZ; 
        // when bit1 is set to 1: TEP; when bit2 is set to 1: sync; 
        // when bit3 is set to 1: disable the update.
        REG_WRITE(
            MCPWM_GEN1_STMP_CFG_REG(PWM0), 
              cSyncAtTEZ << MCPWM_GEN1_A_UPMETHOD_S 
            | cSyncAtTEZ << MCPWM_GEN1_B_UPMETHOD_S
        );

        // Canal A (PWM Operator 1)
        {
            // Configurar razão cíclica
            REG_WRITE(MCPWM_GEN1_TSTMP_A_REG(PWM0), duty);
    
            // Ações a partir de eventos no canal A
            REG_WRITE(
                MCPWM_GEN1_A_REG(PWM0),
                (cActionSetHigh << MCPWM_GEN1_A_DTEA_S)  // LOW quando T=A (decreasing)
                | (cActionSetLow << MCPWM_GEN1_A_UTEA_S) // HIGH quando T=A (increasing)
            );
        }
    

        // Canal B (PWM Operator 1)
        {
            // Configurar razão cíclica
            REG_WRITE(MCPWM_GEN1_TSTMP_B_REG(PWM0), duty);

            // Ações a partir de eventos no canal B
            REG_WRITE(
                MCPWM_GEN1_B_REG(PWM0),
                (cActionSetHigh << MCPWM_GEN1_B_UTEA_S)  // LOW quando T=B (decreasing)
                | (cActionSetLow << MCPWM_GEN1_B_DTEA_S) // HIGH quando T=B (increasing)
            );
        }
    }

    // PWM Operator 2
    {
        // Selecionar Timer 2 para o operador 2
        REG_SET_BITS(
            MCPWM_OPERATOR_TIMERSEL_REG(PWM0), 
            2 << MCPWM_OPERATOR2_TIMERSEL_S,    //  0: timer0, 1: timer1, 2: timer2.
            MCPWM_OPERATOR2_TIMERSEL_M          // máscara binária
        );

        // Configurar update method para TEZ
        // When all bits are set to 0: immediately; when bit0 is set to 1: TEZ; 
        // when bit1 is set to 1: TEP; when bit2 is set to 1: sync; 
        // when bit3 is set to 1: disable the update.
        REG_WRITE(
            MCPWM_GEN2_STMP_CFG_REG(PWM0), 
              cSyncAtTEZ << MCPWM_GEN2_A_UPMETHOD_S 
            | cSyncAtTEZ << MCPWM_GEN2_B_UPMETHOD_S
        );

        // Canal A (PWM Operator 2)
        {
            // Configurar razão cíclica
            REG_WRITE(MCPWM_GEN2_TSTMP_A_REG(PWM0), duty);
    
            // Ações a partir de eventos no canal A
            REG_WRITE(
                MCPWM_GEN2_A_REG(PWM0),
                (cActionSetHigh << MCPWM_GEN2_A_DTEA_S)  // LOW quando T=A (decreasing)
                | (cActionSetLow << MCPWM_GEN2_A_UTEA_S) // HIGH quando T=A (increasing)
            );
        }
    

        // Canal B (PWM Operator 2)
        {
            // Configurar razão cíclica
            REG_WRITE(MCPWM_GEN2_TSTMP_B_REG(PWM0), duty);

            // Ações a partir de eventos no canal B
            REG_WRITE(
                MCPWM_GEN2_B_REG(PWM0),
                (cActionSetHigh << MCPWM_GEN2_B_UTEA_S)  // LOW quando T=B (decreasing)
                | (cActionSetLow << MCPWM_GEN2_B_DTEA_S) // HIGH quando T=B (increasing)
            );
        }
    }
}


void pwm_item_1(){
    current_item = ePwmItem_1;

    // Habilitar clock para o periférico PWM0:
    DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_PWM0_CLK_EN);

    DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_PWM0_RST);
    // desabilitar protecao de memoria
    
    const uint8_t cPwmWorkinMode = 3; // 0: freeze, 1: increase mode, 2: decrease mode, 3: up-down mode
    uint16_t duty = cPwmPeriod/2;

    // pré-escala: Period of PWM_clk = 6.25ns * (PWM_CLK_PRESCALE + 1).
    REG_WRITE(MCPWM_CLK_CFG_REG(PWM0), 7<<0); // 50ns
    
    // Portadora vt1 (PWM0, timer 0)
    {
        // Timer period and update method
        REG_WRITE(MCPWM_TIMER0_CFG0_REG(PWM0), 
                (1 << 24)   // PWM_TIMER0_PERIOD_UPMETHOD. 0: immediately, 1: update at TEZ, 2: update at sync, 3: update at TEZ or sync.
            |   (cPwmPeriod << 8) // Period shadow register of PWM timer0 1000 = 500µs/50ns/2  (up e down divide por dois)
            |   (0 << 0)    // timer0 pre-scale
        );

        // Working mode and start/stop control
        REG_WRITE(MCPWM_TIMER0_CFG1_REG(PWM0), 
                (cPwmWorkinMode << 3)  //  PWM timer0 working mode. 0: freeze, 1: increase mode, 2: decrease mode, 3: up-down mode.
            |   (2 << 0)  //  PWM timer0 start and stop control. 2: PWM timer0 starts and runs on
        );
    
        // Synchronization settings
        // Fazer o Timer 0 emitir um sinal de sincronização (para o Timer 1 receber)
        REG_WRITE(MCPWM_TIMER0_SYNC_REG(PWM0), 
                (0 << MCPWM_TIMER0_PHASE_DIRECTION_S)   // 0: increase; 1: decrease.
            |   (0 << MCPWM_TIMER0_PHASE_S)             // Phase for timer reload at sync event. Don't care
            |   (2 << MCPWM_TIMER0_SYNCO_SEL_S)         // PWM timer0 sync_out selection. 0: sync_in; 1: TEZ; 2: TEP; other-wise: sync_out is always 0.
            |   (0 << MCPWM_TIMER0_SYNC_SW_S)           // Toggling this bit will trigger a software sync.
            |   (0 << MCPWM_TIMER0_SYNCI_EN_S)          // When set, timer reloading with phase on sync input event is enabled.
        );

    }

    // Portadora vt2 (PWM0, timer 1)
    {
        // Timer period and update method
        REG_WRITE(MCPWM_TIMER1_CFG0_REG(PWM0), 
                (1 << 24)   // update at TEZ
            |   (cPwmPeriod << 8) // Period shadow register of PWM timer1 2000 = 500µs/50ns
            |   (0 << 0)    // timer1 pre-scale
        );

        // Working mode and start/stop control
        REG_WRITE(MCPWM_TIMER1_CFG1_REG(PWM0), 
                (cPwmWorkinMode << 3)  //  PWM timer1 working mode. 0: freeze, 1: increase mode, 2: decrease mode, 3: up-down mode.
            |   (2 << 0)  //  PWM timer1 start and stop control. 2: PWM timer1 starts and runs on
        );
        
        // Synchronization settings
        REG_WRITE(MCPWM_TIMER1_SYNC_REG(PWM0), 
                (0 << MCPWM_TIMER1_PHASE_DIRECTION_S)   // 0: increase; 1: decrease.
            |   (cPwmPeriod*0 << MCPWM_TIMER1_PHASE_S)  // Phase for timer reload at sync event.
            |   (0 << MCPWM_TIMER1_SYNCO_SEL_S)         // PWM timer1 sync_out selection. 0: sync_in; 1: TEZ; 2: TEP; other-wise: sync_out is always 0.
            |   (0 << MCPWM_TIMER1_SYNC_SW_S)           // Toggling this bit will trigger a software sync.
            |   (1 << MCPWM_TIMER1_SYNCI_EN_S)          // When set, timer reloading with phase on sync input event is enabled.
        );

    }

    // Conectar a saída de sincronização do Timer 0 na entrada de sincronização do Timer 1
    REG_WRITE(MCPWM_TIMER_SYNCI_CFG_REG(PWM0), 
        1 << MCPWM_TIMER1_SYNCISEL_S
    ); // Select sync input for PWM timer1. 1: PWM timer0 sync_out, 2: PWM timer1 sync_out, 3: PWM timer2 sync_out, ...


    const uint8_t cActionSetLow = 1;
    const uint8_t cActionSetHigh = 2;
    // const uint8_t cActionToggle = 3;

    
    // PWM Operator 0
    {
        // Selecionar Timer 0 para o operador 0
        REG_SET_BITS(
            MCPWM_OPERATOR_TIMERSEL_REG(PWM0), 
            0 << MCPWM_OPERATOR0_TIMERSEL_S,    //  0: timer0, 1: timer1, 2: timer2.
            MCPWM_OPERATOR0_TIMERSEL_M          // máscara binária
        );


        // Configurar update method para TEZ
        // When all bits are set to 0: immediately; when bit0 is set to 1: TEZ; 
        // when bit1 is set to 1: TEP; when bit2 is set to 1: sync; 
        // when bit3 is set to 1: disable the update.
        REG_WRITE(
            MCPWM_GEN0_STMP_CFG_REG(PWM0), 
              0b10 << MCPWM_GEN0_A_UPMETHOD_S 
            | 0b10 << MCPWM_GEN0_B_UPMETHOD_S
        );

        // Canal A PWM (Operator 0)
        {
            // Configurar razão cíclica
            REG_WRITE(MCPWM_GEN0_TSTMP_A_REG(PWM0), duty);
    
            // Ações a partir de eventos no canal A
            REG_WRITE(
                MCPWM_GEN0_A_REG(PWM0),
                (cActionSetHigh << MCPWM_GEN0_A_DTEA_S)  // LOW quando T=A (decreasing)
                | (cActionSetLow << MCPWM_GEN0_A_UTEA_S) // HIGH quando T=A (increasing)
            );
        }


        // Canal B (PWM Operator 0)
        {
            // Configurar razão cíclica
            REG_WRITE(MCPWM_GEN0_TSTMP_B_REG(PWM0), duty);

            // Ações a partir de eventos no canal B
            REG_WRITE(
                MCPWM_GEN0_B_REG(PWM0),
                (cActionSetHigh << MCPWM_GEN0_B_UTEA_S)  // LOW quando T=B (decreasing)
                | (cActionSetLow << MCPWM_GEN0_B_DTEA_S) // HIGH quando T=B (increasing)
            );
        }
    }

    {
        // Selecionar Timer 1 para o operador 1
        REG_SET_BITS(
            MCPWM_OPERATOR_TIMERSEL_REG(PWM0), 
            1 << MCPWM_OPERATOR1_TIMERSEL_S,    //  0: timer0, 1: timer1, 2: timer2.
            MCPWM_OPERATOR1_TIMERSEL_M          // máscara binária
        );

        // Configurar update method para TEZ
        // When all bits are set to 0: immediately; when bit0 is set to 1: TEZ; 
        // when bit1 is set to 1: TEP; when bit2 is set to 1: sync; 
        // when bit3 is set to 1: disable the update.
        REG_WRITE(
            MCPWM_GEN1_STMP_CFG_REG(PWM0), 
              0b10 << MCPWM_GEN1_A_UPMETHOD_S 
            | 0b10 << MCPWM_GEN1_B_UPMETHOD_S
        );

        // Canal A (PWM Operator 1)
        {
            // Configurar razão cíclica
            REG_WRITE(MCPWM_GEN1_TSTMP_A_REG(PWM0), duty);
    
            // Ações a partir de eventos no canal A
            REG_WRITE(
                MCPWM_GEN1_A_REG(PWM0),
                (cActionSetHigh << MCPWM_GEN1_A_DTEA_S)  // LOW quando T=A (decreasing)
                | (cActionSetLow << MCPWM_GEN1_A_UTEA_S) // HIGH quando T=A (increasing)
            );
        }
    

        // Canal B (PWM Operator 1)
        {
            // Configurar razão cíclica
            REG_WRITE(MCPWM_GEN1_TSTMP_B_REG(PWM0), duty);

            // Ações a partir de eventos no canal B
            REG_WRITE(
                MCPWM_GEN1_B_REG(PWM0),
                (cActionSetHigh << MCPWM_GEN1_B_UTEA_S)  // LOW quando T=B (decreasing)
                | (cActionSetLow << MCPWM_GEN1_B_DTEA_S) // HIGH quando T=B (increasing)
            );
        }
    }
}


void pll(){
    // TODO:
}

void pwm_task(void* pvParameters){
    ESP_LOGI(TAG, "Iniciando");

    PwmData pwm_data = {};
    pll();
    config_pwm(&pwm_data);
    config_pins();

    const uint32_t cTaskDelay = 60000;
    while (true){
        
        ESP_LOGI(TAG, "Executando (item %i)", current_item);
        vTaskDelay(pdMS_TO_TICKS(cTaskDelay));
    }

    vTaskDelete(NULL); // finalizar tarefa
    ESP_LOGI(TAG, "Finalizando");
}