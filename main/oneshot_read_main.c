/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "freertos/queue.h"
#include "driver/gptimer.h"
#include "esp_task_wdt.h"

#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "soc/uart_struct.h"
#include <math.h>

#include "esp_dsp.h"

const static char *TAG = "main";

/*---------------------------------------------------------------
        ADC General Macros
---------------------------------------------------------------*/
//ADC1 Channels
#if CONFIG_IDF_TARGET_ESP32
#define EXAMPLE_ADC1_CHAN0          ADC_CHANNEL_4
#define EXAMPLE_ADC1_CHAN1          ADC_CHANNEL_5
#else
#define EXAMPLE_ADC1_CHAN0          ADC_CHANNEL_0
#define EXAMPLE_ADC1_CHAN1          ADC_CHANNEL_1
#endif


#define EXAMPLE_ADC_ATTEN           ADC_ATTEN_DB_6

#define ADC_LENGTH                  8192

static int adc_raw[ADC_LENGTH];
static int voltage[ADC_LENGTH];
static int um_seg[ADC_LENGTH];
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);

typedef struct {
    uint64_t event_count;
} example_queue_element_t;

static bool IRAM_ATTR example_timer_on_alarm_cb_v1(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;
    QueueHandle_t queue = (QueueHandle_t)user_data;
    // stop timer immediately
    //gptimer_stop(timer);
    // Retrieve count value and send to queue
    example_queue_element_t ele = {
        .event_count = edata->count_value
    };
    xQueueSendFromISR(queue, &ele, &high_task_awoken);
    // return whether we need to yield at the end of ISR
    return (high_task_awoken == pdTRUE);
}

/*---------------------------------------------------------------
        FFT Initialization
---------------------------------------------------------------*/

#define N_SAMPLES 8192
int N = N_SAMPLES;

// Window coefficients
__attribute__((aligned(16)))
float wind[N_SAMPLES];
// working complex array
__attribute__((aligned(16)))
int fft_spectrum[N_SAMPLES*2];
// Pointers to result arrays
float* y1_cf = &fft_spectrum[0];
float* y2_cf = &fft_spectrum[N_SAMPLES];
// Sum of y1 and y2
__attribute__((aligned(16)))
float sum_y[N_SAMPLES/2];


/*---------------------------------------------------------------
        APP Main
---------------------------------------------------------------*/
void app_main(void)
{

    esp_task_wdt_delete(NULL);

    esp_err_t ret;
    ESP_LOGI(TAG, "Start Example.");
    ret = dsps_fft2r_init_sc16(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    if (ret  != ESP_OK)
    {
        ESP_LOGE(TAG, "Not possible to initialize FFT. Error = %i", ret);
        return;
    }

    // Generate hann window
    dsps_wind_hann_f32(wind, N);

    //-------------ADC1 Init---------------//
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = EXAMPLE_ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN1, &config));

    //-------------ADC1 Calibration Init---------------//
    adc_cali_handle_t adc1_cali_chan0_handle = NULL;
    bool do_calibration1_chan0 = example_adc_calibration_init(ADC_UNIT_1, EXAMPLE_ADC1_CHAN1, EXAMPLE_ADC_ATTEN, &adc1_cali_chan0_handle);

    //------------GTimer Init-----------------------------//
    example_queue_element_t ele;
    QueueHandle_t queue = xQueueCreate(10, sizeof(example_queue_element_t));
    if (!queue) {
        ESP_LOGE(TAG, "Creating queue failed");
        return;
    }

    ESP_LOGI(TAG, "Create timer handle");
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1 MHz, 1 tick=1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = example_timer_on_alarm_cb_v1,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, queue));
    ESP_LOGI(TAG, "Enable timer");
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    
    ESP_LOGI(TAG, "Start timer, auto-reload at alarm event");    
    gptimer_alarm_config_t alarm_config1 = {
        .reload_count = 0,
        .alarm_count = 122, // period = 122us
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config1));

    // Inicio del loop principal del programa
    while (1)
    {
        ESP_ERROR_CHECK(gptimer_start(gptimer));
        int record = 8192;

        while(record){
            if (xQueueReceive(queue, &ele, pdMS_TO_TICKS(2000))) {
                ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN1, &adc_raw[8192-record]));
                /*ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN1, adc_raw[0]);
                if (do_calibration1_chan0) {
                    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw[0], &voltage[0]));
                    ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN1, voltage[0]);
                }*/
                record--;
            } else {            
                ESP_LOGW(TAG, "Missed one count event");
            }     
        }

        ESP_LOGI(TAG, "Stop timer");
        ESP_ERROR_CHECK(gptimer_stop(gptimer));

        for(int i=0; i< ADC_LENGTH; i++){
            if (do_calibration1_chan0) {
                //Comentada por que saca en mV con poca resolucion
                //ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw[i], &voltage[i])); 

                //Aquí calculo el valor directamente del valor a raw a microm/seg
                //Se supone que es para el sensor de 2.0 ips, pero el de 1.0 se ve mejor jajaja
                //He comentado la función de calibrar, el valor no es valido pero da el pego

                um_seg[i] = (int) ((adc_raw[i]-617)*14.60); // microm/seg

                //Este printf es para sacar el valor, y se puede plotear en un software llamado "Serial Port Plotter v1.3.0"
                // Enviar los valores por puerto serie necesita muuuucho tiempo, ralentiza el programa.
                
                //printf("$%d;", um_seg[i]);
            }
        }

        /*  @Fran yo pondría aquí tu parte del programa que se ejecucion
        *   En el array um_seg están los valores de aceleración: 8192, tomados cada uno cada 122useg (total 1 seg)
        *   hacer FFT y enviar. Y ya después empieza el bucle de nuevo. Dale si quieres un pequeño delay
        */

        // FFT

        // convertir vector de entrada a un vector complejo (Re [0] +Im[0]+.... Re[N-1]+Im[N-1]), dejando la parte imaginaria a 0
        for (int i=0 ; i< N ; i++)
        {
            fft_spectrum[i*2 + 0] = um_seg[i] * wind[i];
            fft_spectrum[i*2 + 1] = 0;
        }

        //ejecucion fft
        unsigned int start_b = dsp_get_cpu_cycle_count();
        dsps_fft2r_sc16(fft_spectrum, N);
        unsigned int end_b = dsp_get_cpu_cycle_count();
        // Bit reverse 
        #ifdef dsps_bit_rev_sc16
        dsps_bit_rev_sc16(fft_spectrum, N);
        #else
        dsps_bit_rev_sc16_ansi(fft_spectrum, N);  
        #endif
        // Convert one complex vector to two complex vectors
        dsps_cplx2reC_sc16(fft_spectrum, N);

        for (int i = 0 ; i < N/2 ; i++) {
            y1_cf[i] = 10 * log10f((y1_cf[i * 2 + 0] * y1_cf[i * 2 + 0] + y1_cf[i * 2 + 1] * y1_cf[i * 2 + 1])/N);
            y2_cf[i] = 10 * log10f((y2_cf[i * 2 + 0] * y2_cf[i * 2 + 0] + y2_cf[i * 2 + 1] * y2_cf[i * 2 + 1])/N);
            // Simple way to show two power spectrums as one plot
            sum_y[i] = fmax(y1_cf[i], y2_cf[i]);
        }

        // Show power spectrum in 64x10 window from -100 to 0 dB from 0..N/4 samples
        
        ESP_LOGW(TAG, "Signal x1");
        dsps_view(y1_cf, N/2, 64, 10,  -60, 40, '|');
        /*
        ESP_LOGW(TAG, "Signal x2");
        dsps_view(y2_cf, N/2, 64, 10,  -60, 40, '|');
        */
        ESP_LOGW(TAG, "Signals x1 and x2 on one plot");
        //dsps_view_s16(fft_spectrum, N/2, 128, 20,  -60, 40, '|');
        for(int i=0; i< N/8; i++){
              //  printf("$%d;", (int) y1_cf[i]);
            }

        ESP_LOGI(TAG, "FFT for %i complex points take %i cycles", N, end_b - start_b);

        ESP_LOGI(TAG, "End Example.");

        vTaskDelay(1000);
    }
    
   
    


    //Tear Down
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    if (do_calibration1_chan0) {
        example_adc_calibration_deinit(adc1_cali_chan0_handle);
    }

}




/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void example_adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}
