// Please keep these 2 lines at the beginning of each cpp module
static const char* LOG_TAG = "Main";
#define LOG_LOCAL_LEVEL ESP_LOG_INFO


#include "defines.h"

#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/rtc.h"
#include "soc/sens_periph.h"
#include "driver/rtc_io.h"
#include "esp_sleep.h"
#include "sys/time.h"
#include "driver/adc.h"
#include "vs1053.h"
#include "testmp3.h"

#define AMP_SHUT_DOWN_PIN 32 

#define SLEEP_MS(X_MS) vTaskDelay(((X_MS >= 10) ? X_MS : 10) / portTICK_PERIOD_MS)

void app_main(void)
{
    LOG_INFO("Hello from main!");

    UARTInit(UART_NUMBER);



    vs1053_init();

    //Sending test mp3 data
    while (1)
    {
        int j = 0;
        for(int i = outputfile_mp3_len; i > 32; i = i-32) {
            vs1053_write_sdi(&outputfile_mp3[j], 32);
            j = j+32;
        }
        SLEEP_MS(500);
    }
}

