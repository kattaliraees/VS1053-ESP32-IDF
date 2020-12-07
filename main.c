#include "esp_system.h"
#include "vs1053.h"
#include "testmp3.h"

#define SLEEP_MS(X_MS) vTaskDelay(((X_MS >= 10) ? X_MS : 10) / portTICK_PERIOD_MS)

void app_main(void)
{    
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

