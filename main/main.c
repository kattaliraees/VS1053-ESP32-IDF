#include "vs1053.h"
//#include <freertos/task.h>
//#include "testmp3.h"

uint8_t outputfile_mp3_len = 0;
uint8_t outputfile_mp3[] = {0x00, 0x00, 0x00};

void app_main(void)
{    
    vs1053_init();

    vs1053_set_volume(50); //0--100

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

