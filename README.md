# VS1053-ESP32-IDF
Simple VS1053 Driver sample using ESP-IDF for ESP32 SoCs.

```C
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


//From SD Card
f = fopen(MOUNT_POINT"/out2.mp3", "rb");
while(fread(aud_buff, 1, 512, f)) {
    int j = 0;
    for(int i = 512; i >= 32; i = i-32) {
        vs1053_write_sdi(&aud_buff[j], 32);
        j = j+32;
    }
}
fclose(f);

