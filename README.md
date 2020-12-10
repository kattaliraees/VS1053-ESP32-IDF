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
