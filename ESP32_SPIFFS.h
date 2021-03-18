#ifndef ESP32_SPIFFS_h
#define ESP32_SPIFFS_h

#include <SPIFFS.h>

String readFile(fs::FS &fs, const char * path);
void writeFile(fs::FS &fs, const char * path, const char * message);
void appendFile(fs::FS &fs, const char * path, const char * message);


#endif
