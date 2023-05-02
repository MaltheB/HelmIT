#include "SFE_MicroOLED.h"

// Headers for the functions in HelmIT_func.ino
#ifndef HELMIT_FUNC_H
#define HELMIT_FUNC_H
float I2C_get_temp(uint8_t adr);
double GPS_get_velocity(char* NMEASentence);
void GPS_get_time(char* NMEASentence, char* return_time);
void display_no_strap(MicroOLED oled);
void sonic_request_reading(uint8_t adr, TwoWire);
void change_screen(int gesture, int* screen);
void sonic_get_distance(int, uint16_t*,TwoWire);
void writeClock(char* current_time, MicroOLED oled);
void writeTemp(float temp, MicroOLED oled);
void writeSpeed(double veolocity_float, MicroOLED oled);
void display_too_close(MicroOLED oled);
#endif