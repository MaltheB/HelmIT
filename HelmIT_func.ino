#include "HelmIT_func.h"
#include "SFE_MicroOLED.h"

float I2C_get_temp(uint8_t adr){
    uint8_t buffer[2];  // Buffer for temperature data

    Wire.beginTransmission(adr);  // Start transmission to temperature sensor
    Wire.write(0x0);  // Write wanted register address. To get the wanted data from the sensor
    Wire.endTransmission();  // End transmission

    Wire.requestFrom(adr, 2);  // Request 2 bytes from temperature sensor
    for (int i = 0; i < 2; i++) {  // Read 2 bytes from temperature sensor
        if (Wire.available()) {  // If there is data in the buffer
        buffer[i] = Wire.read();  // Read data from buffer
        }
    }

    float finalTemp;
    int16_t readTemp = (buffer[0] << 8) | buffer[1];  // Combine the two bytes to one 16 bit integer

    if(readTemp < 32768)  // If the temperature is positive
  {
    finalTemp = readTemp * 0.0078125;  // Convert the temperature to degrees Celsius
  }

  if(readTemp >= 32768)  // If the temperature is negative
  {
    finalTemp = ((readTemp - 1) * 0.0078125) * -1;  // Convert the temperature to degrees Celsius
  }

  return finalTemp;  // Return the temperature
}

double GPS_get_velocity(char* NMEASentence){
    char comma_count = 0;
    char velocity[5] = {0};  // Buffer for velocity data
    for(int i = 0; i < 100; i++){  // Loop through NMEA sentence
        if (NMEASentence[i] == ','){  // If comma is found
            comma_count++;  // Increase comma count
        }
        if (comma_count == 7){  // If 7 commas are found
            for(int j = 1; j < 6; j++){  // Loop through velocity array
                velocity[j-1] = NMEASentence[i+j]; // Copy velocity data from the NMEA sentence to the velocity array
            }
            break;
        }
    }
    return strtod(velocity, NULL);  // Convert velocity data to double and return it
}

void GPS_get_time(char* NMEASentence, char* return_time){
    char NMEA_time[9] = {'0','0','0','0','0','0','0','0','0'};  
    for (int i = 0; i < 100; i++){  // Loop through NMEA sentence
        if (*(NMEASentence + i) == ','){  // If comma is found
            for (int j = 1; j < 10; j++){  // Loop through time array
                if (*(NMEASentence + i + j) == ','){  // If comma is found, which means that the NMEA sentence is empty, break loop
                    j = 10;  // Break loop
                }
                else{  // If comma is not found, which means we have a time value
                *(NMEA_time + j - 1) = *(NMEASentence + i + j); // Copy time data from the NMEA sentence to the time array
                }
            }
            break;
        }
    }

    /*
                        The time data is in UTC, which is 2 hours behind Danish time.
            So the code increments the hour by 2, and if the hour is 23 or 24, it increments the day by 1.
    */
    if (NMEA_time[1] >= '8'){  
        NMEA_time[0] += 1;
        if (NMEA_time[1] == 8){
            NMEA_time[1] = '0';
        }
        else{
            NMEA_time[1] = '1';
        }
    }
    else{
        NMEA_time[1] += 2;
    }

    for (int i = 0; i < 2; i++){  
        return_time[i] = NMEA_time[i];  // Copy hour data from the time array to the return time array
        return_time[i+3] = NMEA_time[i+2];  // Copy minute data from the time array to the return time array
        return_time[i+6] = NMEA_time[i+4];  // Copy second data from the time array to the return time array
    }
}

void display_no_strap(MicroOLED oled){
    uint8_t map[] = {
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0x7f, 0x3f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x0f, 0x07, 0x03, 0x01, 0x00, 
	0x00, 0x01, 0x01, 0x03, 0x0f, 0x0f, 0x1f, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x3f, 0x1f, 0x0f, 
	0x07, 0x03, 0x81, 0xc0, 0xe0, 0xf0, 0xf8, 0xfc, 0xfc, 0xfc, 0xf8, 0xf0, 0xe0, 0xc0, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0xe0, 0xf0, 0xf8, 0xfc, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x7f, 0x7f, 0x7c, 0xf0, 0xe0, 0xc0, 0x80, 
	0x00, 0x00, 0x01, 0x03, 0x07, 0x0f, 0x1f, 0x3f, 0x3f, 0x1f, 0x0f, 0x07, 0x03, 0x81, 0xc0, 0xe0, 
	0xf0, 0xf8, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x3f, 0x1f, 0x0f, 
	0x07, 0x03, 0x81, 0xc1, 0xc3, 0x83, 0x01, 0x00, 0x08, 0x1c, 0x3c, 0x78, 0xf0, 0x61, 0x03, 0x07, 
	0x0f, 0xfe, 0xfc, 0xf8, 0xf0, 0xf0, 0xe0, 0xe0, 0xf0, 0xf8, 0xfc, 0xfe, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x3f, 0x1f, 0x0f, 0x06, 0x00, 0x00, 0x00, 
	0x00, 0x01, 0x03, 0x07, 0x0f, 0x1f, 0x3f, 0x7e, 0x3c, 0x18, 0x00, 0x00, 0x00, 0x80, 0xcc, 0xfe, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xf8, 0xf0, 0xf0, 0xc0, 0x80, 0x80, 0x00, 
	0x00, 0x80, 0xc0, 0xe0, 0xf0, 0xf8, 0xf8, 0xf0, 0xf0, 0xf8, 0xfc, 0xfe, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
    /*
                                Above is a bitmap of the no strap warning sign. 
                Which is simply displayed on the screen when the strap is not on the user's wrist.
    */
    oled.clear(PAGE);
    oled.clear(ALL);
    oled.setCursor(0,0);
    oled.drawBitmap(map);
    oled.display();
    }

void sonic_request_reading(uint8_t adr, TwoWire Wire){
    Wire.beginTransmission(adr);  // Begin transmission to the sonic sensor
    Wire.write(0b01010001);     // Send a request for a reading
    Wire.endTransmission();   // End transmission
}

void change_screen(int gesture, int* screen){
    switch(gesture){  // Switch statement to change the screen based on the gesture
        case 1:  // If the gesture is 1, then the screen is set to 0 (turned off)
            *screen = 0;
            break;

        case 2:  // If the gesture is 2, then the screen is set to 1 (the main screen)
            *screen = 1;
            break;

        case 3:  // If the gesture is 3, then the screen is incremented by 1
            if (*screen == 0){
            }
            else{  
                *screen += 1;
                if (*screen > 3){  // If the screen is greater than 3, then it is set to 1, to loop back to the main screen
                    *screen = 1;
            }
            }
            break;

        case 4:  // If the gesture is 4, then the screen is decremented by 1
            if (*screen == 0){
            }
            else{
                *screen -= 1;
                if (*screen < 1){  // If the screen is less than 1, then it is set to 3, to loop back to the last screen
                    *screen = 3;
            }
            }
            break;
    }
}

void sonic_get_distance(int adr, uint16_t* distance, TwoWire Wire){
    Wire.requestFrom(adr, 2);  // Request 2 bytes from the sonic sensor
    if(Wire.available()){  // If the wire is available, then the distance is set to the 2 bytes received
        *distance = (Wire.read() << 8) | Wire.read();  // The distance is set to the 2 bytes received
    }
}

void writeClock(char* current_time, MicroOLED oled){
    oled.setFontType(1);  // Set font to type 1
    for (int i = 0; i<5; i++) {  // For loop to write the time to the screen
        oled.setCursor(12+7*i,20);
        oled.print(current_time[i]);
    }
}

void writeTemp(float temp, MicroOLED oled){    
    // Function to write the temperature to the screen
    oled.setFontType(0);
    oled.setCursor(17, 7);
    oled.print("Helmet");
    oled.setFontType(1);
    oled.setCursor(12, 18);
    oled.print(temp,2);
    oled.setCursor(30, 30);
    oled.print("C");
}

void writeSpeed(double velocity_float, MicroOLED oled){
  // Function to write the speed to the screen
  oled.setFontType(1);
  oled.setCursor(10, 15);
  oled.print(velocity_float,2);
  oled.setCursor(14, 27);
  oled.print("km/h");
}

void display_too_close(MicroOLED oled){
    // Function to display the too close warning border
    uint8_t map[] = {   0x00, 0xE0, 0xF8, 0xFC, 0x1C, 0x0E, 0x0E, 0x06, 0x02, 0x08,
                        0x0C, 0x0E, 0x06, 0x02, 0x08, 0x0C, 0x0E, 0x06, 0x02, 0x08,
                        0x0C, 0x0E, 0x06, 0x02, 0x08, 0x0C, 0x0E, 0x06, 0x02, 0x08,
                        0x0C, 0x0E, 0x06, 0x02, 0x08, 0x0C, 0x0E, 0x06, 0x02, 0x08,
                        0x0C, 0x0E, 0x06, 0x02, 0x08, 0x0C, 0x0E, 0x06, 0x02, 0x08,
                        0x0C, 0x0E, 0x0E, 0x0E, 0x0E, 0x1C, 0xFC, 0xF8, 0xE0, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x99, 0xD3, 0xC0, 0xC0,
                        0xC0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xC0, 0xC0,
                        0xDC, 0x99, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0x3F, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0x3F, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0xC3, 0xE7, 0xE7, 0xC3, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0xC3, 0xE7, 0xE7, 0xC3, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC8, 0x98, 0x38,
                        0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x01, 0x01, 0xC8, 0x98, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x07, 0x1F, 0x3F, 0x38, 0x70, 0x70, 0x70, 0x30, 0x10,
                        0x40, 0x60, 0x70, 0x30, 0x10, 0x40, 0x60, 0x70, 0x30, 0x10,
                        0x40, 0x60, 0x70, 0x30, 0x10, 0x40, 0x60, 0x70, 0x30, 0x10,
                        0x40, 0x60, 0x70, 0x30, 0x10, 0x40, 0x60, 0x70, 0x30, 0x10,
                        0x40, 0x60, 0x70, 0x30, 0x10, 0x40, 0x60, 0x70, 0x30, 0x10,
                        0x40, 0x60, 0x70, 0x70, 0x78, 0x38, 0x3F, 0x1F, 0x07, 0x00,
                        0x00, 0x00, 0x00, 0x00};
    oled.drawBitmap(map);
}