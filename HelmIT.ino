#include "HelmIT_func.h"  // Header file for HelmIT_func.ino
#include "wiring_private.h"  // For pinPeripheral() function
#include "SFE_MicroOLED.h"  // Library for the OLED
#include "SparkFun_APDS9960.h"  // Library for the gesture sensor
#include "SAMD51_InterruptTimer.h"  // Library for the timer interrupt
#include "Wire.h"  // Library for I2C

// Pin definitions
#define DC_JUMPER 1
#define SOI_SIZE 6  // Size of the sentence of interest (SOI) array
#define PIN_RESET 9  // Reset pin for the OLED
#define APDS9960_INT 10  // Interrupt pin for the gesture sensor
#define TX_pin 11  // TX pin for the GPS
#define RX_pin 13  // RX pin for the GPS
#define STRAP A0  // Pin for the strap sensor
#define SERIAL_BAUD 115200  // Baud rate for the serial monitor
#define temp_address 0x48  // I2C address for the temperature sensor
#define sonic_address 0x70  // I2C address for the ultrasonic sensor

// Global variables GPS
char idx = 0;
char NMEASentence_GPVTG[100] = {0};  // NMEA sentence for velocity
char NMEASentence_GPGGA[100] = {0};  // NMEA sentence for time
bool GPVTG_flag = 0;  // Flag for GPVTG sentence
bool GPGGA_flag = 0;  // Flag for GPGGA sentence
char SOI_GPS_GPVTG[SOI_SIZE] = {'$','G','P','V','T','G'};  // Sentence of interest (SOI) for GPS
char SOI_GNSS_GPVTG[SOI_SIZE] = {'$','G','N','V','T','G'};  // SOI for GNSS
char SOI_GLONASS_GPVTG[SOI_SIZE] = {'$','G','L','V','T','G'};  // SOI for GLONASS
char SOI_GALILEO_GPVTG[SOI_SIZE] = {'$','G','A','V','T','G'};  // SOI for GALILEO
char SOI_GPS_GPGGA[SOI_SIZE] = {'$','G','P','G','G','A'};  // SOI for GPS
char SOI_GNSS_GPGGA[SOI_SIZE] = {'$','G','N','G','G','A'};  // SOI for GNSS
char SOI_GLONASS_GPGGA[SOI_SIZE] = {'$','G','L','G','G','A'};  // SOI for GLONASS
char SOI_GALILEO_GPGGA[SOI_SIZE] = {'$','G','A','G','G','A'};  // SOI for GALILEO
char* SOI[8] = {SOI_GPS_GPVTG, SOI_GNSS_GPVTG, SOI_GLONASS_GPVTG, SOI_GALILEO_GPVTG,
                SOI_GPS_GPGGA, SOI_GNSS_GPGGA, SOI_GLONASS_GPGGA, SOI_GALILEO_GPGGA};  // Array of SOIs
double velocity_float = 0.0;  // Velocity in km/h
char current_time[8] = {'0', '0', ':', '0', '0', ':', '0', '0'};  // Current time

// Other global variables
bool strap_flag = 0;  // Flag for strap sensor
float helm_temp = 0.0;  // Temperature inside of the helm
bool gesture_flag = 0;  // Flag for gesture sensor input
uint16_t distance = 300;  // Distance from ultrasonic sensor in cm for blind spot detection
int screen = 1;  // Screen number 
bool sonic_flag = 0;  // Flag for ultrasonic sensor read request
bool sonic_read_ready = 0;  // Flag for ultrasonic sensor read ready

// GPS UART
Uart myUART(&sercom1, RX_pin, TX_pin, SERCOM_RX_PAD_1, UART_TX_PAD_0);  // UART object for GPS

// Initialize objects
MicroOLED oled(PIN_RESET, DC_JUMPER);  // OLED object
SparkFun_APDS9960 gesture_sensor = SparkFun_APDS9960();  // Gesture sensor object

void setup(){
    Serial.begin(SERIAL_BAUD); // Start serial monitor
    myUART.begin(9600);  // Start GPS UART
    Wire.begin();  // Start I2C
    oled.begin();  // Start OLED
    oled.clear(ALL);  // Clear OLED
    oled.clear(PAGE);  // Clear OLED
    oled.flipVertical(true);  // Flip OLED
    oled.setCursor(0, 0);  // Set cursor to top left corner
    oled.display();

    pinPeripheral(TX_pin, PIO_SERCOM);  // Set TX pin to SERCOM
    pinPeripheral(RX_pin, PIO_SERCOM);  // Set RX pin to SERCOM

    pinMode(STRAP, INPUT);  // Set strap pin to input
    pinMode(APDS9960_INT, INPUT);  // Set gesture sensor interrupt pin to input
    attachInterrupt(digitalPinToInterrupt(APDS9960_INT), gesture_ISR, FALLING);  // Attach interrupt to gesture sensor interrupt pin
    gesture_sensor.init();  // Initialize gesture sensor
    gesture_sensor.enableGestureSensor(true);  // Enable gesture sensor
    delay(5000);  // Wait for GPS to start
}

/*
This section of code runs continuesly like a While(true)
*/
void loop(){
    // Check if strap is clipped in and display no strap warning if not clipped in
    while(digitalRead(STRAP)){  // While strap is not clipped in
        display_no_strap(oled);
        delay(700);
        oled.clear(ALL);
        oled.clear(PAGE);
        oled.display();
        delay(300);
        Serial.println("No strap");
    }
    oled.clear(ALL);
    oled.clear(PAGE);
    /* 
        Update screen depending on the screen number and display warning if there
                            is something in the blind spot
    */
    if (screen){
        if (distance < 150){  // If something is in the blind spot
            display_too_close(oled);  // Display warning
        }
        switch (screen){  // Display screen depending on screen number
            case 1:
                writeClock(current_time, oled);  // Write clock to OLED
                break;
            case 2:
                writeSpeed(velocity_float, oled);  // Write speed to OLED
                break;
            case 3:
                writeTemp(helm_temp, oled);  // Write temperature to OLED
                break;
        }
        oled.display();
    }
    GPS_get_time(NMEASentence_GPGGA, current_time);  // Get current time from GPS
    velocity_float = GPS_get_velocity(NMEASentence_GPVTG);  // Get velocity from GPS
    helm_temp = I2C_get_temp(temp_address);  // Get temperature from helm
    if (!sonic_flag){  // If ultrasonic sensor hasn't been requested to read
        sonic_request_reading(sonic_address, Wire);  // Request ultrasonic sensor to read
        TC.startTimer(150000, sonic_ISR);  // Start timer for ultrasonic sensor read (150ms)
        sonic_flag = 1; 
    }
    if (sonic_read_ready){  // If ultrasonic sensor has have distance ready
        sonic_get_distance(sonic_address, &distance, Wire);  // Get distance from ultrasonic sensor
        sonic_read_ready = 0;  // Reset ultrasonic sensor read ready flag
        sonic_flag = 0;  // Reset ultrasonic sensor read request flag
    }
    if (gesture_flag){  // If gesture sensor has detected a gesture
        detachInterrupt(digitalPinToInterrupt(APDS9960_INT));  // Detach interrupt from gesture sensor interrupt pin 
        if (gesture_sensor.isGestureAvailable()) {  // If gesture sensor has a gesture available 
            int gesture = gesture_sensor.readGesture();  // Read gesture from gesture sensor
            change_screen(gesture, &screen);  // Change screen depending on gesture
        }
        gesture_flag = 0;
        attachInterrupt(digitalPinToInterrupt(APDS9960_INT), gesture_ISR, FALLING);  // Attach interrupt to gesture sensor interrupt pin again
    }
}

void SERCOM1_0_Handler() {  // GPS UART interrupt handler
    myUART.IrqHandler();
}

void SERCOM1_1_Handler() {  // GPS UART interrupt handler
    myUART.IrqHandler();
}

void SERCOM1_2_Handler() {  // GPS UART interrupt handler
    myUART.IrqHandler();

    // Get only $GPVTG and $GPGGA sentence
    while(myUART.available()) { // While there is data in the UART buffer
        bool check = 0;  // Flag for checking if the data is $GPVTG or $GPGGA
        char data = myUART.read();  // Read data from UART buffer
        if (idx >= SOI_SIZE){  // If we have found a SOI (sentence of interest)
            if (data == '$'){  // If data is $, reset index and flags and start over
                GPVTG_flag = 0;
                GPGGA_flag = 0;
                idx = 1;
            }
            else if (GPVTG_flag){  // If data is $GPVTG data, store it in the GPVTG array
                NMEASentence_GPVTG[idx] = data;
                idx++;
            }
            else if (GPGGA_flag){  // If data is $GPGGA data, store it in the GPGGA array
                NMEASentence_GPGGA[idx] = data;
                idx++;
            }
        }
        else{  // If we haven't found a SOI yet
            idx++;  // Increment index
            if (data == *(SOI[0] + idx - 1) || data == *(SOI[1] + idx - 1) ||
                    data == *(SOI[2] + idx - 1) || data == *(SOI[3] + idx - 1)){  // If data is $GPVTG data, store it in the GPVTG array
                NMEASentence_GPVTG[idx - 1] = data;
                if (idx == SOI_SIZE){  // If we have found a SOI (sentence of interest)
                    GPVTG_flag = 1;  // Set GPVTG flag
                }
                check = 1;
            }
            if (data == *(SOI[4] + idx - 1) || data == *(SOI[5] + idx - 1) ||
                    data == *(SOI[6] + idx - 1) || data == *(SOI[7] + idx - 1)){  // If data is $GPGGA data, store it in the GPGGA array
                NMEASentence_GPGGA[idx - 1] = data;
                if (idx == SOI_SIZE){  // If we have found a SOI (sentence of interest)
                    GPGGA_flag = 1;  // Set GPGGA flag
                }
                check = 1;
            }
            else if (!check){  // If data is not $GPVTG or $GPGGA data, reset index and flags and start over
                GPVTG_flag = 0;
                GPGGA_flag = 0;
                idx = 0;
            }
        }
    }
}

void SERCOM1_3_Handler() {  // GPS UART interrupt handler
    myUART.IrqHandler();
}

void gesture_ISR(){  // Gesture sensor interrupt handler
    gesture_flag = 1;
}

void sonic_ISR(){  // Ultrasonic sensor interrupt handler
    TC.stopTimer();  // Stop timer
    sonic_read_ready = 1;
}