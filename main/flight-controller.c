#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "soc/gpio_struct.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/spi_common.h"
#include "driver/i2c.h"
#include <math.h>
#include <driver/adc.h>

#define QMC5883L_I2C_ADDR     0x0d //hmc5883 i2c address
#define i2c_frequency       500000 // max frequency of i2c clk
#define i2c_port                 0 //i2c channel on ESP-WROOM-32 ESP32S
#define i2c_gpio_scl            19 //D19 on ESP-WROOM-32 ESP32S
#define i2c_gpio_sda            18 //D18 on ESP-WROOM-32 ESP32S
#include "./interfaces/i2c.c"
#include "./functionc/qmc5883l.c"

#define BMP280_I2C_ADDR       0x76
int digT1, digT2, digT3;
int digP1, digP2, digP3, digP4, digP5, digP6, digP7, digP8, digP9;
#include "./functionc/bmp280.c"

//requirements for mcpwm
#include "esp_attr.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#define GPIO_MCPWM0A   21       //Motor1
#define GPIO_MCPWM0B   25       //Motor2
#define GPIO_MCPWM1A   32       //Motor3
#define GPIO_MCPWM1B   33       //Motor4
#include "./interfaces/esp_perif.c"

//requirements for hspi attached to nrf24l01
#define HSPI_MISO_PIN  14       //ad0
#define HSPI_MOSI_PIN  27       //sda
#define HSPI_SCLK_PIN  12       //scl
#define HSPI_CS_PIN    26       //ncs
#define HSPI_SPI_CLOCK 1000000  //1MHz 
#include "./interfaces/hspi.c"

#define NRF24L01_CE    13
#include "./functionc/nrf24l01.c"
//#include "./tasks/transmiter_comms.c"

//requirements for vspi attached to mpu9250
#define VSPI_MISO_PIN  17       //ad0
#define VSPI_MOSI_PIN  5        //sda
#define VSPI_SCLK_PIN  23       //scl
#define VSPI_CS_PIN    16       //ncs
#define VSPI_SPI_CLOCK 10000000  // 10 MHz - must be >> 1M for 8ksamp/sec
#include "./interfaces/vspi.c"
#include "./tasks/imu.c"

//globals for control and command
int   seq, throttle=1000, yaw, pitch, roll, state;
int   clearInts = 0, cal_cnt = 0, astate = 0, calib = 0, nsamp = 0;

//pid globals shared between pid loops and debug
float xPIDout, yPIDout, zPIDout, aPIDout;
float xSig, xErr, xPgain = 0.50, xIgain = 0.033, xDgain = 60, xInt = 0, xDer, xLast = 0; //pitch
float ySig, yErr, yPgain = 0.50, yIgain = 0.033, yDgain = 60, yInt = 0, yDer, yLast = 0; //roll
float zSig, zErr, zPgain = 0.00, zIgain = 0.000, zDgain = 60, zInt = 0, zDer, zLast = 0; //yaw
float aSig, aErr, aPgain = 0.00, aIgain = 0.000, aDgain =  0, aInt = 0, aDer, aLast = 0; //altitude

//globals for attitude control
int Motor1=1000, Motor2=1000, Motor3=1000, Motor4=1000;
float height = 3.72; float heightprog= -5.3; float height_cal = 0;
float heading= 14.5; float headingprog= 333;
float xdisp = -1.6; float ydisp = -0.1;
#include "./tasks/attitude_control.c"

//debugging and monitoring interface
char  blackbox_str[256];
#include "./functionc/blackbox.c"
char col = 0; int cntp =0; int cal=0; char par = 'p'; float amt = 0.1; char axis = 'x'; int lastsamp = 0;
#include "./functionc/debug_interface.c"

void transmitter_comms () {
    //wait for packet from transmitter - if no packets over 10 timeouts land
    int timeout = 40; int waitcnt;
    uint8_t data[32];
    while(1) {
        waitcnt = nrf24_receive_pkt (data, timeout);
        if (waitcnt < timeout){
           //printf("        ");for(int a=0; a<12;a++)printf("%3d ",data[a]);printf("\n");
           printf("%8.4f   waited %3dmsec   ", (float)esp_timer_get_time()/1000000, 10*waitcnt);
           seq =      256 * data[0] + data[1];
           throttle = 256 * data[2] + data[3];
           yaw =      256 * data[4] + data[5];
           pitch =    256 * data[6] + data[7];
           roll =     256 * data[8] + data[9];
           state =    256 * data[10] + data[11];
           //sscanf((char*)data,"%d,%d,%d,%d,%d,%d",&seq, &throttle, &yaw, &pitch, &roll, &state);

           if(yaw>=1000&&yaw<=2000)headingprog = headingprog + (float)(yaw - 1500) / 400;
           if(throttle>=1000&&throttle<=2000)heightprog = heightprog + (float)(throttle - 1500) / 2000;
           printf("seq = %5d %4d %4d %4d %4d st%d  %5.2f %5.2f\n", seq,throttle,yaw,pitch,roll,state,
                                                     heightprog,headingprog);
        } 
        else { printf("no packet = timed out at %dmsec\n", 10*waitcnt); }

        //send blackbox data to transmitter
        if (waitcnt < timeout){ 
               blackbox_string(); 
               nrf24_transmit_pkt ((uint8_t*)blackbox_str, 32); }

        //vTaskDelay(20/portTICK_RATE_MS); //should keep up with tranmitter commands 
    }
}

void app_main() {
    nvs_flash_init();
     
    vspi_init();         //spi for imu
    imu_init (vspi); 
    vTaskDelay(50/portTICK_RATE_MS);  

    hspi_init();         //spi for radio
    nrf24_gpio_init();

    escint_init();       //pwm servo for esc to motors

    //i2cdetect();
    //bmp280_cal();       //air pressure altitude measurements
    //qmc5883_init();     //magnetometer heading measurements

    //start tasks
    xTaskCreatePinnedToCore (imu_read, "imu_read", 8096, NULL, 5, NULL, 1);
    vTaskDelay(1);
    xTaskCreatePinnedToCore (attitude_control, "attitude_control", 4096, NULL, 4, NULL, 0);
    vTaskDelay(1);
    xTaskCreatePinnedToCore (transmitter_comms, "transmitter_comms", 4096, NULL, 4, NULL, 0);

    int cnt = 0;
    while(1){
       //read bmp280 and qmc5883 - do this here other task too fast for bmp280
       //if (cnt == 10) height_cal =  bmp280_read();
       //height = 28.4*(height_cal - bmp280_read();
       //heading = qmc5883_read();

       //printf("%5d height=%3d heading=%3d deg    imu.pitch=%7.2f   imu.mroll=%7.2f\n", 
       //       cnt, (uint8_t)height, (uint8_t)heading, imu.pitch, imu.roll);

       //debug_interface();
       vTaskDelay(1000/portTICK_RATE_MS);  
       ++cnt;
    }
    removeDevice(vspi);
    removeDevice(hspi);
    vTaskDelay(1);
}
