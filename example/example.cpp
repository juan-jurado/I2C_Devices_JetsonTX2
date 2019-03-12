#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <lidarlite.h>

int getkey() {
    int character;
    struct termios orig_term_attr;
    struct termios new_term_attr;

    /* set the terminal to raw mode */
    tcgetattr(fileno(stdin), &orig_term_attr);
    memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    new_term_attr.c_lflag &= ~(ECHO|ICANON);
    new_term_attr.c_cc[VTIME] = 0;
    new_term_attr.c_cc[VMIN] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

    /* read a character from the stdin stream without blocking */
    /*   returns EOF (-1) if no character is available */
    character = fgetc(stdin);

    /* restore the original terminal attributes */
    tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    return character;
}

int main() {
    // Objects belong to I2C Class
    I2C_Device *Lidar = new I2C_Device(1, kLidarLiteI2CAddress);
    I2C_Device *IMU   = new I2C_Device(1, ACCELEROMETER_I2CAddress);
    
    LidarLite *lidarLite = new LidarLite();
    
    int err = Lidar->open_I2CDevice();
    if (err < 0){
        printf("Error: %d", Lidar->error);
    } else {

        int hardwareVersion = lidarLite->getHardwareVersion(Lidar) ;
        int softwareVersion = lidarLite->getSoftwareVersion(Lidar) ;
        printf("Hardware Version: %d\n",hardwareVersion) ;
        printf("Software Version: %d\n",softwareVersion) ;

        // 27 is the ESC key

        while(Lidar->error >= 0 && getkey() != 27){
            int distance = lidarLite->getDistance(Lidar);
            if (distance < 0) {
                int llError ;
                llError = lidarLite->getError(Lidar) ;
                printf("Lidar-Lite error: %d\n",llError) ;
            } else {
                int previousDistance = lidarLite->getPreviousDistance(Lidar);
                // printf("Distance: %dcm\n", dist);
                int velocity = lidarLite->getVelocity();
                printf("Distance: %5d cm  |  Previous Distance: %5d cm   | Velocity: % 8d \n",distance,previousDistance,velocity);
            }
        }
    }
    lidar -> close_I2CDevice;
    IMU ->close_I2CDevice;
    delete Lidar;
    delete IMU;
    
}
