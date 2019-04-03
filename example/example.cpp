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
    I2C_Device *NXP   = new I2C_Device(1, NXPS32K148_I2CAddress);

    std::vector<I2C_Device*> I2C_BUS1;
    I2C_BUS1.push_back(Lidar);
    I2C_BUS1.push_back(IMU);
    I2C_BUS1.push_back(NXP);

    LidarLite *lidarLite = new LidarLite();

    int hardwareVersion = lidarLite->getHardwareVersion(*Lidar) ;
    int softwareVersion = lidarLite->getSoftwareVersion(*Lidar) ;
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
            int velocity = lidarLite->getVelocity(Lidar);
            printf("Distance: %5d cm  |  Previous Distance: %5d cm   | Velocity: % 8d \n",distance,previousDistance,velocity);
        }
    }

    delete Lidar;
    delete IMU;

}
