#include "lidarlite.h"
#include <stdexcept>
#include <chrono>
typedef std::chrono::high_resolution_clock Clock;

// Interface for Lidar-Lite V2 (Blue Label) with NVIDIA Jetson TK1
//2.000000

I2C_Device::I2C_Device(unsigned char _kI2CBus, char _I2CDevice_Address){
    kI2CBus = _kI2CBus;                         // Desired I2C bus on Jetson TX2
    I2CDevice_Address = _I2CDevice_Address;     // Desired I2C Address on Device
    error = 0 ;
    error = this->open_I2CDevice();
    if(error < 0){
      //STOPS EXECUTION
      std::string errorMessage = std::string("Error: on opening device address")+I2CDevice_Address;
      throw std::runtime_error(errorMessage);
    }
}
I2C_Device::~I2C_Device(){
    close_I2CDevice();
}
// Returns true if device file descriptor opens correctly, false otherwise
bool I2C_Device::open_I2CDevice(){
    char fileNameBuffer[32];
    sprintf(fileNameBuffer,"/dev/i2c-%d", kI2CBus);
    I2C_FileDescriptor = open(fileNameBuffer, O_RDWR);
    if (I2C_FileDescriptor < 0) {
        // Could not open the file
        error = errno ;
        return false ;
    }
    if (ioctl(I2C_FileDescriptor, I2C_SLAVE, I2CDevice_Address) < 0) {
        // Could not open the LIDAR on the bus
        error = errno ;
        return false ;
    }
    return true ;
}
void I2C_Device::close_I2CDevice(){
    if (I2C_FileDescriptor > 0) {
        close(I2C_FileDescriptor);
        // WARNING - This is not quite right, need to check for error first
        I2C_FileDescriptor = -1 ;
    }
}
// Read the given register on the Lidar-Lite
int I2C_Device::read_I2CDevice(int readRegister){
    // Do not use i2c_smbus_read_byte_data here ; LidarLite V2 needs STOP between write and read
    int toReturn ;
    toReturn = i2c_smbus_write_byte(I2C_FileDescriptor, readRegister) ;
    if (toReturn < 0) {
        error = errno ;
        toReturn = -1 ;
    }
    toReturn = i2c_smbus_read_byte(I2C_FileDescriptor) ;
    if (toReturn < 0) {
        error = errno ;
        toReturn = -1 ;
    }
    return toReturn ;
}
// Write the the given value to the given register on the Lidar-Lite
int I2C_Device::write_I2CDevice(int writeRegister, int writeValue){
    int toReturn = i2c_smbus_write_byte_data(I2C_FileDescriptor, writeRegister, writeValue);
    //i2c_smbus_write_byte() to write single byte.
    // Wait a little bit to make sure it settles
    //usleep(10000);
    if (toReturn < 0) {
        error = errno ;
        toReturn = -1 ;
    }
    return toReturn ;
}
int I2C_Device::write_I2CDevice_block_of_u8(std::vector<std::uint8_t> bloques){
    //int i2c_master_send               (const struct i2c_client *client, const char *buf, int count)
    //s32 i2c_smbus_write_block_data    (const struct i2c_client *client, u8 command, u8 length, const u8 *values)
    //s32 i2c_smbus_write_i2c_block_data(const struct i2c_client *client, u8 command, u8 length, const u8 *values)
    int toReturn = i2c_master_send(I2C_FileDescriptor, (const char*)bloques.begin(), bloques.size());
    if(toReturn < 0){
      error = errno;
      toReturn = -1;
    }
    return toReturn;
}

LidarLite::LidarLite(){
	error = 0;
}
LidarLite::~LidarLite(){
  delete Lidar_;
}
LidarLite::LidarLite(I2C_Device* Lidar){
	error = 0;
  Lidar_ = Lidar;

}
// Return the current calculated distance in centimeters
int LidarLite::getDistance(){
    int ioResult ;
    int msb, lsb ;
    ioResult = Lidar_->write_I2CDevice(kLidarLiteCommandControlRegister,kLidarLiteMeasure);
    if (ioResult < 0) {
        return ioResult ;
    }
    ioResult = Lidar_->read_I2CDevice(kLidarLiteCalculateDistanceMSB);
    if (ioResult < 0) {
        return ioResult ;
    } else {
        msb = ioResult ;
    }
    ioResult = Lidar_->read_I2CDevice(kLidarLiteCalculateDistanceLSB);
    if (ioResult < 0) {
        return ioResult ;
    } else {
        lsb = ioResult ;
    }

    int distance = (msb << 8) + lsb ;

    return distance ;
}
// Return the previous measurement in centimeters
int LidarLite::getPreviousDistance() {

    int ioResult ;
    int msb, lsb ;
    ioResult = Lidar_->read_I2CDevice(kLidarLitePreviousMeasuredDistanceMSB);
    if (ioResult < 0) {
        return ioResult ;
    } else {
        msb = ioResult ;
    }
    ioResult = Lidar_->read_I2CDevice(kLidarLitePreviousMeasuredDistanceLSB);
    if (ioResult < 0) {
        return ioResult ;
    } else {
        lsb = ioResult ;
    }

    int distance = (msb << 8) + lsb ;

    return distance ;
}
// Return the velocity (rate of change) in centimeters; +/-
// Velocity is returned from the Lidar-Lite as an 8-bit 2's complement number
// The returned value is converted to a signed integer
int LidarLite::getVelocity(){
    int ioResult = Lidar_->read_I2CDevice(kLidarLiteVelocityMeasurementOutput);
    if (ioResult == 255) {
        return 0 ;
    }
    if (ioResult > 127) {

        return  ioResult - 256 ;
    }
    return ioResult ;
}
// Return the Lidar-Lite hardware version
int LidarLite::getHardwareVersion(){
    return Lidar_->read_I2CDevice(kLidarLiteHardwareVersion) ;
}
// Return the Lidar-Lite software version
int LidarLite::getSoftwareVersion() {
    return Lidar_->read_I2CDevice(kLidarLiteSoftwareVersion) ;
}
// Return the last i/o error
int LidarLite::getError(){
    return Lidar_->error ;
}

NXPs32k148::NXPs32k148(I2C_Device* NXP){
  NXP_ = NXP;
  sending_ = std::thread(&NXPs32k148::send_acceleration_breaking_direction, this);
  time10ms_count_ = Clock::now();
}
NXPs32k148::~NXPs32k148(){
  delete NXP_;
  kill_i2c_thread = 1;
  sending_.join();
}
void NXPs32k148::set_reference_points(float acc, float dir, float brk){
  acceleration_->flotante = acc;
  direction_->flotante    = dir;
  break_->flotante        = brk;
}
std::uint8_t NXPs32k148::get_n_byte(std::uint32_t un, int pos){
  int ret;
  if(pos > 3){ret = (std::uint8_t)((un >> pos*bits_in_byte) & 0x000000FF);}
  else {ret = 0;}
	return ret;
}
std::uint8_t NXPs32k148::get_n_byte(std::uint32_t un, int pos){
	return (std::uint8_t)((un >> pos*bits_in_byte) & 0x000000FF);
}
//abrir esta funcion en un hilo
void NXPs32k148::send_acceleration_breaking_direction(){
    std::vector<std::uint8_t> bytes_a_mandar;
    while(1){
        while((time10ms_count_.time_since_epoch()) < std::chrono::nanoseconds(10000000)){}
  //Reset clock after 10000us duration
        time10ms_count_ = Clock::now();
        if(kill_i2c_thread){break;}
  //mandar datos de i2c
#define BYTES_PER_FLOAT 4
        for(int data = 0; data < data_to_send.size(); data++){
          for(int i = 0; i < BYTES_PER_FLOAT; i++){
            bytes_a_mandar.push_back(get_n_byte(data_to_send[data]->hex,i));
          }
        }
        NXP_->write_I2CDevice_block_of_u8(bytes_a_mandar);

    }
}
