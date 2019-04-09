#include "NXPs32k148.h"

#include <unistd.h>

NXPs32k148::NXPs32k148(I2C_Device* NXP){
  NXP_ = NXP;
  sending_ = std::thread(&NXPs32k148::send_acceleration_breaking_direction, this);
}
NXPs32k148::~NXPs32k148(){
  delete NXP_;
  kill_i2c_thread = 1;
  sending_.join();
  std::cout << "Count cycles = " << count_cycles << std::endl;
}
void NXPs32k148::set_reference_points(float acc, float dir, float brk){
  mtx.lock();
  acceleration_.flotante = acc;
  direction_.flotante    = dir;
  break_.flotante        = brk;
  mtx.unlock();
  std::cout << "representacion en hexadecimal de :" << acceleration_.flotante << " es :" << std::hex << (int)acceleration_.hex << std::endl;
}
std::uint8_t NXPs32k148::get_n_byte(std::uint32_t un, int pos){
  int ret;
  if(pos < 4){ret = (std::uint8_t)((un >> pos*bits_in_byte) & 0x000000FF);}
  else {ret = 0;}
	return ret;
}

void NXPs32k148::send_acceleration_breaking_direction_one_time(){
    std::vector<std::uint8_t> bytes_a_mandar;
  //mandar datos de i2c
#define BYTES_PER_FLOAT 4
        for(unsigned int data = 0; data < data_to_send.size(); data++){
          for(int i = 0; i < BYTES_PER_FLOAT; i++){
            //convertir info
            bytes_a_mandar.push_back(get_n_byte(data_to_send[data]->hex,i));
            std::cout << "bytes a mandar "<< data*4+i+1 << " = " << std::hex << (int)bytes_a_mandar.at(data*4+i) << std::endl;

          }
        }
        int check = NXP_->write_I2CDevice_block_of_u8(bytes_a_mandar);
        if(check == -1){
          std::cout << "Error en el bus i2c: errno -> " << error << std::endl;
        }
        else {
          std::cout << "Se escribieron correctamente " << check << " bytes a traves de i2c\n";
        }
        bytes_a_mandar.clear();


}

void wait(std::chrono::microseconds period, Clock::time_point& beginning,Clock::time_point& end){
  beginning = Clock::now();
  while(std::chrono::duration_cast<std::chrono::microseconds>(end - beginning).count()  < std::chrono::duration_cast<std::chrono::microseconds>(period).count()){
    usleep(1000);
    end = Clock::now();
  }
}

//Esta función se abre en un hilo
void NXPs32k148::send_acceleration_breaking_direction(){
    std::vector<std::uint8_t> bytes_a_mandar;
    while(1){
		count_cycles++;

  //Reset clock after 10000us duration
        wait(step, time_begin, time_count);
        if(kill_i2c_thread){break;}
  //mandar datos de i2c
//#define BYTES_PER_FLOAT 4
        for(unsigned int data = 0; data < data_to_send.size(); data++){
          for(int i = 0; i < BYTES_PER_FLOAT; i++){
            mtx.lock();
            bytes_a_mandar.push_back(get_n_byte(data_to_send[data]->hex,i));
            mtx.unlock();
          }
        }
        int check = NXP_->write_I2CDevice_block_of_u8(bytes_a_mandar);
        if(check == -1){
          std::cout << "Error en el bus i2c: errno -> " << error << std::endl;
        }
        else {
          std::cout << "Se escribieron correctamente " << check << " bytes a traves de i2c\n";
        }
        bytes_a_mandar.clear();

    }
}
