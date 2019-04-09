#ifndef NXPS32K148_H
#define NXPS32K148_H

#include <vector>
#include <thread>
#include <pthread.h>
#include <chrono>
//thread safe types
#include <mutex>
#include "I2C_Device.h"

typedef std::chrono::high_resolution_clock Clock;

#define bits_in_byte 8

union float_to_hex {
  float flotante;
  std::uint32_t hex;
};

class NXPs32k148
{
public:
  int error;
  NXPs32k148(I2C_Device* NXP);
  ~NXPs32k148();
  /**Manda la información cada 10ms*/

  void set_reference_points(float acc, float dir, float brk);
  void send_acceleration_breaking_direction_one_time();
private:
  std::mutex mtx;
  std::uint8_t get_n_byte(std::uint32_t un, int pos);
  Clock::time_point time_count, time_begin;
  std::thread sending_;
  void send_acceleration_breaking_direction();
  I2C_Device* NXP_;
  //Se usa la union para estar checando constantemente el numero ingresado al float pero en hexadecimal.
  float_to_hex acceleration_  = {0.0};
  float_to_hex direction_     = {0.0};
  float_to_hex break_         = {0.0};
  //Mete los 3 floats dentro del arreglo
  std::array<float_to_hex*,3> data_to_send = {{{&direction_},{&break_},{&acceleration_}}};
  bool kill_i2c_thread = 0;
  double count_cycles = 0;
  std::chrono::microseconds step = std::chrono::microseconds(10003);
};

#endif
