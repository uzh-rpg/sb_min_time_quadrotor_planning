/*
 * timer.h
 *
 *  Created on: Oct 13, 2020
 *      Author: Robert Penicka
 */

#pragma once

#include <sys/time.h>

#include <cstddef>

class Timer {
 public:
  Timer();
  virtual ~Timer();
  void start();
  void stop();
  void reset();
  long getTimeMS();

 private:
  bool started;
  time_t start_tv_sec;
  suseconds_t start_tv_usec;
  time_t stop_tv_sec;
  suseconds_t stop_tv_usec;
};
