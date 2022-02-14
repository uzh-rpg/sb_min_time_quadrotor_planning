/*
 * timer.cpp
 *
 *  Created on: Oct 13, 2020
 *      Author: Robert Penicka
 */

#include "timer.hpp"

Timer::Timer() { started = false; }

Timer::~Timer() {}

void Timer::start() {
  started = true;
  struct timeval t;
  gettimeofday(&t, NULL);
  start_tv_sec = t.tv_sec;
  start_tv_usec = t.tv_usec;
}

void Timer::stop() {
  struct timeval t;
  gettimeofday(&t, NULL);
  stop_tv_sec = t.tv_sec;
  stop_tv_usec = t.tv_usec;
  started = false;
}

void Timer::reset() {
  started = true;
  start_tv_sec = 0;
  start_tv_usec = 0;
}

long Timer::getTimeMS() {
  struct timeval t;
  if (!started) {
    t.tv_sec = stop_tv_sec - start_tv_sec;
    t.tv_usec = stop_tv_usec - start_tv_usec;
  } else {
    struct timeval now;
    gettimeofday(&now, NULL);
    t.tv_sec = now.tv_sec - start_tv_sec;
    t.tv_usec = now.tv_usec - start_tv_usec;
  }
  if (t.tv_usec < 0) {
    t.tv_sec -= 1;
    t.tv_usec += 1000000;
  }
  return t.tv_sec * 1000 + t.tv_usec / 1000;
}
