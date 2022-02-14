#include <log4cxx/basicconfigurator.h>
#include <log4cxx/logger.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <csignal>
#include <fstream>
#include <iostream>
#include <string>

#include "common.hpp"

#include "sst.hpp"
#include "timer.hpp"

using namespace log4cxx;

std::string problemFile;

std::string canvasOutput = "";

YAML::Node config;
std::vector<double> start_loaded;
std::vector<double> end_loaded;
std::vector<Point2D> borders_loaded;
std::vector<Point2D> gates_loaded;
std::string world_file;
std::vector<std::vector<double>> array;

SST *singnal_handler_;

void signal_callback_sst(int sig) {
  std::cout << "Signal " << sig << " received" << std::endl;
  singnal_handler_->signal(sig);
  // exit(sig);
}

int test_sst(int argc, char **argv) {
  // register singal for killing
  std::string drone_config_file = "drone.yaml";
  YAML::Node drone_config = YAML::LoadFile(drone_config_file);
  std::string planner_config_file = "sst.yaml";
  YAML::Node planner_config = YAML::LoadFile(planner_config_file);
  SST sst(planner_config, drone_config);
  sst.parse_cmd_args(argc, argv);
  singnal_handler_ = &sst;
  std::signal(SIGINT, signal_callback_sst);
  sst.iterate();
  return 0;
}

int main(int argc, char **argv) {
  startLogger("main");

  seed();

  test_sst(argc, argv);
  return 1;
}
