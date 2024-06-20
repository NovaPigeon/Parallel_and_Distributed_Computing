#include "ace-route.h"
#include "config.h"
#include <cassert>
#include <filesystem>
#include <iostream>
#include <unistd.h>

#include <log4cplus/configurator.h>
#include <log4cplus/consoleappender.h>
#include <log4cplus/layout.h>
#include <log4cplus/logger.h>
#include <log4cplus/loggingmacros.h>

static log4cplus::Logger logger = log4cplus::Logger::getInstance("ace.main");

int main(int argc, char *argv[]) {
  char *input_netlist = nullptr;
  char *output_netlist = nullptr;
  char *config_file = nullptr;

  while (int opt = getopt(argc, argv, "i:o:c:")) {
    switch (opt) {
    case 'i':
      input_netlist = optarg;
      break;
    case 'o':
      output_netlist = optarg;
      break;
    case 'c': // optional
      config_file = optarg;
      break;
    }
    if (opt == -1) {
      break;
    }
  }
  if (input_netlist == nullptr || output_netlist == nullptr) {
    std::cerr << "Usage: " << argv[0]
              << " -i <input_netlist.phys> -o <output_netlist.phys> [-c "
                 "<config_file>]"
              << std::endl;
    return 1;
  }

  log4cplus::initialize();
  const char *log4cplus_properties = "log4cplus.properties";
  if (std::filesystem::exists(log4cplus_properties)) {
    log4cplus::PropertyConfigurator::doConfigure(log4cplus_properties);
  } else {
    std::cerr << "Failed to load " << log4cplus_properties << std::endl;
    std::cerr << "Loading default configuration..." << std::endl;
    log4cplus::BasicConfigurator::doConfigure();
  }

  auto begin_time = std::chrono::steady_clock::now();
  Config config;
  if (config_file != nullptr) {
    config.parse(config_file);
  }

  AceRoute aceroute(input_netlist, output_netlist, config);

  aceroute.route();

  auto end_time = std::chrono::steady_clock::now();
  LOG4CPLUS_INFO(
      logger,
      "Elapsed time (s): " << std::chrono::duration_cast<std::chrono::seconds>(
                                  end_time - begin_time)
                                  .count());
  log4cplus::Logger::shutdown();

  std::exit(0);
}
