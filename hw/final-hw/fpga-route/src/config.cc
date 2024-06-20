#include "config.h"
#include <fstream>
#include <iostream>

void Config::parse(const std::string &filename) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    std::cerr << "Error: cannot open config file " << filename << std::endl;
    std::exit(1);
  }
  std::string line;
  while (std::getline(file, line)) {
    line.erase(std::remove_if(line.begin(), line.end(), ::isspace), line.end());
    if (line.empty() || line[0] == '#') {
      continue;
    }
    auto pos = line.find('=');
    if (pos == std::string::npos) {
      std::cerr << "Error: invalid line in config file: " << line << std::endl;
      std::exit(1);
    }
    auto key = line.substr(0, pos);
    auto value = line.substr(pos + 1);
    if (setters.find(key) == setters.end()) {
      std::cerr << "Error: invalid key in config file: " << key << std::endl;
      std::exit(1);
    }
    setters[key](value);
  }

  if (connections_log.size() > 0 && use_bipartition_par) {
    std::cerr << "Warning: connections_log is set but use_bipartition_par is "
                 "also set. Disabling use_bipartition_par."
              << std::endl;
    use_bipartition_par = false;
  }
}