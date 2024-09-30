#pragma once
#ifndef SDU_CONTROLLERS_UTILITY_HPP
#define SDU_CONTROLLERS_UTILITY_HPP

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace sdu_controllers::utils
{
  std::vector<std::vector<std::string>> read_csv(const std::string& filename)
  {
    std::vector<std::vector<std::string>> data;
    std::ifstream file(filename);

    if (!file.is_open())
    {
      std::cerr << "Failed to open file: " << filename << std::endl;
      return data;
    }

    std::string line;
    while (std::getline(file, line))
    {
      std::vector<std::string> row;
      std::stringstream ss(line);
      std::string cell;

      while (std::getline(ss, cell, ','))
      {
        row.push_back(cell);
      }

      data.push_back(row);
    }

    file.close();
    return data;
  }

}  // namespace sdu_controllers::utils

#endif  // SDU_CONTROLLERS_UTILITY_HPP
