#include "vehicle_racing_num_core/racing_num_writer.hpp"

#include <cstdlib>
#include <fstream>

namespace vehicle_racing_num_core
{

std::string DefaultOutputPath()
{
  const char *home = std::getenv("HOME");
  if (home == nullptr || home[0] == '\0')
  {
    return "autoStartGkj/command";
  }
  return std::string(home) + "/autoStartGkj/command";
}

bool WriteRacingNum(const std::string &output_file, int racing_num)
{
  std::ofstream outfile(output_file, std::ofstream::trunc);
  if (!outfile.is_open())
  {
    return false;
  }
  outfile << racing_num;
  return true;
}

} // namespace vehicle_racing_num_core
