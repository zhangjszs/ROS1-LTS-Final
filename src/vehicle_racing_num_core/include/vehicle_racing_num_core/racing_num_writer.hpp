#ifndef VEHICLE_RACING_NUM_CORE_RACING_NUM_WRITER_HPP_
#define VEHICLE_RACING_NUM_CORE_RACING_NUM_WRITER_HPP_

#include <string>

namespace vehicle_racing_num_core
{

std::string DefaultOutputPath();

bool WriteRacingNum(const std::string &output_file, int racing_num);

} // namespace vehicle_racing_num_core

#endif // VEHICLE_RACING_NUM_CORE_RACING_NUM_WRITER_HPP_
