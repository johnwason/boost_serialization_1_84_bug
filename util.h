


#include <array>
#include <vector>
#include <set>
#include <string>
#include <sstream>
#include <stdexcept>
#include <random>
// #include <Eigen/Core>
// #include <Eigen/Geometry>

#include <boost/filesystem.hpp>




#pragma once

namespace tesseract_common
{
static bool almostEqualRelativeAndAbs(double a, double b, double max_diff = 1e-6, double max_rel_diff = std::numeric_limits<double>::epsilon())
{
  double diff = std::fabs(a - b);
  if (diff <= max_diff)
    return true;

  a = std::fabs(a);
  b = std::fabs(b);
  double largest = (b > a) ? b : a;

  return (diff <= largest * max_rel_diff);
}

template <typename KeyValueContainerType, typename ValueType>
bool isIdenticalMap(
    const KeyValueContainerType& map_1,
    const KeyValueContainerType& map_2,
    const std::function<bool(const ValueType&, const ValueType&)>& value_eq =
        [](const ValueType& v1, const ValueType& v2) { return v1 == v2; })
{
  if (map_1.size() != map_2.size())
    return false;

  for (const auto& entry : map_1)
  {
    // Check if the key exists
    const auto cp = map_2.find(entry.first);
    if (cp == map_2.end())
      return false;
    // Check if the value is the same
    if (!value_eq(cp->second, entry.second))
      return false;
  }
  return true;
}

static std::string getTempPath() { return boost::filesystem::temp_directory_path().string() + std::string(1, boost::filesystem::path::preferred_separator); }
}