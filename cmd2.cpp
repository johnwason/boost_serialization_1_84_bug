#include <iostream>

#include <gtest/gtest.h>
#include <boost/serialization/shared_ptr.hpp>

#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>
#if (BOOST_VERSION >= 107400) && (BOOST_VERSION < 107500)
#include <boost/serialization/library_version_type.hpp>
#endif
#include <boost/serialization/map.hpp>
#include <boost/serialization/unordered_map.hpp>
#include <string>

#include "cmd2.h"

#include "util.h"

namespace tesseract_environment1
{

ChangeJointPositionLimitsCommand::ChangeJointPositionLimitsCommand()
  : Command(CommandType::CHANGE_JOINT_POSITION_LIMITS){};

ChangeJointPositionLimitsCommand::ChangeJointPositionLimitsCommand(std::string joint_name, double lower, double upper)
  : Command(CommandType::CHANGE_JOINT_POSITION_LIMITS)
  , limits_({ std::make_pair(std::move(joint_name), std::make_pair(lower, upper)) })
{
  assert(upper > lower);
}

ChangeJointPositionLimitsCommand::ChangeJointPositionLimitsCommand(
    std::unordered_map<std::string, std::pair<double, double>> limits)
  : Command(CommandType::CHANGE_JOINT_POSITION_LIMITS), limits_(std::move(limits))
{
  assert(std::all_of(limits_.begin(), limits_.end(), [](const auto& p) { return p.second.second > p.second.first; }));
}

const std::unordered_map<std::string, std::pair<double, double>>& ChangeJointPositionLimitsCommand::getLimits() const
{
  return limits_;
}

bool ChangeJointPositionLimitsCommand::operator==(const ChangeJointPositionLimitsCommand& rhs) const
{
  auto fn = [](const std::pair<double, double>& p1, const std::pair<double, double>& p2) {
    return tesseract_common::almostEqualRelativeAndAbs(p1.first, p2.first) &&
           tesseract_common::almostEqualRelativeAndAbs(p1.second, p2.second);
  };
  bool equal = true;
  equal & Command::operator==(rhs);
  equal & tesseract_common::isIdenticalMap<std::unordered_map<std::string, std::pair<double, double>>,
                                            std::pair<double, double>>(limits_, rhs.limits_, fn);
  return equal;
}
bool ChangeJointPositionLimitsCommand::operator!=(const ChangeJointPositionLimitsCommand& rhs) const
{
  return !operator==(rhs);
}

template <class Archive>
void ChangeJointPositionLimitsCommand::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Command);
  ar& BOOST_SERIALIZATION_NVP(limits_);
}
}  // namespace tesseract_environment

#include "ser.h"
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_environment1::ChangeJointPositionLimitsCommand)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_environment1::ChangeJointPositionLimitsCommand)
