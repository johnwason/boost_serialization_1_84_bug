

#include "cmd.h"

#pragma once

namespace tesseract_environment1
{

class ChangeJointPositionLimitsCommand : public Command
{
public:
  using Ptr = std::shared_ptr<ChangeJointPositionLimitsCommand>;
  using ConstPtr = std::shared_ptr<const ChangeJointPositionLimitsCommand>;

  ChangeJointPositionLimitsCommand();

  ChangeJointPositionLimitsCommand(std::string joint_name, double lower, double upper);

  ChangeJointPositionLimitsCommand(std::unordered_map<std::string, std::pair<double, double>> limits);

  const std::unordered_map<std::string, std::pair<double, double>>& getLimits() const;

  bool operator==(const ChangeJointPositionLimitsCommand& rhs) const;
  bool operator!=(const ChangeJointPositionLimitsCommand& rhs) const;

private:
  std::unordered_map<std::string, std::pair<double, double>> limits_;

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};
}  // namespace tesseract_environment

BOOST_CLASS_EXPORT_KEY2(tesseract_environment1::ChangeJointPositionLimitsCommand, "ChangeJointPositionLimitsCommand1")