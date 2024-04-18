#include <iostream>


#include <memory>
#include <vector>
#include <boost/serialization/export.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/tracking.hpp>
#include <boost/serialization/tracking_enum.hpp>
#include <boost/serialization/array.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/unordered_map.hpp>
#include <boost/serialization/shared_ptr.hpp>

#include <boost/filesystem.hpp>

#include <gtest/gtest.h>

#include <fstream>
#define _USE_MATH_DEFINES
#include <math.h>

// serialization.h

// Use this macro for serialization defined using the invasive method inside the class
#define TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(Type)                                                                 \
  template void Type::serialize(boost::archive::xml_oarchive& ar, const unsigned int version);                         \
  template void Type::serialize(boost::archive::xml_iarchive& ar, const unsigned int version);                         \
  template void Type::serialize(boost::archive::binary_oarchive& ar, const unsigned int version);                      \
  template void Type::serialize(boost::archive::binary_iarchive& ar, const unsigned int version);


namespace tesseract_common
{

struct Serialization
{
  template <typename SerializableType>
  static bool toArchiveFileXML(const SerializableType& archive_type,
                               const std::string& file_path,
                               const std::string& name = "")
  {
    boost::filesystem::path fp(file_path);
    if (!fp.has_extension())
      fp.append(".").append("xml");

    std::ofstream os(fp.string());
    {  // Must be scoped because all data is not written until the oost::archive::xml_oarchive goes out of scope
      boost::archive::xml_oarchive oa(os);
      // Boost uses the same function for serialization and deserialization so it requires a non-const reference
      // Because we are only serializing here it is safe to cast away const
      if (name.empty())
        oa << boost::serialization::make_nvp<SerializableType>("archive_type",
                                                               const_cast<SerializableType&>(archive_type));  // NOLINT
      else
        oa << boost::serialization::make_nvp<SerializableType>(name.c_str(),
                                                               const_cast<SerializableType&>(archive_type));  // NOLINT
    }

    return true;
  }

  template <typename SerializableType>
  static SerializableType fromArchiveFileXML(const std::string& file_path)
  {
    SerializableType archive_type;

    {  // Must be scoped because all data is not written until the oost::archive::xml_oarchive goes out of scope
      std::ifstream ifs(file_path);
      assert(ifs.good());
      boost::archive::xml_iarchive ia(ifs);
      ia >> BOOST_SERIALIZATION_NVP(archive_type);
    }

    return archive_type;
  }

 
  template <typename SerializableType>
  static bool toArchiveFileBinary(const SerializableType& archive_type,
                                  const std::string& file_path,
                                  const std::string& name = "")
  {
    boost::filesystem::path fp(file_path);
    if (!fp.has_extension())
      fp.append(".").append("bin");

    std::ofstream os(fp.string(), std::ios_base::binary);
    {  // Must be scoped because all data is not written until the oost::archive::xml_oarchive goes out of scope
      boost::archive::binary_oarchive oa(os);
      // Boost uses the same function for serialization and deserialization so it requires a non-const reference
      // Because we are only serializing here it is safe to cast away const
      if (name.empty())
        oa << boost::serialization::make_nvp<SerializableType>("archive_type",
                                                               const_cast<SerializableType&>(archive_type));  // NOLINT
      else
        oa << boost::serialization::make_nvp<SerializableType>(name.c_str(),
                                                               const_cast<SerializableType&>(archive_type));  // NOLINT
    }

    return true;
  }

  template <typename SerializableType>
  static SerializableType fromArchiveFileBinary(const std::string& file_path)
  {
    SerializableType archive_type;

    {  // Must be scoped because all data is not written until the oost::archive::xml_oarchive goes out of scope
      std::ifstream ifs(file_path, std::ios_base::binary);
      assert(ifs.good());
      boost::archive::binary_iarchive ia(ifs);
      ia >> BOOST_SERIALIZATION_NVP(archive_type);
    }

    return archive_type;
  }
  template <typename SerializableType>
  static std::string toArchiveStringXML(const SerializableType& archive_type, const std::string& name = "")
  {
    std::stringstream ss;
    {  // Must be scoped because all data is not written until the oost::archive::xml_oarchive goes out of scope
      boost::archive::xml_oarchive oa(ss);

      // Boost uses the same function for serialization and deserialization so it requires a non-const reference
      // Because we are only serializing here it is safe to cast away const
      if (name.empty())
        oa << boost::serialization::make_nvp<SerializableType>("archive_type",
                                                               const_cast<SerializableType&>(archive_type));  // NOLINT
      else
        oa << boost::serialization::make_nvp<SerializableType>(name.c_str(),
                                                               const_cast<SerializableType&>(archive_type));  // NOLINT
    }

    return ss.str();
  }

  template <typename SerializableType>
  static SerializableType fromArchiveStringXML(const std::string& archive_xml)
  {
    SerializableType archive_type;

    {  // Must be scoped because all data is not written until the oost::archive::xml_oarchive goes out of scope
      std::stringstream ss(archive_xml);
      boost::archive::xml_iarchive ia(ss);
      ia >> BOOST_SERIALIZATION_NVP(archive_type);
    }

    return archive_type;
  }
};
}
// utils.cpp

namespace tesseract_common
{
bool almostEqualRelativeAndAbs(double a, double b, double max_diff = 1e-6, double max_rel_diff = std::numeric_limits<double>::epsilon())
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

std::string getTempPath() { return boost::filesystem::temp_directory_path().string() + std::string(1, boost::filesystem::path::preferred_separator); }
}
// command.h

namespace boost::serialization
{
class access;
}

namespace tesseract_environment
{
enum class CommandType
{
  UNINITIALIZED = -1,
  ADD_LINK = 0,
  MOVE_LINK = 1,
  MOVE_JOINT = 2,
  REMOVE_LINK = 3,
  REMOVE_JOINT = 4,
  CHANGE_LINK_ORIGIN = 5,
  CHANGE_JOINT_ORIGIN = 6,
  CHANGE_LINK_COLLISION_ENABLED = 7,
  CHANGE_LINK_VISIBILITY = 8,
  MODIFY_ALLOWED_COLLISIONS = 9,
  REMOVE_ALLOWED_COLLISION_LINK = 10,
  ADD_SCENE_GRAPH = 11,
  CHANGE_JOINT_POSITION_LIMITS = 12,
  CHANGE_JOINT_VELOCITY_LIMITS = 13,
  CHANGE_JOINT_ACCELERATION_LIMITS = 14,
  ADD_KINEMATICS_INFORMATION = 15,
  REPLACE_JOINT = 16,
  CHANGE_COLLISION_MARGINS = 17,
  ADD_CONTACT_MANAGERS_PLUGIN_INFO = 18,
  SET_ACTIVE_DISCRETE_CONTACT_MANAGER = 19,
  SET_ACTIVE_CONTINUOUS_CONTACT_MANAGER = 20,
  ADD_TRAJECTORY_LINK = 21
};

template <class Archive>
void save(Archive& ar, const CommandType& g, const unsigned int version);  // NOLINT

template <class Archive>
void load(Archive& ar, CommandType& g, const unsigned int version);  // NOLINT

template <class Archive>
void serialize(Archive& ar, CommandType& g, const unsigned int version);  // NOLINT

class Command
{
public:
  using Ptr = std::shared_ptr<Command>;
  using ConstPtr = std::shared_ptr<const Command>;

  Command(CommandType type = CommandType::UNINITIALIZED);
  virtual ~Command() = default;
  Command(const Command&) = default;
  Command& operator=(const Command&) = default;
  Command(Command&&) = default;
  Command& operator=(Command&&) = default;

  CommandType getType() const { return type_; }

  bool operator==(const Command& rhs) const;
  bool operator!=(const Command& rhs) const;

private:
  CommandType type_;

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

using Commands = std::vector<std::shared_ptr<const Command>>;
}  // namespace tesseract_environment

BOOST_CLASS_EXPORT_KEY2(tesseract_environment::Command, "Command")

// command.cpp

namespace tesseract_environment
{
Command::Command(CommandType type) : type_(type) {}

template <class Archive>
void save(Archive& ar, const CommandType& g, const unsigned int /*version*/)
{
  int value = static_cast<int>(g);
  ar &= BOOST_SERIALIZATION_NVP(value);
}

template <class Archive>
void load(Archive& ar, CommandType& g, const unsigned int /*version*/)
{
  int value = 0;
  ar &= BOOST_SERIALIZATION_NVP(value);
  g = static_cast<CommandType>(value);
}

template <class Archive>
void serialize(Archive& ar, CommandType& g, const unsigned int version)
{
  split_free(ar, g, version);
}

bool Command::operator==(const Command& rhs) const
{
  bool equal = true;
  equal &= type_ == rhs.type_;
  return equal;
}
bool Command::operator!=(const Command& rhs) const { return !operator==(rhs); }  // LCOV_EXCL_LINE

template <class Archive>
void Command::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(type_);
}
}  // namespace tesseract_environment

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_environment::Command)

// change_joint_position_limits_command.h

namespace tesseract_environment
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

BOOST_CLASS_EXPORT_KEY2(tesseract_environment::ChangeJointPositionLimitsCommand, "ChangeJointPositionLimitsCommand")

// change_joint_position_limits_command.cpp

namespace tesseract_environment
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
  equal &= Command::operator==(rhs);
  equal &= tesseract_common::isIdenticalMap<std::unordered_map<std::string, std::pair<double, double>>,
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

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_environment::ChangeJointPositionLimitsCommand)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_environment::ChangeJointPositionLimitsCommand)

// unit_test_utils.h

template <typename SerializableType>
void testSerialization(const SerializableType& object, const std::string& typename_string)
{
  {  // Archive program to XML file
    std::string file_path = tesseract_common::getTempPath() + typename_string + ".xml";
    EXPECT_TRUE(tesseract_common::Serialization::toArchiveFileXML<SerializableType>(object, file_path));

    SerializableType nobject{ tesseract_common::Serialization::fromArchiveFileXML<SerializableType>(file_path) };
    EXPECT_FALSE(object != nobject);  // Using != because it call == for code coverage
  }
    
  {  // Archive program to binary file
    std::string file_path = tesseract_common::getTempPath() + typename_string + "4.binary";
    EXPECT_TRUE(tesseract_common::Serialization::toArchiveFileBinary<SerializableType>(object, file_path));

    SerializableType nobject{ tesseract_common::Serialization::fromArchiveFileBinary<SerializableType>(file_path) };
    EXPECT_FALSE(object != nobject); 
     // Using != because it call == for code coverage
  }

  {  // Archive program to string
    std::string object_string =
        tesseract_common::Serialization::toArchiveStringXML<SerializableType>(object, typename_string);
    EXPECT_FALSE(object_string.empty());

    SerializableType nobject{ tesseract_common::Serialization::fromArchiveStringXML<SerializableType>(object_string) };
    EXPECT_FALSE(object != nobject);  // Using != because it call == for code coverage
  }
}

template <typename SerializableTypeBase, typename SerializableTypeDerived>
void testSerializationDerivedClass(const std::shared_ptr<SerializableTypeBase>& object,
                                   const std::string& typename_string)
{
  {  // Archive program to XML file
    std::string file_path = tesseract_common::getTempPath() + typename_string + ".xml";
    EXPECT_TRUE(
        tesseract_common::Serialization::toArchiveFileXML<std::shared_ptr<SerializableTypeBase>>(object, file_path));

    auto nobject =
        tesseract_common::Serialization::fromArchiveFileXML<std::shared_ptr<SerializableTypeBase>>(file_path);
    auto nobject_derived = std::dynamic_pointer_cast<SerializableTypeDerived>(nobject);

    auto object_derived = std::dynamic_pointer_cast<SerializableTypeDerived>(object);
    EXPECT_FALSE(*object_derived != *nobject_derived);  // Using != because it call == for code coverage
  }

  {  // Archive program to binary file
    std::string file_path = tesseract_common::getTempPath() + typename_string + "3.binary";
    EXPECT_TRUE(
        tesseract_common::Serialization::toArchiveFileBinary<std::shared_ptr<SerializableType>>(object, file_path));

    auto nobject =
        tesseract_common::Serialization::fromArchiveFileBinary<std::shared_ptr<SerializableTypeBase>>(file_path);
    auto nobject_derived = std::dynamic_pointer_cast<SerializableTypeDerived>(nobject);

    auto object_derived = std::dynamic_pointer_cast<SerializableTypeDerived>(object);
    EXPECT_FALSE(*object_derived != *nobject_derived);  // Using != because it call == for code coverage
  }

  {  // Archive program to string
    std::string object_string =
        tesseract_common::Serialization::toArchiveStringXML<std::shared_ptr<SerializableTypeBase>>(object,
                                                                                                   typename_string);
    EXPECT_FALSE(object_string.empty());

    auto nobject =
        tesseract_common::Serialization::fromArchiveStringXML<std::shared_ptr<SerializableTypeBase>>(object_string);
    auto nobject_derived = std::dynamic_pointer_cast<SerializableTypeDerived>(nobject);

    auto object_derived = std::dynamic_pointer_cast<SerializableTypeDerived>(object);
    EXPECT_FALSE(*object_derived != *nobject_derived);  // Using != because it call == for code coverage
  }
}

TEST(serialization_test,xml)
{
    auto object = std::make_shared<tesseract_environment::ChangeJointPositionLimitsCommand>("joint6", -M_PI, M_PI);
    testSerialization<tesseract_environment::ChangeJointPositionLimitsCommand>(*object, "ChangeJointPositionLimitsCommand");
    testSerializationDerivedClass<tesseract_environment::Command, tesseract_environment::ChangeJointPositionLimitsCommand>(object, "ChangeJointPositionLimitsCommand");

}


int main(int argc, char* argv[]) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}