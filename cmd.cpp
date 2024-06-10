#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/split_free.hpp>

#include "cmd.h"

namespace tesseract_environment1
{
Command::Command(CommandType type) : type_(type) {}

template <class Archive>
void save(Archive& ar, const CommandType& g, const unsigned int /*version*/)
{
  int value = static_cast<int>(g);
  ar & BOOST_SERIALIZATION_NVP(value);
}

template <class Archive>
void load(Archive& ar, CommandType& g, const unsigned int /*version*/)
{
  int value = 0;
  ar & BOOST_SERIALIZATION_NVP(value);
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
  equal & type_ == rhs.type_;
  return equal;
}
bool Command::operator!=(const Command& rhs) const { return !operator==(rhs); }  // LCOV_EXCL_LINE

template <class Archive>
void Command::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(type_);
}
}  // namespace tesseract_environment

#include "ser.h"
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_environment1::Command)
