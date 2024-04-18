#pragma once

#include <fstream>
#include <sstream>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/tracking.hpp>
#include <boost/serialization/tracking_enum.hpp>

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