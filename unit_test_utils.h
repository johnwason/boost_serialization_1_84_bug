#pragma once

#include "ser.h"

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
        tesseract_common::Serialization::toArchiveFileBinary<std::shared_ptr<SerializableTypeBase>>(object, file_path));

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