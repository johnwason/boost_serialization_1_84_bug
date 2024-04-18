#include <iostream>

#include <gtest/gtest.h>
// #include <boost/serialization/shared_ptr.hpp>
#include <tesseract_common/serialization.h>
#include <tesseract_common/utils.h>
#include <tesseract_common/unit_test_utils.h>

#include <tesseract_environment/command.h>
#include <tesseract_environment/commands/change_joint_position_limits_command.h>

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



#include <fstream>

#include <math.h>

#include "cmd.h"

// serialization.h


#if 1
// change_joint_position_limits_command.h



// change_joint_position_limits_command.cpp
#endif

TEST(serialization_test,xml)
{
    using namespace tesseract_common;
    auto object = std::make_shared<tesseract_environment::ChangeJointPositionLimitsCommand>("joint6", -M_PI, M_PI);
    testSerialization<tesseract_environment::ChangeJointPositionLimitsCommand>(*object, "ChangeJointPositionLimitsCommand");
    testSerializationDerivedClass<tesseract_environment::Command, tesseract_environment::ChangeJointPositionLimitsCommand>(object, "ChangeJointPositionLimitsCommand");

}


int main(int argc, char* argv[]) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}