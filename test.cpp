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

#include "util.h"
#include "unit_test_utils.h"


#include "cmd.h"
#include "cmd2.h"


TEST(serialization_test,xml)
{
    auto object = std::make_shared<tesseract_environment1::ChangeJointPositionLimitsCommand>("joint6", -M_PI, M_PI);
    testSerialization<tesseract_environment1::ChangeJointPositionLimitsCommand>(*object, "ChangeJointPositionLimitsCommand");
    testSerializationDerivedClass<tesseract_environment1::Command, tesseract_environment1::ChangeJointPositionLimitsCommand>(object, "ChangeJointPositionLimitsCommand");

}


int main(int argc, char* argv[]) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}