#include <gtest/gtest.h>
#include "Path.h"

TEST(CoveragePathTest, AddViewpointsCorrectly)
{
  Pose pose1{Pose::Position{50.381589, -4.133504, 200}, Pose::Orientation{1, 1, 1, 1}};
  Path coveragePath{};
  CoverageViewpoint viewpoint1{pose1, 0};
  coveragePath.addCoverageViewpoint(viewpoint1);

  auto addedViewpointPose = coveragePath.getPath()[0].getPose();

  EXPECT_EQ(addedViewpointPose.position.latitude, pose1.position.latitude) << "Latitude of first added Viewpoint not expected value";
  EXPECT_EQ(addedViewpointPose.position.longitude, pose1.position.longitude) << "Longitude of first added Viewpoint not expected value";;
  EXPECT_EQ(addedViewpointPose.position.altitude, pose1.position.altitude) << "Altitude of first added Viewpoint not expected value";
  EXPECT_EQ(addedViewpointPose.orientation.x, pose1.orientation.x) << "x quaternion value of first added Viewpoint not expected value";
  EXPECT_EQ(addedViewpointPose.orientation.y, pose1.orientation.y) << "y quaternion value of first added Viewpoint not expected value";
  EXPECT_EQ(addedViewpointPose.orientation.z, pose1.orientation.z) << "z quaternion value of first added Viewpoint not expected value";
  EXPECT_EQ(addedViewpointPose.orientation.w, pose1.orientation.w) << "w quaternion value of first added Viewpoint not expected value";

  Pose pose2{Pose::Position{-29.915841, 27.687391, 2000}, Pose::Orientation{0, 0, 0, 0}};
  CoverageViewpoint viewpoint2{pose2, 1};
  coveragePath.addCoverageViewpoint(viewpoint2);

  auto oldViewpointPose = coveragePath.getPath()[0].getPose();

  EXPECT_EQ(addedViewpointPose.position.latitude, pose1.position.latitude) << "Latitude of first added Viewpoint not expected value after second viewpoint added";
  EXPECT_EQ(addedViewpointPose.position.longitude, pose1.position.longitude) << "Longitude of first added Viewpoint not expected value after second viewpoint added";;
  EXPECT_EQ(addedViewpointPose.position.altitude, pose1.position.altitude) << "Altitude of first added Viewpoint not expected value after second viewpoint added";
  EXPECT_EQ(addedViewpointPose.orientation.x, pose1.orientation.x) << "x quaternion value of first added Viewpoint not expected value after second viewpoint added";
  EXPECT_EQ(addedViewpointPose.orientation.y, pose1.orientation.y) << "y quaternion value of first added Viewpoint not expected value after second viewpoint added";
  EXPECT_EQ(addedViewpointPose.orientation.z, pose1.orientation.z) << "z quaternion value of first added Viewpoint not expected value after second viewpoint added";
  EXPECT_EQ(addedViewpointPose.orientation.w, pose1.orientation.w) << "w quaternion value of first added Viewpoint not expected value after second viewpoint added";

  auto newViewpointPose = coveragePath.getPath()[1].getPose();

  EXPECT_EQ(newViewpointPose.position.latitude, pose2.position.latitude) << "Latitude of second added Viewpoint not expected value after second viewpoint added";
  EXPECT_EQ(newViewpointPose.position.longitude, pose2.position.longitude) << "Longitude of second added Viewpoint not expected value after second viewpoint added";;
  EXPECT_EQ(newViewpointPose.position.altitude, pose2.position.altitude) << "Altitude of second added Viewpoint not expected value after second viewpoint added";
  EXPECT_EQ(newViewpointPose.orientation.x, pose2.orientation.x) << "x quaternion value of second added Viewpoint not expected value after second viewpoint added";
  EXPECT_EQ(newViewpointPose.orientation.y, pose2.orientation.y) << "y quaternion value of second added Viewpoint not expected value after second viewpoint added";
  EXPECT_EQ(newViewpointPose.orientation.z, pose2.orientation.z) << "z quaternion value of second added Viewpoint not expected value after second viewpoint added";
  EXPECT_EQ(newViewpointPose.orientation.w, pose2.orientation.w) << "w quaternion value of second added Viewpoint not expected value after second viewpoint added";
}
