#include <actionlib/client/simple_action_client.h>
#include <eigen_conversions/eigen_msg.h>
#include <ram_msgs/AdditiveManufacturingTrajectory.h>
#include <ram_path_planning/contours.hpp>
#include <ram_path_planning/donghong_ding.hpp>
#include <ram_path_planning/ContoursAction.h>
#include <ram_path_planning/DonghongDingAction.h>
#include <ros/package.h>
#include <ros/ros.h>

#include <gtest/gtest.h>

std::unique_ptr<ros::NodeHandle> nh;

// Action clients
using DonghongDingActionClient = actionlib::SimpleActionClient<ram_path_planning::DonghongDingAction>;
using ContoursActionClient = actionlib::SimpleActionClient<ram_path_planning::ContoursAction>;

std::unique_ptr<DonghongDingActionClient> donghong_ding_ac;
std::unique_ptr<ContoursActionClient> contours_ac;

bool use_gui;

TEST(TestSuite, testSrvExistence)
{
  ram_path_planning::DonghongDing<ram_path_planning::DonghongDingAction> donghong_ding;
  ram_path_planning::Contours<ram_path_planning::ContoursAction> contours;

  donghong_ding_ac.reset(new DonghongDingActionClient(donghong_ding.service_name_, true));
  bool donghong_ding_exists(donghong_ding_ac->waitForServer(ros::Duration(5)));

  contours_ac.reset(new ContoursActionClient(contours.service_name_, true));
  bool contours_exists(contours_ac->waitForServer(ros::Duration(5)));

  EXPECT_TRUE(donghong_ding_exists);
  EXPECT_TRUE(contours_exists);

}
/// Donghong Ding Algorithm
/// YAML
TEST(TestSuite, testPolygonWithinternalContoursDongHongDing)
{
  // Donghong Ding Algorithm
  ram_path_planning::DonghongDingGoal goal;

  goal.file = ros::package::getPath("ram_path_planning") + "/yaml/2_polygons_4_contours.yaml";
  goal.deposited_material_width = 1e-3;
  goal.contours_filtering_tolerance = 0.25e-3;
  goal.height_between_layers = 2e-3;
  goal.number_of_layers = 2;

  donghong_ding_ac->sendGoal(goal);
  bool success(donghong_ding_ac->waitForResult(ros::Duration(5)));

  bool state_succeeded(donghong_ding_ac->getState().state_ == actionlib::SimpleClientGoalState::SUCCEEDED);
  ram_path_planning::DonghongDingResultConstPtr result(donghong_ding_ac->getResult());

  if (!result->error_msg.empty())
    ROS_ERROR_STREAM("testPolygonWithinternalContoursDongHongDing: " << result->error_msg);

  EXPECT_TRUE(success);
  EXPECT_TRUE(state_succeeded);
  EXPECT_TRUE(result->error_msg.empty());
}

TEST(TestSuite, testIntersectedPolygonsDongHongDing)
{
  ram_path_planning::DonghongDingGoal goal;
  goal.file = ros::package::getPath("ram_path_planning") + "/yaml/polygon_intersection.yaml";
  goal.deposited_material_width = 1e-3;
  goal.contours_filtering_tolerance = 0.25e-3;
  goal.height_between_layers = 0.001;
  goal.number_of_layers = 2;

  donghong_ding_ac->sendGoal(goal);
  bool success(donghong_ding_ac->waitForResult(ros::Duration(5)));

  bool state_aborted(donghong_ding_ac->getState().state_ == actionlib::SimpleClientGoalState::ABORTED);
  ram_path_planning::DonghongDingResultConstPtr result(donghong_ding_ac->getResult());

  if (!result->error_msg.empty())
    ROS_ERROR_STREAM("testIntersectedPolygonsDongHongDing: " << result->error_msg);

  EXPECT_TRUE(success);
  EXPECT_TRUE(state_aborted);
  EXPECT_TRUE(!result->error_msg.empty()); // Should fail because polygons intersect
}

TEST(TestSuite, testConvexPolygonDongHongDing)
{
  ram_path_planning::DonghongDingGoal goal;
  goal.file = ros::package::getPath("ram_path_planning") + "/yaml/round_rectangle.yaml";
  goal.deposited_material_width = 1e-3;
  goal.contours_filtering_tolerance = 0.25e-3;
  goal.height_between_layers = 2e-3;
  goal.number_of_layers = 2;

  donghong_ding_ac->sendGoal(goal);
  bool success(donghong_ding_ac->waitForResult(ros::Duration(5)));

  bool state_succeeded(donghong_ding_ac->getState().state_ == actionlib::SimpleClientGoalState::SUCCEEDED);
  ram_path_planning::DonghongDingResultConstPtr result(donghong_ding_ac->getResult());

  if (!result->error_msg.empty())
    ROS_ERROR_STREAM("testConvexPolygonDongHongDing: " << result->error_msg);

  EXPECT_TRUE(success);
  EXPECT_TRUE(state_succeeded);
  EXPECT_TRUE(result->error_msg.empty());
}

TEST(TestSuite, testConcavePolygonBigDongHongDing)
{
  ram_path_planning::DonghongDingGoal goal;
  goal.file = ros::package::getPath("ram_path_planning") + "/yaml/concave_4_contours_big.yaml";
  goal.deposited_material_width = 0.1;
  goal.contours_filtering_tolerance = 0.5e-3;
  goal.height_between_layers = 0.1;
  goal.number_of_layers = 2;

  donghong_ding_ac->sendGoal(goal);
  bool success(donghong_ding_ac->waitForResult(ros::Duration(5)));

  bool state_succeeded(donghong_ding_ac->getState().state_ == actionlib::SimpleClientGoalState::SUCCEEDED);
  ram_path_planning::DonghongDingResultConstPtr result(donghong_ding_ac->getResult());

  if (!result->error_msg.empty())
    ROS_ERROR_STREAM("testConcavePolygonBigDongHongDing: " << result->error_msg);

  EXPECT_TRUE(success);
  EXPECT_TRUE(state_succeeded);
  EXPECT_TRUE(result->error_msg.empty());
}

TEST(TestSuite, testConcavePolygonSmallDongHongDing)
{
  ram_path_planning::DonghongDingGoal goal;
  goal.file = ros::package::getPath("ram_path_planning") + "/yaml/concave_4_contours_small.yaml";
  goal.deposited_material_width = 1e-3;
  goal.contours_filtering_tolerance = 0.25e-5;
  goal.height_between_layers = 0.1;
  goal.number_of_layers = 2;

  donghong_ding_ac->sendGoal(goal);
  bool success(donghong_ding_ac->waitForResult(ros::Duration(5)));

  bool state_succeeded(donghong_ding_ac->getState().state_ == actionlib::SimpleClientGoalState::SUCCEEDED);
  ram_path_planning::DonghongDingResultConstPtr result(donghong_ding_ac->getResult());

  if (!result->error_msg.empty())
    ROS_ERROR_STREAM("testConcavePolygonSmallDongHongDing: " << result->error_msg);

  EXPECT_TRUE(success);
  EXPECT_TRUE(state_succeeded);
  EXPECT_TRUE(result->error_msg.empty());
}

TEST(TestSuite, testStarDongHongDing)
{
  ram_path_planning::DonghongDingGoal goal;
  goal.file = ros::package::getPath("ram_path_planning") + "/yaml/star.yaml";
  goal.deposited_material_width = 1e-3;
  goal.contours_filtering_tolerance = 0.25e-3;
  goal.height_between_layers = 0.1;
  goal.number_of_layers = 2;

  donghong_ding_ac->sendGoal(goal);
  bool success(donghong_ding_ac->waitForResult(ros::Duration(5)));

  bool state_succeeded(donghong_ding_ac->getState().state_ == actionlib::SimpleClientGoalState::SUCCEEDED);
  ram_path_planning::DonghongDingResultConstPtr result(donghong_ding_ac->getResult());

  if (!result->error_msg.empty())
    ROS_ERROR_STREAM(result->error_msg);

  EXPECT_TRUE(success);
  EXPECT_TRUE(state_succeeded);
  EXPECT_TRUE(result->error_msg.empty());
}

/// MESHES

TEST(TestSuite, testSTLFileTwistedPyramidDongHongDing)
{
  ram_path_planning::DonghongDingGoal goal;
  goal.file = ros::package::getPath("ram_path_planning") + "/meshes/twisted_pyramid.stl";
  goal.deposited_material_width = 1e-3;
  goal.contours_filtering_tolerance = 1e-3;
  goal.height_between_layers = 5e-3;

  donghong_ding_ac->sendGoal(goal);
  bool success(donghong_ding_ac->waitForResult(ros::Duration(5)));

  bool state_succeeded(donghong_ding_ac->getState().state_ == actionlib::SimpleClientGoalState::SUCCEEDED);
  ram_path_planning::DonghongDingResultConstPtr result(donghong_ding_ac->getResult());

  if (!result->error_msg.empty())
    ROS_ERROR_STREAM("testSTLFileTwistedPyramidDongHongDing: " << result->error_msg);

  EXPECT_TRUE(success);
  EXPECT_TRUE(state_succeeded);
  EXPECT_TRUE(result->error_msg.empty());
}

TEST(TestSuite, testPLYFileTwistedPyramidDongHongDing)
{
  ram_path_planning::DonghongDingGoal goal;
  goal.file = ros::package::getPath("ram_path_planning") + "/meshes/twisted_pyramid.ply";
  goal.deposited_material_width = 1e-3;
  goal.contours_filtering_tolerance = 1e-3;
  goal.height_between_layers = 5e-3;

  donghong_ding_ac->sendGoal(goal);
  bool success(donghong_ding_ac->waitForResult(ros::Duration(5)));

  bool state_succeeded(donghong_ding_ac->getState().state_ == actionlib::SimpleClientGoalState::SUCCEEDED);
  ram_path_planning::DonghongDingResultConstPtr result(donghong_ding_ac->getResult());

  if (!result->error_msg.empty())
    ROS_ERROR_STREAM("testPLYFileTwistedPyramidDongHongDing: " << result->error_msg);

  EXPECT_TRUE(success);
  EXPECT_TRUE(state_succeeded);
  EXPECT_TRUE(result->error_msg.empty());
}

TEST(TestSuite, testOBJFileTwistedPyramidDongHongDing)
{
  ram_path_planning::DonghongDingGoal goal;
  goal.file = ros::package::getPath("ram_path_planning") + "/meshes/twisted_pyramid.obj";
  goal.deposited_material_width = 1e-3;
  goal.contours_filtering_tolerance = 1e-3;
  goal.height_between_layers = 5e-3;

  donghong_ding_ac->sendGoal(goal);
  bool success(donghong_ding_ac->waitForResult(ros::Duration(5)));

  bool state_succeeded(donghong_ding_ac->getState().state_ == actionlib::SimpleClientGoalState::SUCCEEDED);
  ram_path_planning::DonghongDingResultConstPtr result(donghong_ding_ac->getResult());

  if (!result->error_msg.empty())
    ROS_ERROR_STREAM("testOBJFileTwistedPyramidDongHongDing: " << result->error_msg);

  EXPECT_TRUE(success);
  EXPECT_TRUE(state_succeeded);
  EXPECT_TRUE(result->error_msg.empty());
}

TEST(TestSuite, testPLYFileTwoTwistedPyramidsDongHongDing)
{
  ram_path_planning::DonghongDingGoal goal;
  goal.file = ros::package::getPath("ram_path_planning") + "/meshes/two_twisted_pyramids.ply";
  goal.deposited_material_width = 1e-3;
  goal.contours_filtering_tolerance = 0.25e-3;
  goal.height_between_layers = 5e-3;

  donghong_ding_ac->sendGoal(goal);
  bool success(donghong_ding_ac->waitForResult(ros::Duration(5)));

  bool state_aborted(donghong_ding_ac->getState().state_ == actionlib::SimpleClientGoalState::ABORTED);
  ram_path_planning::DonghongDingResultConstPtr result(donghong_ding_ac->getResult());

  if (!result->error_msg.empty())
    ROS_ERROR_STREAM("testPLYFileTwoTwistedPyramidsDongHongDing: " << result->error_msg);

  EXPECT_TRUE(success);
  EXPECT_TRUE(state_aborted);
  EXPECT_TRUE(!result->error_msg.empty()); // Not implemented yet!
}

TEST(TestSuite, testConeTruncatedDongHongDing)
{
  ram_path_planning::DonghongDingGoal goal;
  goal.file = ros::package::getPath("ram_path_planning") + "/meshes/cone_truncated.ply";
  goal.deposited_material_width = 1e-3;
  goal.contours_filtering_tolerance = 0.6 * 1e-3;
  goal.height_between_layers = 5e-3;

  donghong_ding_ac->sendGoal(goal);
  bool success(donghong_ding_ac->waitForResult(ros::Duration(5)));

  bool state_succeeded(donghong_ding_ac->getState().state_ == actionlib::SimpleClientGoalState::SUCCEEDED);
  ram_path_planning::DonghongDingResultConstPtr result(donghong_ding_ac->getResult());

  if (!result->error_msg.empty())
    ROS_ERROR_STREAM("testConeTruncatedDongHongDing: " << result->error_msg);

  EXPECT_TRUE(success);
  EXPECT_TRUE(state_succeeded);
  EXPECT_TRUE(result->error_msg.empty());
}

TEST(TestSuite, testDomeDongHongDing)
{
  ram_path_planning::DonghongDingGoal goal;
  goal.file = ros::package::getPath("ram_path_planning") + "/meshes/dome.ply";
  goal.deposited_material_width = 1e-3;
  goal.contours_filtering_tolerance = 0.5 * 1e-3;
  goal.height_between_layers = 2e-3;

  donghong_ding_ac->sendGoal(goal);
  bool success(donghong_ding_ac->waitForResult(ros::Duration(5)));

  bool state_succeeded(donghong_ding_ac->getState().state_ == actionlib::SimpleClientGoalState::SUCCEEDED);
  ram_path_planning::DonghongDingResultConstPtr result(donghong_ding_ac->getResult());

  if (!result->error_msg.empty())
    ROS_ERROR_STREAM("testDomeDongHongDing: " << result->error_msg);

  EXPECT_TRUE(success);
  EXPECT_TRUE(state_succeeded);
  EXPECT_TRUE(result->error_msg.empty());
}

TEST(TestSuite, testInversedPyramidDongHongDing)
{
  ram_path_planning::DonghongDingGoal goal;
  goal.file = ros::package::getPath("ram_path_planning") + "/meshes/inversed_pyramid.ply";
  goal.deposited_material_width = 1e-3;
  goal.contours_filtering_tolerance = 0;
  goal.height_between_layers = 5e-3;

  donghong_ding_ac->sendGoal(goal);
  bool success(donghong_ding_ac->waitForResult(ros::Duration(5)));

  bool state_succeeded(donghong_ding_ac->getState().state_ == actionlib::SimpleClientGoalState::SUCCEEDED);
  ram_path_planning::DonghongDingResultConstPtr result(donghong_ding_ac->getResult());

  if (!result->error_msg.empty())
    ROS_ERROR_STREAM("testInversedPyramidDongHongDing: " << result->error_msg);

  EXPECT_TRUE(success);
  EXPECT_TRUE(state_succeeded);
  EXPECT_TRUE(result->error_msg.empty());
}

TEST(TestSuite, testSTLCubeNonZSlicingDongHongDing)
{
  ram_path_planning::DonghongDingGoal goal;
  goal.file = ros::package::getPath("ram_path_planning") + "/meshes/cube.stl";
  goal.deposited_material_width = 1e-3;
  goal.contours_filtering_tolerance = 1e-3;
  goal.height_between_layers = 5e-3;
  // Slicing vector is not Z:
  goal.slicing_direction.x = -0.15;
  goal.slicing_direction.y = -0.2;
  goal.slicing_direction.z = 1;

  donghong_ding_ac->sendGoal(goal);
  bool success(donghong_ding_ac->waitForResult(ros::Duration(5)));

  bool state_succeeded(donghong_ding_ac->getState().state_ == actionlib::SimpleClientGoalState::SUCCEEDED);
  ram_path_planning::DonghongDingResultConstPtr result(donghong_ding_ac->getResult());

  if (!result->error_msg.empty())
    ROS_ERROR_STREAM("testSTLCubeNonZSlicingDongHongDing: " << result->error_msg);

  EXPECT_TRUE(success);
  EXPECT_TRUE(state_succeeded);
  EXPECT_TRUE(result->error_msg.empty());
}

/// Contours Algorithm
/// YAML
TEST(TestSuite, testPolygonWithinternalContours)
{
  // Contours Algorithm
  ram_path_planning::ContoursGoal goal;
  goal.file = ros::package::getPath("ram_path_planning") + "/yaml/2_polygons_4_contours.yaml";
  goal.deposited_material_width = 1e-3;
  goal.height_between_layers = 2e-3;
  goal.number_of_layers = 2;

  contours_ac->sendGoal(goal);
  bool success(contours_ac->waitForResult(ros::Duration(5)));

  bool state_aborted(contours_ac->getState().state_ == actionlib::SimpleClientGoalState::ABORTED);
  ram_path_planning::ContoursResultConstPtr result(contours_ac->getResult());

  if (!result->error_msg.empty())
    ROS_ERROR_STREAM("testPolygonWithinternalContours: " << result->error_msg);

  EXPECT_TRUE(success);
  EXPECT_TRUE(state_aborted);
  EXPECT_TRUE(!result->error_msg.empty()); // Should fail because polygons contain internal contours
}

TEST(TestSuite, testIntersectedPolygonsContours)
{
  ram_path_planning::ContoursGoal goal;
  goal.file = ros::package::getPath("ram_path_planning") + "/yaml/polygon_intersection.yaml";
  goal.deposited_material_width = 1e-3;
  goal.height_between_layers = 0.001;
  goal.number_of_layers = 2;

  contours_ac->sendGoal(goal);
  bool success(contours_ac->waitForResult(ros::Duration(5)));

  bool state_aborted(contours_ac->getState().state_ == actionlib::SimpleClientGoalState::ABORTED);
  ram_path_planning::ContoursResultConstPtr result(contours_ac->getResult());

  if (!result->error_msg.empty())
    ROS_ERROR_STREAM("testIntersectedPolygonsContours: " << result->error_msg);

  EXPECT_TRUE(success);
  EXPECT_TRUE(state_aborted);
  EXPECT_TRUE(!result->error_msg.empty()); // Should fail because polygons intersect
}

TEST(TestSuite, testConvexPolygonContours)
{
  ram_path_planning::ContoursGoal goal;
  goal.file = ros::package::getPath("ram_path_planning") + "/yaml/round_rectangle.yaml";
  goal.deposited_material_width = 1e-3;
  goal.height_between_layers = 2e-3;
  goal.number_of_layers = 2;

  contours_ac->sendGoal(goal);
  bool success(contours_ac->waitForResult(ros::Duration(5)));

  bool state_succeeded(contours_ac->getState().state_ == actionlib::SimpleClientGoalState::SUCCEEDED);
  ram_path_planning::ContoursResultConstPtr result(contours_ac->getResult());

  if (!result->error_msg.empty())
    ROS_ERROR_STREAM("testConvexPolygonContours: " << result->error_msg);

  EXPECT_TRUE(success);
  EXPECT_TRUE(state_succeeded);
  EXPECT_TRUE(result->error_msg.empty());
}

TEST(TestSuite, testStarContours)
{
  ram_path_planning::ContoursGoal goal;
  goal.file = ros::package::getPath("ram_path_planning") + "/yaml/star.yaml";
  goal.deposited_material_width = 1e-3;
  goal.height_between_layers = 0.1;
  goal.number_of_layers = 2;

  contours_ac->sendGoal(goal);
  bool success(contours_ac->waitForResult(ros::Duration(5)));

  bool state_succeeded(contours_ac->getState().state_ == actionlib::SimpleClientGoalState::SUCCEEDED);
  ram_path_planning::ContoursResultConstPtr result(contours_ac->getResult());

  if (!result->error_msg.empty())
    ROS_ERROR_STREAM("testStarContours: " << result->error_msg);

  EXPECT_TRUE(success);
  EXPECT_TRUE(state_succeeded);
  EXPECT_TRUE(result->error_msg.empty());
}

/// MESHES

TEST(TestSuite, testSTLFileTwistedPyramidContours)
{
  ram_path_planning::ContoursGoal goal;
  goal.file = ros::package::getPath("ram_path_planning") + "/meshes/twisted_pyramid.stl";
  goal.deposited_material_width = 1e-3;
  goal.height_between_layers = 5e-3;

  contours_ac->sendGoal(goal);
  bool success(contours_ac->waitForResult(ros::Duration(5)));

  bool state_succeeded(contours_ac->getState().state_ == actionlib::SimpleClientGoalState::SUCCEEDED);
  ram_path_planning::ContoursResultConstPtr result(contours_ac->getResult());

  if (!result->error_msg.empty())
    ROS_ERROR_STREAM("testSTLFileTwistedPyramidContours: " << result->error_msg);

  EXPECT_TRUE(success);
  EXPECT_TRUE(state_succeeded);
  EXPECT_TRUE(result->error_msg.empty());
}

TEST(TestSuite, testPLYFileTwoTwistedPyramidsContours)
{
  ram_path_planning::ContoursGoal goal;
  goal.file = ros::package::getPath("ram_path_planning") + "/meshes/two_twisted_pyramids.ply";
  goal.deposited_material_width = 1e-3;
  goal.height_between_layers = 5e-3;

  contours_ac->sendGoal(goal);
  bool success(contours_ac->waitForResult(ros::Duration(5)));

  bool state_aborted(contours_ac->getState().state_ == actionlib::SimpleClientGoalState::ABORTED);
  ram_path_planning::ContoursResultConstPtr result(contours_ac->getResult());

  if (!result->error_msg.empty())
    ROS_ERROR_STREAM("testPLYFileTwoTwistedPyramidsContours: " << result->error_msg);

  EXPECT_TRUE(success);
  EXPECT_TRUE(state_aborted);
  EXPECT_TRUE(!result->error_msg.empty());  // Not implemented yet!

  if (use_gui)
    ROS_ERROR_STREAM("PLEASE CLOSE THE VTK WINDOW TO TERMINATE");
}

int main(int argc,
         char **argv)
{
  ros::init(argc, argv, "path_planning_test_client");
  nh.reset(new ros::NodeHandle);

  nh->param<bool>("use_gui", use_gui, false);
  if (use_gui)
    ROS_ERROR_STREAM("Press 'enter' in the terminal to go to the next step");

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
