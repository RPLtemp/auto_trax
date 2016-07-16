//
// Created by frank on 10.07.16.
//

#include "auto_trax_path_planning/rrt_unit_test.h"
namespace test {
  void InitializeParameters(const ros::NodeHandle &nh, OCGridParameterBag *parameter) {
    nh.param("subscribed_rostopic_scan_summary", parameter->subscribed_rostopic_scan_summary,
             kDefaultScanSummarySubTopic);
    nh.param("queue_size_subscriber_scan_summary", parameter->queue_size_subscriber_scan_summary,
             kDefaultScanSummarySubQueueSize);
    nh.param("published_rostopic_oc_grid", parameter->published_rostopic_oc_grid, kDefaultOCGridPubTopic);
    nh.param("queue_size_pub_oc_grid", parameter->queue_size_pub_oc_grid, kDefaultOCGridQueueSize);
    nh.param("grid_height", parameter->grid_height, kDefaultGridHeight);
    nh.param("grid_width", parameter->grid_width, kDefaultGridWidth);
    nh.param("grid_resolution", parameter->grid_resolution, kDefaultGridResolution);
    nh.param("origin_position_x", parameter->origin_position_x, kDefaultGridOriginPositionX);
    nh.param("origin_position_y", parameter->origin_position_y, kDefaultGridOriginPositionY);
    nh.param("origin_position_z", parameter->origin_position_z, kDefaultGridOriginPositionZ);
    nh.param("origin_quaternion_x", parameter->origin_quaternion_x, kDefaultGridOriginQuaternionX);
    nh.param("origin_quaternion_y", parameter->origin_quaternion_y, kDefaultGridOriginQuaternionY);
    nh.param("origin_quaternion_z", parameter->origin_quaternion_z, kDefaultGridOriginQuaternionZ);
    nh.param("origin_quaternion_w", parameter->origin_quaternion_w, kDefaultGridOriginQuaternionW);
    nh.param("frame_id", parameter->frame_id, kDefaultGridFrameID);
    nh.param("obstacle_padding", parameter->obstacle_padding, kDefaultObstaclePadding);
    nh.param("sensor_available_range", parameter->sensor_available_range, kDefaultSensorAvailableRange);
    nh.param("start_state_x", parameter->start_state_x, kDefaultStartX);
    nh.param("start_state_y", parameter->start_state_y, kDefaultStartY);
    nh.param("goal_state_x", parameter->goal_state_x, kDefaultGoalX);
    nh.param("goal_state_y", parameter->goal_state_y, kDefaultGoalY);
    nh.param("subscribed_rostopic_oc_grid", parameter->subscribed_rostopic_oc_grid, kDefaultOCGridSubTopic);
    nh.param("published_rostopic_path", parameter->published_rostopic_path, kDefaultPathPubTopic);
    nh.param("queue_size_pub_path", parameter->queue_size_pub_path, kDefaultPathPubQueueSize);
    nh.param("queue_size_subscriber_oc_grid", parameter->queue_size_subscriber_oc_grid, kDefaultOCGridQueueSize);
    nh.param("rrt_goal_max_dist", parameter->rrt_goal_max_dist, kDefaultGoalMaxDist);
    nh.param("rrt_max_step_size", parameter->rrt_max_step_size, kDefaultRRTMaxStepSize);
    nh.param("rrt_step_size", parameter->rrt_step_size, kDefaultRRTStepSize);
    nh.param("rrt_weighpoints_size", parameter->rrt_weighpoints_size, kDefaultRRTWeightPointsSize);
    nh.param("rrt_max_iteration", parameter->rrt_max_iteration, kDefaultRRTMaxIteration);
    nh.param("start_vel_x", parameter->start_vel_x, kDefaultStartVelX);
    nh.param("start_vel_y", parameter->start_vel_y, kDefaultStartVelY);
    nh.param("goal_vel_x", parameter->goal_vel_x, kDefaultGoalVelX);
    nh.param("goal_vel_y", parameter->goal_vel_y, kDefaultGoalVelY);

  }

}
int main(int argc, char** argv)
{
  ros::init (argc, argv, "auto_trax_rrt_path_node");
  ros::NodeHandle nh;

  OCGridParameterBag parameter;
  test::InitializeParameters(nh, &parameter);

  test::RRTUnitTest rrtUnitTest(nh, parameter);

  ros::spin();
}
