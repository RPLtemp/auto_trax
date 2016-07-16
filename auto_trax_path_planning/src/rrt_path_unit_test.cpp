//
// Created by frank on 10.07.16.
//

#include "auto_trax_path_planning/rrt_unit_test.h"



  using namespace RRT;
  using namespace Eigen;
  using namespace std;

namespace test{
  RRTUnitTest::RRTUnitTest(const ros::NodeHandle &nh,
                           const OCGridParameterBag &parameters) :
          nh_(nh), rrt_param_(parameters) {
    ROS_DEBUG("RRT Path Processor started!");

    rrt_oc_grid_.header.frame_id = rrt_param_.frame_id;
    rrt_oc_grid_.info.width = rrt_param_.grid_width;
    rrt_oc_grid_.info.height = rrt_param_.grid_height;
    rrt_oc_grid_.info.resolution = rrt_param_.sensor_available_range / rrt_param_.grid_width;
    rrt_param_.grid_resolution = rrt_oc_grid_.info.resolution;
    rrt_oc_grid_.info.origin.position.x = rrt_param_.origin_position_x;
    rrt_oc_grid_.info.origin.position.y = rrt_param_.origin_position_y;
    rrt_oc_grid_.info.origin.position.z = rrt_param_.origin_position_z;
    rrt_oc_grid_.info.origin.orientation.x = rrt_param_.origin_quaternion_x;
    rrt_oc_grid_.info.origin.orientation.y = rrt_param_.origin_quaternion_y;
    rrt_oc_grid_.info.origin.orientation.z = rrt_param_.origin_quaternion_z;
    rrt_oc_grid_.info.origin.orientation.w = rrt_param_.origin_quaternion_w;
    rrt_oc_grid_.data.resize(rrt_oc_grid_.info.width * rrt_oc_grid_.info.height);
    rrt_weighpoints_size_ = rrt_param_.rrt_weighpoints_size;
    rrt_max_iteration_ = rrt_param_.rrt_max_iteration;

    sub_oc_grid_ = nh_.subscribe(rrt_param_.subscribed_rostopic_oc_grid,
                                 rrt_param_.queue_size_subscriber_oc_grid,
                                 &RRTUnitTest::CallbackOCGrid,
                                 this);
    pub_path_ = nh_.advertise<nav_msgs::Path>(rrt_param_.published_rostopic_path,
                                              rrt_param_.queue_size_pub_path);

    pub_oc_grid_debug = nh_.advertise<nav_msgs::OccupancyGrid>("rrt_oc_grid",
                                                               rrt_param_.queue_size_pub_oc_grid);

    pub_setpoint_ = nh_.advertise<std_msgs::Float64>("setpoint_angle", 1);

    pub_path_found_ = nh_.advertise<std_msgs::Bool>("path_found", 1);

    _stateSpace = make_shared<GridStateSpace>(rrt_param_.grid_height,
                                              rrt_param_.grid_width,
                                              rrt_param_.grid_height,
                                              rrt_param_.grid_width);
//
//    _stateSpace = make_shared<GridStateSpace>(800,
//                                              600,
//                                              40,
//                                              30);

    _biRRT = new BiRRT<Vector2f>(_stateSpace);

    //  setup birrt
    _biRRT->setStartState(Vector2f((rrt_param_.start_state_x - rrt_param_.origin_position_x) /
                                   rrt_param_.grid_resolution,
                                   (rrt_param_.start_state_y - rrt_param_.origin_position_y) /
                                   rrt_param_.grid_resolution));
    _biRRT->setGoalState(Vector2f((rrt_param_.goal_state_x - rrt_param_.origin_position_x)/
                                  rrt_param_.grid_resolution,
                                  (rrt_param_.goal_state_y - rrt_param_.origin_position_y) /
                                  rrt_param_.grid_resolution));

//    _biRRT->setStartState(Vector2f(50,100));
//    _biRRT->setGoalState(Vector2f(400, 300));

    _biRRT->setStepSize(rrt_param_.rrt_step_size);
    _biRRT->setMaxStepSize(rrt_param_.rrt_max_step_size);
    _biRRT->setGoalMaxDist(rrt_param_.rrt_goal_max_dist);

    _startVel = Vector2f(rrt_param_.start_vel_x, rrt_param_.start_vel_y);
    _goalVel = Vector2f(rrt_param_.goal_vel_x, rrt_param_.goal_vel_y);

    result_.header.frame_id = rrt_param_.frame_id;
  }

  RRTUnitTest::~RRTUnitTest() { }

  void RRTUnitTest::CallbackOCGrid(const nav_msgs::OccupancyGrid &oc_grid) {

    clearObstacles();
    rrt_oc_grid_.data.clear();
    rrt_oc_grid_.data.resize(oc_grid.info.height * oc_grid.info.width);
    // locate new obstacles
    for (int i = 0; i < oc_grid.info.width; i++)
    {
      for (int j = 0; j < oc_grid.info.height; j++)
      {
        if (oc_grid.data[j * oc_grid.info.width + i] == 100)
        {
          float x = i * oc_grid.info.resolution + oc_grid.info.origin.position.x;
          float y = j * oc_grid.info.resolution + oc_grid.info.origin.position.y;

//          setObstacleAt(x,y);
          _stateSpace->obstacleGrid().obstacleAt(Vector2i(i,j)) = true;

        }
      }
    }
//    for (int i = 0; i < 40; i++) {
//      for (int j = 0; j < 30; j++) {
////        if ((i * oc_grid.info.width + j) % 30 == 0) {
//        if ( i > 3 &&  j < 14) {
////          float y = j * oc_grid.info.resolution + oc_grid.info.origin.position.y;
////          float y = i * oc_grid.info.resolution + oc_grid.info.origin.position.y;
//
////          setObstacleAt(x, y);
//
//          _stateSpace->obstacleGrid().obstacleAt(Vector2i(i,j)) = true;
//
//        }
//      }
//    }

    for (int i = 0; i < rrt_param_.grid_width; i++) {
      for (int j = 0; j < rrt_param_.grid_height; j++) {
        if (_stateSpace->obstacleGrid().obstacleAt(i, j) == true) {
          // oc_grid debug
          rrt_oc_grid_.data.at(j * oc_grid.info.width + i) = 100;
        }
      }
    }
//    rrt_oc_grid_.info.width = 40;
//    rrt_oc_grid_.info.height = 30;
//    rrt_oc_grid_.info.resolution = .2;
//    rrt_oc_grid_.data.resize(1200);
//    for (int i = 0; i < 40; i++) {
//      for (int j = 0; j < 30; j++) {
//        if (_stateSpace->obstacleGrid().obstacleAt(i, j) == true) {
//          // oc_grid debug
//          rrt_oc_grid_.data.at(j * 40 + i) = 100;
//        }
//      }
//    }

    pub_oc_grid_debug.publish(rrt_oc_grid_);

    if (findSolution()) {
      printPath();

      path_found_.data = true;
      pub_path_found_.publish(path_found_);
      setpoint_.data = getFirstSetpoint();
      pub_setpoint_.publish(setpoint_);

    } else {
      result_.poses.clear();
      pub_path_.publish(result_);
      path_found_.data = false;
      pub_path_found_.publish(path_found_);
    }

  }


  void RRTUnitTest::reset() {

    //  store waypoint cache
    vector <Vector2f> waypoints;
    if (_biRRT->startSolutionNode() && _biRRT->goalSolutionNode()) {
      waypoints = _previousSolution;
      if (waypoints.size() > 0) {
        //  don't keep the start or end states
        waypoints.erase(waypoints.begin());
        waypoints.erase(waypoints.end() - 1);

        //  down-sample
        RRT::DownSampleVector<Vector2f>(waypoints, rrt_weighpoints_size_);
      }
    } else {
      _previousSolution.clear();
    }

    _biRRT->reset();

    _biRRT->setWaypoints(waypoints);

  }

  void RRTUnitTest::clearObstacles() {
    _stateSpace->obstacleGrid().clear();
  }

  void RRTUnitTest::step(int numTimes) {
    for (int i = 0; i < numTimes; i++) {
      _biRRT->grow();
    }

    //  store solution
    _previousSolution.clear();
    if (_biRRT->startSolutionNode() != nullptr) {
      _biRRT->getPath(_previousSolution);
    RRT::SmoothPath<Vector2f>(_previousSolution, *_stateSpace);

    }
  }

  bool RRTUnitTest::findSolution() {
    reset();
    int counter = 0;
    while (_biRRT->startSolutionNode() == nullptr && counter < rrt_max_iteration_) {
      step(1);
      counter++;
      if (_biRRT->startSolutionNode() != nullptr) {
        cout << "Path found " << endl;
        return true;
      }
    }

    cout << "Path not found in " << counter << " iterations." << endl;
    return false;
  }

  void RRTUnitTest::printPath() {
    if (_previousSolution.size() > 0) {

      std::cout << "Path covers ... " << std::endl;
      std::cout << " x " << '\t' << " y " << std::endl;

      result_.poses.clear();
      result_.poses.resize(_previousSolution.size());

      for (int i = 0; i < _previousSolution.size(); i++) {
        std::cout << _previousSolution[i].x() <<
                '\t'
        << _previousSolution[i].y()  << std::endl;
        result_.poses[i].pose.position.x = _previousSolution[i].x() * rrt_param_.grid_resolution +
                                           rrt_param_.origin_position_x;
        result_.poses[i].pose.position.y = _previousSolution[i].y() * rrt_param_.grid_resolution +
                                           rrt_param_.origin_position_x;
      }

      pub_path_.publish(result_);
    }

  }

  void RRTUnitTest::setObstacleAt(float x, float y) {
    Vector2i gridLoc = Vector2i(round((x - rrt_param_.origin_position_x) /
                                      rrt_param_.grid_resolution),
                                round((y - rrt_param_.origin_position_y) /
                                      rrt_param_.grid_resolution));

    _stateSpace->obstacleGrid().obstacleAt(gridLoc) = true;

  }

  void RRTUnitTest::debugPrint() {
  }

  double RRTUnitTest::getFirstSetpoint() {
    Eigen::Vector2f path_point = _previousSolution.at(1);
    float x = path_point.x() * rrt_param_.grid_resolution + rrt_param_.origin_position_x;
    float y = path_point.y() * rrt_param_.grid_resolution + rrt_param_.origin_position_y;
    return atan2(y, x);
  }

}