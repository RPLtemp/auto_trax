/*
 * occupancy_grid_plugin.cpp
 *
 *  Created on: Jul 31, 2016
 *      Author: pvechersky
 */

#include <boost/bind.hpp>
#include <cv_bridge/cv_bridge.h>
#include <gazebo/gazebo.hh>
#include <gazebo/math/Pose.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>

namespace gazebo
{
  class OccupancyGrid : public WorldPlugin
  {
    public:
      void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {
        world_ = _parent;

        if (!GetParameters(_sdf)) {
          ROS_ERROR("Please specify all parameters");
          return;
        }

        int img_rows = map_length_ / resolution_;
        int img_cols = map_width_ / resolution_;

        math::Vector3 start_point(origin_x_ - 0.5 * map_length_ + 0.5 * resolution_,
                                  origin_y_ - 0.5 * map_width_ + 0.5 * resolution_,
                                  origin_z_);

        math::Vector3 end_point(origin_x_ + 0.5 * map_length_ - 0.5 * resolution_,
                                origin_y_ + 0.5 * map_width_ - 0.5 * resolution_,
                                origin_z_);

        gazebo::physics::PhysicsEnginePtr engine = world_->GetPhysicsEngine();
        engine->InitForThread();
        gazebo::physics::RayShapePtr ray =
                boost::dynamic_pointer_cast<physics::RayShape>(
                    engine->CreateShape("ray", gazebo::physics::CollisionPtr()));

        cv::Mat grid_img(img_rows, img_cols, CV_8UC1, cv::Scalar(255));

        for (double x = start_point.x; x <= end_point.x; x += resolution_) {
          for (double y = start_point.y; y <= end_point.y; y += resolution_) {
            math::Vector3 pt(x, y, origin_z_);
            bool is_occupied = CheckIfOccupied(pt, ray, resolution_);

            if (is_occupied) {
              double dist_x = end_point.x - x;
              double dist_y = end_point.y - y;

              int ind_x = dist_x / resolution_;
              int ind_y = dist_y / resolution_;

              grid_img.at<uchar>(cv::Point(ind_y, ind_x)) = 0;
            }
          }
        }

        cv::imwrite(img_path_, grid_img);
      }

      bool GetParameters(sdf::ElementPtr sdf) {
        if (sdf->HasElement("MapLength"))
          map_length_ = sdf->GetElement("MapLength")->Get<double>();
        else
          return false;

        if (sdf->HasElement("MapWidth"))
          map_width_ = sdf->GetElement("MapWidth")->Get<double>();
        else
          return false;

        if (sdf->HasElement("Resolution"))
          resolution_ = sdf->GetElement("Resolution")->Get<double>();
        else
          return false;

        if (sdf->HasElement("OriginX"))
          origin_x_ = sdf->GetElement("OriginX")->Get<double>();
        else
          return false;

        if (sdf->HasElement("OriginY"))
          origin_y_ = sdf->GetElement("OriginY")->Get<double>();
        else
          return false;

        if (sdf->HasElement("OriginZ"))
          origin_z_ = sdf->GetElement("OriginZ")->Get<double>();
        else
          return false;

        if (sdf->HasElement("ImagePath"))
          img_path_ = sdf->GetElement("ImagePath")->Get<std::string>();
        else
          return false;

        return true;
      }

      bool CheckIfOccupied(const math::Vector3& central_point,
                           gazebo::physics::RayShapePtr ray,
                           const double resolution) {
        math::Vector3 start_point = central_point;
        math::Vector3 end_point = central_point;

        double dist;
        std::string entity_name;

        start_point.x += resolution / 2;
        end_point.x -= resolution / 2;
        ray->SetPoints(start_point, end_point);
        ray->GetIntersection(dist, entity_name);

        if (dist <= resolution) return true;

        start_point = central_point;
        end_point = central_point;
        start_point.y += resolution / 2;
        end_point.y -= resolution / 2;
        ray->SetPoints(start_point, end_point);
        ray->GetIntersection(dist, entity_name);

        if (dist <= resolution) return true;

        return false;
      }

    private:
      physics::WorldPtr world_;

      double map_length_;
      double map_width_;
      double resolution_;

      double origin_x_;
      double origin_y_;
      double origin_z_;

      std::string img_path_;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(OccupancyGrid);
}
