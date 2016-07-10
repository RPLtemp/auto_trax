//
// Created by frank on 10.07.16.
//

#include "auto_trax_path_planning/occupancy_grid_processor.h"

void InitializeParameters(const ros::NodeHandle nh, OCGridParameterBag* parameter){
  nh.param("subscribed_rostopic_scan_summary", parameter->subscribed_rostopic_scan_summary, kDefaultScanSummarySubTopic);
  nh.param("queue_size_subscriber_scan_summary", parameter->queue_size_subscriber_scan_summary, kDefaultScanSummarySubQueueSize);
  nh.param("published_rostopic_oc_grid", parameter->published_rostopic_oc_grid, kDefaultOCGridPubTopic);
  nh.param("queue_size_pub_oc_grid", parameter->queue_size_pub_oc_grid, kDefaultOCGridQueueSize);

}

int main(int argc, char** argv)
{
  ros::init (argc, argv, "auto_trax_oc_grid_node");
  ros::NodeHandle nh("~");

  OCGridParameterBag parameter;
  InitializeParameters(nh, &parameter);

  OCGridProcessor(nh, parameter);

  ros::spin();
}
