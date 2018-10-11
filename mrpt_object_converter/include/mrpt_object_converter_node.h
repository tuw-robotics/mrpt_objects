#ifndef OBJECT_CONVERTER_NODE_H
#define OBJECT_CONVERTER_NODE_H

#include <ros/ros.h>
#include <tuw_object_msgs/ObjectDetection.h>
#include <tf/transform_listener.h>
#include <mrpt/poses/CPose3D.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/core.hpp>
#include <mrpt/obs/CObservationObject.h>
#include <mrpt_msgs/ObservationObject.h>

/**
 * This class listens to objectdetections and inserts them into mrpt maps.
 * Furthermore, it also listens to nav_msgs::OccupancyGridMap messages and converts those maps to mrpt maps
 * @brief The ObjectListener class
 */

class ObjectConverterNode
{
public:
  struct ParametersNode
  {
      ParametersNode();
      ros::NodeHandle nh;
      bool debug;
      bool contour_filtering;
      bool robot_perspective;
      float contour_offset;
      std::string tf_prefix;
      std::string world_frame_name;
      std::string source_frame_name;
      std::string publisher_topic_name;
      std::string subscriber_topic_name;
  };

  ObjectConverterNode(ros::NodeHandle &n);
  ~ObjectConverterNode();

  ParametersNode *param();
  void init();
  void callbackObjectDetections(const tuw_object_msgs::ObjectDetection &_msg);
  void callbackScan(const sensor_msgs::LaserScan &_laser_msg);
  bool getTF(std::string source_frame, mrpt::poses::CPose3D &des);

private:
  int x_range;
	int y_range;
	float scale_factor = 100.0;
	int add_factor = 500.0;
  ros::NodeHandle n_;
  ros::Subscriber sub_object_detections_;
  ros::Subscriber sub_scan_;
  ros::Publisher pub_bearings_;
  std::vector<cv::Point2f> contour_ = {};
  ParametersNode* params_;
  tf::TransformListener listenerTF_;
  mrpt::poses::CPose3D map_pose_;
  std::map<std::string, mrpt::poses::CPose3D> static_tf_;
  bool already_printed_=false;

  bool convert(const tuw_object_msgs::ObjectWithCovariance &_obj, mrpt::obs::CObservationObject::TMeasurement &_meas);
  
};

#endif // OBJECT_LISTENER_NODE_H
