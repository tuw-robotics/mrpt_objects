#ifndef OBJECT_CONVERTER_NODE_H
#define OBJECT_CONVERTER_NODE_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tuw_object_msgs/ObjectDetection.h>
#include <boost/filesystem.hpp>
#include <mrpt/maps/CMultiMetricMap.h>
#include <fstream>
#include <string>
#include <memory>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <tf/transform_listener.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/containers/deepcopy_poly_ptr.h>
#include <mrpt_msgs/ObservationRangeBearing.h>

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
        std::string tf_prefix;
        std::string base_frame_id;
        std::string publisher_topic_name;
        std::string subscriber_topic_name;
    };


   ObjectConverterNode(ros::NodeHandle &n);
    ~ObjectConverterNode();

    ParametersNode *param();
    void init();
    void callbackObjectDetections(const tuw_object_msgs::ObjectDetection &_msg);
    bool getStaticTF(std::string source_frame, mrpt::poses::CPose3D &des);

  private:

    ros::NodeHandle n_;
    ros::Subscriber sub_object_detections_;
    ros::Publisher pub_bearings_;
    ParametersNode* params_;
    tf::TransformListener listenerTF_;
    mrpt::poses::CPose3D map_pose_;
    std::map<std::string, mrpt::poses::CPose3D> static_tf_;
};

#endif // OBJECT_LISTENER_NODE_H
