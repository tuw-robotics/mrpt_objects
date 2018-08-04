#include <mrpt_object_converter_node.h>
#include <mrpt_bridge/map.h>
#include <mrpt_bridge/pose.h>
#include <mrpt_bridge/bearing.h>
#include <mrpt/maps/CBearing.h>
#include <mrpt/maps/CBearingMap.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/system/filesystem.h>

#include <mrpt/io/CFileOutputStream.h>
#include <mrpt/io/CFileInputStream.h>

#include <string>

#include <mrpt/obs/CObservationBearingRange.h>
#include <mrpt/math/types_math.h>

#include <mrpt/config/CConfigFile.h>

#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CSphere.h>
#include <mrpt/opengl/stock_objects.h>

ObjectConverterNode::ParametersNode::ParametersNode() : nh("~")
{
    nh.param<std::string>(std::string("base_frame_id"), base_frame_id, "base_link");
    nh.param<std::string>(std::string("tf_prefix"), tf_prefix, "");
    nh.param<bool>("debug", debug, false);
    ROS_INFO("tf_prefix: %s", tf_prefix.c_str());
    ROS_INFO("base_frame_id: %s", base_frame_id.c_str());
}

ObjectConverterNode::ObjectConverterNode(ros::NodeHandle& n) : n_(n)
{
  params_ = new ObjectConverterNode::ParametersNode();
}

ObjectConverterNode::~ObjectConverterNode()
{
  delete params_;
}

ObjectConverterNode::ParametersNode* ObjectConverterNode::param()
{
  return (ObjectConverterNode::ParametersNode*) params_;
}

void ObjectConverterNode::init()
{
  sub_object_detections_ = n_.subscribe("map_doors", 1, &ObjectConverterNode::callbackObjectDetections, this);

  pub_bearings_ = n_.advertise<mrpt_msgs::ObservationRangeBearing>("bearings", 1, true);
}

void ObjectConverterNode::callbackObjectDetections(const tuw_object_msgs::ObjectDetection &_msg)
{
  using namespace mrpt::obs;
  using namespace mrpt::maps;
  using namespace mrpt::containers;
  using namespace mrpt::poses;

  CObservationBearingRange obs;
  obs.setSensorPose(map_pose_);
  obs.fieldOfView_pitch = M_PI/180.0 * 270.0;

  for (std::vector<tuw_object_msgs::ObjectWithCovariance>::const_iterator it = _msg.objects.begin();
       it != _msg.objects.end(); ++it)
  {
    if (it->object.shape == tuw_object_msgs::Object::SHAPE_DOOR)
    {
      const auto o_id = it->object.ids[0];
      CBearing::Ptr bear;

      CPose3D pose;
      mrpt_bridge::convert(it->object.pose, pose);

      CObservationBearingRange::TMeasurement d;
      {
        d.landmarkID = o_id;
        d.pitch = 0;
        double dx = pose.x() - map_pose_.x();
        double dy = pose.y() - map_pose_.y();
        d.yaw = atan2(dy, dx);
        d.range = pose.distance3DTo(map_pose_.x(), map_pose_.y(), pose.z());
      }
      obs.sensedData.push_back(d);
    }
  }
  mrpt_msgs::ObservationRangeBearing obs_msg;
  mrpt_bridge::convert(obs, obs_msg);
  pub_bearings_.publish(obs_msg);
}

bool ObjectConverterNode::getStaticTF(std::string source_frame, mrpt::poses::CPose3D &des)
{
  std::string target_frame_id = tf::resolve(param()->tf_prefix, param()->base_frame_id);
  std::string source_frame_id = source_frame;
  std::string key = target_frame_id + "->" + source_frame_id;
  mrpt::poses::CPose3D pose;
  tf::StampedTransform transform;

  if (static_tf_.find(key) == static_tf_.end()) {

      try
      {
          if (param()->debug)
              ROS_INFO(
                  "debug: updateLaserPose(): target_frame_id='%s' source_frame_id='%s'",
                  target_frame_id.c_str(), source_frame_id.c_str());

          listenerTF_.lookupTransform(
              target_frame_id, source_frame_id, ros::Time(0), transform);
          tf::Vector3 translation = transform.getOrigin();
          tf::Quaternion quat = transform.getRotation();
          pose.x() = translation.x();
          pose.y() = translation.y();
          pose.z() = translation.z();
          tf::Matrix3x3 Rsrc(quat);
          mrpt::math::CMatrixDouble33 Rdes;
          for (int c = 0; c < 3; c++)
              for (int r = 0; r < 3; r++) Rdes(r, c) = Rsrc.getRow(r)[c];
          pose.setRotationMatrix(Rdes);
          static_tf_[key] = pose;
          ROS_INFO("Static tf '%s' with '%s'",
                   key.c_str(), pose.asString().c_str());
      }
      catch (tf::TransformException ex)
      {
          ROS_INFO("getStaticTF");
          ROS_ERROR("%s", ex.what());
          ros::Duration(1.0).sleep();
          return false;
      }
  }
  des = static_tf_[key];
  return true;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_listener_node");
  ros::NodeHandle nh;

  ObjectConverterNode obj_listener_node(nh);
  obj_listener_node.init();
  ros::Rate rate(2);

  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
}
