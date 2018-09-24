#include <mrpt_object_converter_node.h>
#include <mrpt_bridge/map.h>
#include <mrpt_bridge/pose.h>
#include <mrpt_bridge/landmark.h>
#include <mrpt/maps/CBearing.h>
#include <mrpt/maps/CBearingMap.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/math/CQuaternion.h>

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

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

ObjectConverterNode::ParametersNode::ParametersNode() : nh("~")
{
    nh.param<std::string>(std::string("base_frame_id"), base_frame_id, "base_link");
    nh.param<std::string>(std::string("source_frame_id"), source_frame_id, "r0/laser0");
    nh.param<std::string>(std::string("tf_prefix"), tf_prefix, "");
    nh.param<bool>("debug", debug, false);
    nh.param<std::string>(std::string("subscriber_topic_name"), subscriber_topic_name, std::string("map_doors"));
    nh.param<std::string>(std::string("publisher_topic_name"), publisher_topic_name, std::string("bearing_gt"));
    nh.param<float>(std::string("contour_offset"), contour_offset, 0.25);
    nh.param<bool>("contour_filtering", contour_filtering, false);
    nh.param<bool>("robot_perspective", robot_perspective, false);
    ROS_INFO("tf_prefix: %s", tf_prefix.c_str());
    ROS_INFO("base_frame_id: %s", base_frame_id.c_str());
    ROS_INFO("publisher topic name: %s", publisher_topic_name.c_str());
    ROS_INFO("subscribed to: %s", subscriber_topic_name.c_str());
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
  auto id_mat = mrpt::math::CMatrixDouble44();
  id_mat.setIdentity();
  map_pose_ = mrpt::poses::CPose3D(id_mat);

  sub_object_detections_ = n_.subscribe(param()->subscriber_topic_name, 1, &ObjectConverterNode::callbackObjectDetections, this);

  if (param()->contour_filtering)
  {
    sub_scan_ = n_.subscribe("scan", 1, &ObjectConverterNode::callbackScan, this);
  }

  pub_bearings_ = n_.advertise<mrpt_msgs::ObservationRangeBearing>(param()->publisher_topic_name, 1, true);
}

void ObjectConverterNode::callbackScan(const sensor_msgs::LaserScan &_laser_msg)
{
  size_t nr =  _laser_msg.ranges.size();
  contour_.resize(nr+1);
  size_t i;
  unsigned int contour_cnt;
  for ( i = 0, contour_cnt = 0; i < nr; i++ ) {
        double length = _laser_msg.ranges[i];

        if ( ( length < _laser_msg.range_max ) && isfinite ( length ) ) {
            double angle  = _laser_msg.angle_min + ( _laser_msg.angle_increment * i );
            cv::Point2f p;
            p.x = cos ( angle ) * length;
            p.y = sin ( angle ) * length;
            contour_[contour_cnt] = p * scale_factor + cv::Point2f(add_factor, add_factor);
            contour_cnt++;
        }

  }
  if (contour_cnt < contour_.size())
  {
      contour_.resize(contour_cnt);
  }

  std::vector<cv::Point2f> hull;
  cv::convexHull(contour_, hull, false);
  contour_ = hull;
}

void ObjectConverterNode::callbackObjectDetections(const tuw_object_msgs::ObjectDetection &_msg)
{
  using namespace mrpt::obs;
  using namespace mrpt::maps;
  using namespace mrpt::containers;
  using namespace mrpt::poses;

  if (param()->contour_filtering && !contour_.size())
  {
    return;
  }

  CObservationBearingRange obs;
  obs.setSensorPose(map_pose_);
  obs.fieldOfView_pitch = M_PI/180.0 * 270.0;

  std::vector<cv::Point2f> vpb;
  std::vector<bool> vbb;

  for (std::vector<tuw_object_msgs::ObjectWithCovariance>::const_iterator it = _msg.objects.begin();
       it != _msg.objects.end(); ++it)
  {
    if (it->object.shape == tuw_object_msgs::Object::SHAPE_DOOR)
    {
      const auto o_id = it->object.ids[0];
      auto position = it->object.pose.position;
      auto orientation = it->object.pose.orientation;

      CBearing::Ptr bear;

      double door_angle = ((int) it->object.shape_variables[5] == 0) ? -it->object.shape_variables[3] : it->object.shape_variables[3];

      CObservationBearingRange::TMeasurement d;
      {
        d.landmarkID = o_id;
        double dx = position.x - map_pose_.x();
        double dy = position.y - map_pose_.y();
        d.yaw = atan2(dy,dx);
        d.pitch = door_angle;
        d.range = sqrt(dx*dx + dy*dy);

        cv::Vec2f offset = cv::Vec2f(dx,dy) * scale_factor;
        offset[0] *= param()->contour_offset;
        offset[1] *= param()->contour_offset;
        offset = -offset;

        float dx_offset = dx * scale_factor + offset[0] + add_factor;
        float dy_offset = dy * scale_factor + offset[1] + add_factor;

        if (param()->contour_filtering)
        {
            cv::Point2f cv_d;
            cv_d.x = dx_offset;
            cv_d.y = dy_offset;
            //vpb.push_back(cv_d);
            //vbb.push_back(false);

            if(cv::pointPolygonTest(contour_,cv_d,false) >= 0)
            {
                obs.sensedData.push_back(d);
                //vbb.back() = true;
            }
        }
        else
        {
            obs.sensedData.push_back(d);
        }
      }
    }

    //if (param()->contour_filtering)
    //{
    //  cv::Mat img_display = cv::Mat::zeros(1000,1000,CV_8UC3);
    //  unsigned int incr = 0;
    //  for (const cv::Point2f &p_b : vpb)
    //  {
    //    //img_display.at<cv::Scalar>(cv::Point2i(p_b)) = cv::Scalar(0,255,0);
    //    cv::Scalar color;
    //    if (vbb[incr])
    //    {
    //      color = cv::Scalar(0,255,0);
    //    }
    //    else
    //    {
    //      color = cv::Scalar(0,0,255);
    //    }
    //    cv::circle(img_display, p_b,3,color,1);
    //    incr++;
    //  }

    //  for (std::vector<cv::Point2f>::iterator it = contour_.begin(); it != contour_.end(); ++it)
    //  {
    //    auto p_ls_0 = *it;
    //    auto it_next = it+1;
    //    if (it_next == contour_.end())
    //    {
    //      it_next = contour_.begin();
    //    }
    //    auto p_ls_1 = *(it_next);
    //    cv::line(img_display, p_ls_0, p_ls_1, cv::Scalar(255,0,0),2);
    //  }

    //  cv::imshow("contour outline", img_display);
    //  cv::waitKey(1);
    //}
  }

  if (obs.sensedData.size() > 0)
  {
        mrpt_msgs::ObservationRangeBearing obs_msg;
        obs_msg.header.stamp = _msg.header.stamp;
        obs_msg.header.frame_id = _msg.header.frame_id;
        mrpt_bridge::convert(obs, obs_msg);
        pub_bearings_.publish(obs_msg);
  }

  already_printed_ = true;
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
  //ros::Rate rate(2);

  while (ros::ok())
  {
    ros::spin();
  }
}
