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
    nh.param<std::string>(std::string("tf_prefix"), tf_prefix, "");
    nh.param<bool>("debug", debug, false);
    nh.param<std::string>(std::string("subscriber_topic_name"), subscriber_topic_name, std::string("map_doors"));
    nh.param<std::string>(std::string("publisher_topic_name"), publisher_topic_name, std::string("bearing_gt"));
    nh.param<bool>("contour_filtering", contour_filtering, false);
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
  float offset_factor = 0;
  size_t nr =  _laser_msg.ranges.size();
  contour_.resize(nr+1);
  size_t i;
  unsigned int contour_cnt;
  //float min_x, min_y, max_x, max_y;
  //min_x = min_y = std::numeric_limits<float>::max();
  //max_x = max_y = -std::numeric_limits<float>::max();
  for ( i = 0, contour_cnt = 0; i < nr; i++ ) {
        double length = _laser_msg.ranges[i];

        if ( ( length < _laser_msg.range_max ) && isfinite ( length ) ) {
            double angle  = _laser_msg.angle_min + ( _laser_msg.angle_increment * i );
            cv::Point2f p;
            p.x = cos ( angle ) * length + offset_factor;
            p.y = sin ( angle ) * length + offset_factor;
            contour_[contour_cnt] = p;
            contour_cnt++;
            //min_x = std::min(p.x,min_x);
            //max_x = std::max(p.x,max_x);
            //min_y = std::min(p.y,min_y);
            //max_y = std::max(p.y,max_y);
        }

  }
  contour_[contour_cnt] = cv::Point2f(0,0);
  contour_cnt++;
  if (contour_cnt < contour_.size())
  {
      contour_.resize(contour_cnt);
  }
  //x_range = static_cast<int>(std::ceil((max_x - min_x) * scale_factor));
  //y_range = static_cast<int>(std::ceil((max_y - min_y) * scale_factor));

  std::vector<cv::Point2f> hull;
  cv::convexHull(contour_, hull, false);
  contour_ = hull;
}

void ObjectConverterNode::callbackObjectDetections(const tuw_object_msgs::ObjectDetection &_msg)
{
  //cv::Mat img = cv::Mat::zeros(x_range,y_range,CV_8UC3);
  //for (std::vector<cv::Point2f>::iterator p=contour_.begin(); p < contour_.end()-1; ++p)
  //{
  //  cv::line(img, *p * scale_factor, *(p+1) * scale_factor, cv::Scalar(255,255,255),2);
  //}
  //cv::line(img, contour_[contour_.size()-1] * scale_factor,contour_[0] * scale_factor, cv::Scalar(255,255,255),2);

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

  //std::cout << "=====CONTOUR=====" << std::endl;
  //for (const auto &p : contour_)
  //{
  //    std::cout << p << std::endl;
  //}
  //std::cout << "=====CONTOUR=====" << std::endl;

  for (std::vector<tuw_object_msgs::ObjectWithCovariance>::const_iterator it = _msg.objects.begin();
       it != _msg.objects.end(); ++it)
  {
    if (it->object.shape == tuw_object_msgs::Object::SHAPE_DOOR)
    {
      const auto o_id = it->object.ids[0];
      auto position = it->object.pose.position;
      auto orientation = it->object.pose.orientation;

      CBearing::Ptr bear;
      //Eigen::Quaterniond q_it(orientation.w,orientation.x,orientation.y,orientation.z);
      //mrpt::math::CMatrixDouble44 m_it;
      //m_it.block<3,3>(0,0) = q_it.toRotationMatrix();
      //m_it.block<3,1>(0,3) = Eigen::Matrix<double,3,1>(position.x,position.y,position.z);
      //m_it.block<1,4>(3,0) = Eigen::Matrix<double,1,4>(0,0,0,1);

      //CPose3D pose(m_it);

      //if (!already_printed_)
      //{
      //  std::cout << "pose " << o_id << " " << pose << std::endl;
      //}

      //double door_angle = it->object.shape_variables[3];
      //bool clockwise = ((int) it->object.shape_variables[5] == 0) ? true : false;
      //{
      //  double c = 0, s = 0;
      //  if (clockwise)
      //  {
      //    c = cos(-door_angle);
      //    s = sin(-door_angle);
      //  } else {
      //    c = cos(door_angle);
      //    s = sin(door_angle);
      //  }
      //  std::vector<double> mat_data = {c,-s,0,s,c,0,0,0,1};
      //  mrpt::math::CMatrixDouble33 R_d(std::move(mat_data.data()));
      //  pose.setRotationMatrix(pose.getRotationMatrix() * R_d);
      //}

      CObservationBearingRange::TMeasurement d;
      {
        d.landmarkID = o_id;
        d.pitch = 0;
        double dx = position.x - map_pose_.x();
        double dy = position.y - map_pose_.y();
        d.yaw = atan2(dy,dx);
        d.range = sqrt(dx*dx + dy*dy);

        double offset = 0.25;
        //cv::circle(img, cv::Point2f(dy,dx) * scale_factor, 2, cv::Scalar(255,0,0));
        //printf("(%lf,%lf)\n", dx,dy);
        if (param()->contour_filtering)
        {
            if(cv::pointPolygonTest(contour_,cv::Point2f(dy < 0 ? dy+offset : dy-offset,dx < 0 ? dx+offset : dx-offset),false) >= 0)
            {
                //Filter by means of laser scan
                obs.sensedData.push_back(d);
            }
        }
        else
        {
            obs.sensedData.push_back(d);
        }
      }
    }
  }

  if (obs.sensedData.size() > 0)
  {
        mrpt_msgs::ObservationRangeBearing obs_msg;
        obs_msg.header.stamp = _msg.header.stamp;
        obs_msg.header.frame_id = _msg.header.frame_id;
        mrpt_bridge::convert(obs, obs_msg);
        pub_bearings_.publish(obs_msg);
  }

  //cv::imshow("LaserScanPoly", img);
  //cv::waitKey(2);
  ROS_INFO("published bearings: %d\n", obs.sensedData.size());
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
