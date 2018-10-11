#include <mrpt_object_converter_node.h>
#include <mrpt_bridge/pose.h>
#include <mrpt_bridge/landmark.h>
#include <string>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

ObjectConverterNode::ParametersNode::ParametersNode() : nh("~")
{
    nh.param<std::string>(std::string("tf_prefix"), tf_prefix, "");
    nh.param<bool>("debug", debug, false);
    nh.param<std::string>(std::string("subscriber_topic_name"), subscriber_topic_name, std::string("map_doors"));
    nh.param<std::string>(std::string("publisher_topic_name"), publisher_topic_name, std::string("bearing_gt"));
    nh.param<float>(std::string("contour_offset"), contour_offset, 0.25);
    nh.param<bool>("contour_filtering", contour_filtering, false);
    nh.param<bool>("robot_perspective", robot_perspective, false);
    nh.param<std::string>("world_frame", world_frame_name, std::string("map"));
    nh.param<std::string>("source_frame", source_frame_name, std::string("r0/laser0"));
    ROS_INFO("tf_prefix: %s", tf_prefix.c_str());
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

  pub_bearings_ = n_.advertise<mrpt_msgs::ObservationObject>(param()->publisher_topic_name, 1, true);
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

bool ObjectConverterNode::convert(const tuw_object_msgs::ObjectWithCovariance &_obj, mrpt::obs::CObservationObject::TMeasurement &_meas)
{
  const auto o_id = _obj.object.ids[0];
  auto position = _obj.object.pose.position;
  auto orientation = _obj.object.pose.orientation;

  _meas.landmarkID = o_id;
  double dx = position.x - map_pose_.x();
  double dy = position.y - map_pose_.y();
  _meas.yaw = atan2(dy,dx);
  
  double dx_inv = map_pose_.x() - position.x;
  double dy_inv = map_pose_.y() - position.y;
  _meas.pitch = atan2(dy_inv,dx_inv);

  _meas.range = sqrt(dx*dx + dy*dy);

  cv::Vec2f offset = cv::Vec2f(dx,dy) * scale_factor;
  offset[0] *= param()->contour_offset;
  offset[1] *= param()->contour_offset;
  offset = -offset;

  float dx_offset = dx * scale_factor + offset[0] + add_factor;
  float dy_offset = dy * scale_factor + offset[1] + add_factor;
  
  mrpt::poses::CPose3D p_so;
  mrpt_bridge::convert(_obj.object.pose, p_so);
  _meas.pose_so = p_so;
  
  if ( !getTF( param()->source_frame_name, map_pose_ ) )
  {
    double maxval = std::numeric_limits<double>::max();
    _meas.pose_wo = mrpt::poses::CPose3D(maxval, maxval, maxval,0,0,0);
  }
  else 
  {
  	_meas.pose_wo = map_pose_ + p_so; 
  }
  
  const auto shape_vars = _obj.object.shape_variables;
  _meas.shape_variables.resize(shape_vars.size());
  unsigned int cnt = 0;
  for (const double sv : shape_vars)
  {
    _meas.shape_variables[cnt++] = sv;
  }
  
  if (param()->contour_filtering)
  {
      cv::Point2f cv_d;
      cv_d.x = dx_offset;
      cv_d.y = dy_offset;

      if(cv::pointPolygonTest(contour_,cv_d,false) >= 0)
      {
        return true;
      }
  }
  else
  {
      return true;
  }
  
  return false;
}

void ObjectConverterNode::callbackObjectDetections(const tuw_object_msgs::ObjectDetection &_msg)
{
  using namespace mrpt::obs;
  using namespace mrpt::poses;

  if (param()->contour_filtering && !contour_.size())
  {
    return;
  }

  CObservationObject obs;
  MRPT_TODO("sensor pose assumed at 0, this is not true");
  obs.setSensorPose(CPose3D(0,0,0,0,0,0));
  obs.fieldOfView_pitch = M_PI/180.0 * 270.0;

  std::vector<cv::Point2f> vpb;

  for (std::vector<tuw_object_msgs::ObjectWithCovariance>::const_iterator it = _msg.objects.begin();
       it != _msg.objects.end(); ++it)
  {
    if (it->object.shape == tuw_object_msgs::Object::SHAPE_DOOR)
    {
      CObservationObject::TMeasurement meas;
      if(convert(*it,meas))
      {
        obs.sensedData.push_back(meas);
      }
    }
  }

  if (obs.sensedData.size() > 0)
  {
    mrpt_msgs::ObservationObject obs_msg;
    obs_msg.header.stamp = _msg.header.stamp;
    obs_msg.header.frame_id = _msg.header.frame_id;
    mrpt_bridge::convert(obs, obs_msg);
    std::cout << "publishing " << obs_msg.sensed_data.size() << " bearings " << std::endl;
    pub_bearings_.publish(obs_msg);
  }

  already_printed_ = true;
  
}

bool ObjectConverterNode::getTF(std::string source_frame, mrpt::poses::CPose3D &des)
{
  std::string target_frame_id = tf::resolve(param()->tf_prefix, param()->world_frame_name);
  std::string source_frame_id = source_frame;
  std::string key = target_frame_id + "->" + source_frame_id;
  mrpt::poses::CPose3D pose;
  tf::StampedTransform transform;

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
    //ROS_INFO("Static tf '%s' with '%s'",
    //         key.c_str(), pose.asString().c_str());
  }
  catch (tf::TransformException ex)
  {
    ROS_INFO("getStaticTF");
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
    return false;
  }
  
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
