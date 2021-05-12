#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h> // for tf::getPrefixParam()
#include <tf/transform_datatypes.h>

std::string g_topic;
std::string g_frame_id;
std::string g_footprint_frame_id;
std::string g_position_frame_id;
std::string g_stabilized_frame_id;
std::string g_child_frame_id;

bool g_publish_roll_pitch;
bool g_invert_tf;

std::string g_tf_prefix;

tf::TransformBroadcaster *g_transform_broadcaster;

#ifndef TF_MATRIX3x3_H
  typedef btScalar tfScalar;
  namespace tf { typedef btMatrix3x3 Matrix3x3; }
#endif

void addTransform(std::vector<geometry_msgs::TransformStamped>& transforms, const tf::StampedTransform& tf)
{
  transforms.push_back(geometry_msgs::TransformStamped());
  tf::transformStampedTFToMsg(tf, transforms.back());
}

void sendTransform(geometry_msgs::Pose const &pose, const std_msgs::Header& header, std::string child_frame_id = "")
{
  std::vector<geometry_msgs::TransformStamped> transforms;

  tf::StampedTransform tf;
  tf.stamp_ = header.stamp;

  tf.frame_id_ = header.frame_id;
  if (!g_frame_id.empty()) tf.frame_id_ = g_frame_id;
  tf.frame_id_ = tf::resolve(g_tf_prefix, tf.frame_id_);

  if (!g_child_frame_id.empty()) child_frame_id = g_child_frame_id;
  if (child_frame_id.empty()) child_frame_id = "base_link";

  tf::Quaternion orientation;
  tf::quaternionMsgToTF(pose.orientation, orientation);
  tfScalar yaw, pitch, roll;
  tf::Matrix3x3(orientation).getEulerYPR(yaw, pitch, roll);
  tf::Point position;
  tf::pointMsgToTF(pose.position, position);

  // position intermediate transform (x,y,z)
  if( !g_position_frame_id.empty() && child_frame_id != g_position_frame_id) {
    tf.child_frame_id_ = tf::resolve(g_tf_prefix, g_position_frame_id);
    tf.setOrigin(tf::Vector3(position.x(), position.y(), position.z() ));
    tf.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

    if (g_invert_tf) {
      auto temp = tf.inverse();
      tf.setOrigin(temp.getOrigin());
      tf.setRotation(temp.getRotation());
    }

    addTransform(transforms, tf);
  }

  // footprint intermediate transform (x,y,yaw)
  if (!g_footprint_frame_id.empty() && child_frame_id != g_footprint_frame_id) {
    tf.child_frame_id_ = tf::resolve(g_tf_prefix, g_footprint_frame_id);
    tf.setOrigin(tf::Vector3(position.x(), position.y(), 0.0));
    tf.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, yaw));

    if (g_invert_tf) {
      auto temp = tf.inverse();
      tf.setOrigin(temp.getOrigin());
      tf.setRotation(temp.getRotation());
    }

    addTransform(transforms, tf);

    yaw = 0.0;
    position.setX(0.0);
    position.setY(0.0);
    tf.frame_id_ = tf::resolve(g_tf_prefix, g_footprint_frame_id);
  }

  // stabilized intermediate transform (z)
  if (!g_footprint_frame_id.empty() && child_frame_id != g_stabilized_frame_id) {
    tf.child_frame_id_ = tf::resolve(g_tf_prefix, g_stabilized_frame_id);
    tf.setOrigin(tf::Vector3(0.0, 0.0, position.z()));
    tf.setBasis(tf::Matrix3x3::getIdentity());
    
    if (g_invert_tf) {
      auto temp = tf.inverse();
      tf.setOrigin(temp.getOrigin());
      tf.setRotation(temp.getRotation());
    }

    addTransform(transforms, tf);

    position.setZ(0.0);
    tf.frame_id_ = tf::resolve(g_tf_prefix, g_stabilized_frame_id);
  }

  // base_link transform (roll, pitch)
  if (g_publish_roll_pitch) {
    tf.child_frame_id_ = tf::resolve(g_tf_prefix, child_frame_id);
    tf.setOrigin(position);
    tf.setRotation(tf::createQuaternionFromRPY(roll, pitch, yaw));
    
    if (g_invert_tf) {
      auto temp = tf.inverse();
      tf.setOrigin(temp.getOrigin());
      tf.setRotation(temp.getRotation());
    }

    addTransform(transforms, tf);
  }

  g_transform_broadcaster->sendTransform(transforms);
}

void odomCallback(nav_msgs::Odometry const &odometry) {
  sendTransform(odometry.pose.pose, odometry.header, odometry.child_frame_id);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "message_to_tf");

  ros::NodeHandle priv_nh("~");
  priv_nh.getParam("topic", g_topic);
  priv_nh.getParam("frame_id", g_frame_id);
  priv_nh.getParam("footprint_frame_id", g_footprint_frame_id);
  priv_nh.getParam("position_frame_id", g_position_frame_id);
  priv_nh.getParam("stabilized_frame_id", g_stabilized_frame_id);
  priv_nh.getParam("child_frame_id", g_child_frame_id);

  // get topic from the commandline
  if (argc > 1) {
      g_topic = argv[1];
  }

  g_publish_roll_pitch = true;
  priv_nh.getParam("publish_roll_pitch", g_publish_roll_pitch);

  g_invert_tf = false;
  priv_nh.getParam("invert_tf", g_invert_tf);

  g_tf_prefix = tf::getPrefixParam(priv_nh);
  g_transform_broadcaster = new tf::TransformBroadcaster;

  ros::NodeHandle node;
  ros::Subscriber sub;

  if (!g_topic.empty()) {
    ROS_WARN("Listening to %s", g_topic.c_str());
    sub = node.subscribe(g_topic, 10, &odomCallback);
  }
  else {
    ROS_FATAL("Usage: rosrun message_to_tf message_to_tf <topic>");
    return 1;
  }

  ros::spin();
  delete g_transform_broadcaster;
  return 0;
}
