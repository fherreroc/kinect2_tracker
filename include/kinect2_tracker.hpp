/**
* \ref kinect2_tracker.hpp
*
*  \date 20160322
*  \author Stephen Reddish
*  \version 1.0
*  \bug
*   It will be quicker to work out the vec3s before passing them to the transform publisher
*   Solve bodgey if torso elses 
*  \copyright GNU Public License.
*/

#ifndef KINECT2_TRACKER_HPP_
#define KINECT2_TRACKER_HPP_

// ROS Dependencies
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <iostream>
#include "user_IDs.h" 
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/distortion_models.h>
#include <image_transport/image_transport.h>
#include "NiTE.h"
#include <sensor_msgs/image_encodings.h>
#include <Eigen/Geometry> 
#include <tf_conversions/tf_eigen.h>

#ifndef ALPHA
#define ALPHA 1/256
#endif

#define MAX_USERS 10

#define USER_MESSAGE(msg) \
        {printf("[%08llu] User #%d:\t%s\n",ts, user.getId(),msg);}

/// Joint map
typedef std::map<std::string, nite::SkeletonJoint> JointMap;

/**
* Class \ref kinect2_tracker. This class can track the skeleton of people and returns joints as a TF stream,
*/
class kinect2_tracker
{
public:
  /**
  * Constructor
  */
  kinect2_tracker() :
     it(nh_)
  {

    // Get some parameters from the server
    ros::NodeHandle pnh("~");
    if (!pnh.getParam("tf_prefix", tf_prefix_))
    {
      ROS_FATAL("tf_prefix not found on Param Server! Maybe you should add it to your launch file!");
      ros::shutdown();
      return;
    }

    if (!pnh.getParam("relative_frame", relative_frame_))
    {
      ROS_FATAL("relative_frame not found on Param Server! Maybe you should add it to your launch file!");
      ros::shutdown();
      return;
    }

    // Initialize OpenNI
    if (openni::OpenNI::initialize() != openni::STATUS_OK)
    {
      ROS_FATAL("OpenNI initial error");
      ros::shutdown();
      return;
    }

    // Open the device
    if (devDevice_.open(openni::ANY_DEVICE) != openni::STATUS_OK)
    {
      ROS_FATAL("Can't Open Device");
      ros::shutdown();
      return;
    }
    else
      ROS_INFO("Device opened");

    if(this->depth_stream.create(this->devDevice_,openni::SENSOR_DEPTH)==openni::STATUS_OK)
    {
      //Set depth mode
//      const openni::SensorInfo* sinfo = this->devDevice_.getSensorInfo(openni::SENSOR_DEPTH);
//      const openni::Array< openni::VideoMode>& modesDepth = sinfo->getSupportedVideoModes();
//      if(this->depth_stream.setVideoMode(modesDepth[0]) != openni::STATUS_OK)
//        ROS_ERROR("Depth format not supported...");

      if(this->depth_stream.start()!=openni::STATUS_OK)
      {
        ROS_FATAL("Impossible to start depth stream");
        ros::shutdown();
        return;
      }
      else
      {
        ROS_INFO("Depth stream initialize successfully");
      }
    }
    else
    {
      ROS_FATAL("Impossible to create depth stream");
      ros::shutdown();
      return;
    }

    // Initialize the tracker
    nite::NiTE::initialize();

    // user tracker registration
    niteRc_ = userTracker_.create();
    if (niteRc_ != nite::STATUS_OK)
    {
      ROS_FATAL("Couldn't create user tracker");
      ros::shutdown();
      return;
    }

    // Initialize the users IDs publisher
    userPub_ = nh_.advertise<skeleton_tracker::user_IDs>("/people", 1);

    this->image_out_publisher_ = nh_.advertise<sensor_msgs::Image>("image_out/image_raw", 1);
    this->camera_publisher = this->it.advertiseCamera("depth/image_raw", 1);

    rate_ = new ros::Rate(100);

  }
  /**
  * Destructor
  */
  ~kinect2_tracker()
  {
    nite::NiTE::shutdown();
  }

  /**
  * Spinner!!!
  */
  void spinner()
  {
    // Broadcast the joint frames (if they exist)
    this->getSkeleton();

    //Depth
    bool mirror=true;
    this->depth_stream.setMirroringEnabled(mirror);

    this->depth_stream.readFrame(&this->depth_frame);
    //ROS_INFO("Depth stream resolution %dx%d, data size: %d", this->depth_frame.getHeight(), this->depth_frame.getWidth(),this->depth_frame.getDataSize());
    //std::cout << "height: " << this->depth_frame.getHeight() << " width: " << this->depth_frame.getWidth() << std::endl;
    //std::cout << "data size: " << this->depth_frame.getDataSize() << std::endl;
    this->image_out_Image_msg_.header.stamp=ros::Time::now();
    this->image_out_Image_msg_.height=this->depth_frame.getHeight();
    this->image_out_Image_msg_.width=this->depth_frame.getWidth();
    this->image_out_Image_msg_.encoding="mono16";
    this->image_out_Image_msg_.step=this->depth_frame.getWidth()*2;
    this->image_out_Image_msg_.data.resize(this->depth_frame.getWidth()*this->depth_frame.getHeight()*2);
    memcpy(this->image_out_Image_msg_.data.data(),this->depth_frame.getData(),this->depth_frame.getWidth()*this->depth_frame.getHeight()*2);
    this->image_out_publisher_.publish(this->image_out_Image_msg_);

    
    this->depth_img.header.stamp=ros::Time::now();
    this->depth_img.header.frame_id="kinect2_link";
    this->depth_img.height=this->depth_frame.getHeight();
    this->depth_img.width=this->depth_frame.getWidth();
    this->depth_img.encoding="mono16";
    this->depth_img.step=this->depth_frame.getWidth()*2;
    this->depth_img.data.resize(this->depth_frame.getWidth()*this->depth_frame.getHeight()*2);
    memcpy(this->depth_img.data.data(),this->depth_frame.getData(),this->depth_frame.getWidth()*this->depth_frame.getHeight()*2);
    
    this->depth_info.header.stamp    = this->depth_img.header.stamp;
    this->depth_info.header.frame_id = this->depth_img.header.frame_id;
    this->depth_info.height          = this->depth_img.height;
    this->depth_info.width           = this->depth_img.width;
    this->depth_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

    float cx = 254.878f;
    float cy = 205.395f;
    float fx = 365.456f;
    float fy = 365.456f;
    float k1 = 0.0905474;
    float k2 = -0.26819;
    float k3 = 0.0950862;
    float p1 = 0.0;
    float p2 = 0.0; 

    //float fx = devDevice_.getDepthFocalLength (); // Horizontal focal length
    //float fy = devDevice_.getDepthFocalLength (); // Vertical focal length
    //float cx = ((float)this->depth_frame.getWidth() - 1.f) / 2.f;  // Center x
    //float cy = ((float)this->depth_frame.getHeight()- 1.f) / 2.f; // Center y    
    //double k1,k2,k3,p1,p2=0.0;

    this->depth_info.D.resize(5);
    this->depth_info.D[0] = k1;
    this->depth_info.D[1] = k2;
    this->depth_info.D[2] = k3;
    this->depth_info.D[3] = p1;
    this->depth_info.D[4] = p2;


    
    this->depth_info.K.fill(0.0);
    this->depth_info.K[0] = fx;
    this->depth_info.K[2] = cx;
    this->depth_info.K[4] = fy;
    this->depth_info.K[5] = cy;
    this->depth_info.K[8] = 1.0;

    this->depth_info.R.fill(0.0);

    this->depth_info.P.fill(0.0);
    this->depth_info.P[0] = fx;
    this->depth_info.P[2] = cx;
    this->depth_info.P[5] = fy;
    this->depth_info.P[6] = cy;
    this->depth_info.P[10] = 1.0;
    
    this->camera_publisher.publish(this->depth_img, this->depth_info);

    rate_->sleep();
  }

  /**
  * Update the Users State
  * @param user: the user
  * @param ts: timestamp
  */
  void updateUserState(const nite::UserData& user, unsigned long long ts)
  {
    if (user.isNew())
      USER_MESSAGE("New")
    else if (user.isVisible() && !g_visibleUsers_[user.getId()])
      USER_MESSAGE("Visible")
    else if (!user.isVisible() && g_visibleUsers_[user.getId()])
      USER_MESSAGE("Out of Scene")
    else if (user.isLost())
      USER_MESSAGE("Lost")

    g_visibleUsers_[user.getId()] = user.isVisible();

    if (g_skeletonStates_[user.getId()] != user.getSkeleton().getState())
    {
      switch (g_skeletonStates_[user.getId()] = user.getSkeleton().getState())
      {
        case nite::SKELETON_NONE:
          USER_MESSAGE("Stopped tracking.")
          break;
        case nite::SKELETON_CALIBRATING:
          USER_MESSAGE("Calibrating...")
          break;
        case nite::SKELETON_TRACKED:
          USER_MESSAGE("Tracking!")
          break;
        case nite::SKELETON_CALIBRATION_ERROR_NOT_IN_POSE:
        case nite::SKELETON_CALIBRATION_ERROR_HANDS:
        case nite::SKELETON_CALIBRATION_ERROR_LEGS:
        case nite::SKELETON_CALIBRATION_ERROR_HEAD:
        case nite::SKELETON_CALIBRATION_ERROR_TORSO:
          USER_MESSAGE("Calibration Failed... :-|")
          break;
      }
    }
  }

  /**
  * Publish the joints over the TF stream
  * @param actual_joint_name: joint name
  * @param actual_joint: the current joint
  * @param parent_joint_name: parent joint name connected to actual_joint
  * @param parent_joint: the parent joint
  * @param uid: user's ID
  */
  void publishJointTF(std::string         actual_joint_name, 
                      nite::SkeletonJoint actual_joint, 
                      std::string         parent_joint_name, 
                      nite::SkeletonJoint parent_joint, 
                      int                 uid) 
  {
    if (actual_joint.getPositionConfidence() > 0.0)
    {
      tf::Vector3 actualPos = tf::Vector3(actual_joint.getPosition().x / 1000.0, actual_joint.getPosition().y / 1000.0, actual_joint.getPosition().z / 1000.0);
      tf::Vector3 parentPos = tf::Vector3(parent_joint.getPosition().x / 1000.0, parent_joint.getPosition().y / 1000.0, parent_joint.getPosition().z / 1000.0);
      
      tf::Quaternion actualRot;
      if(actual_joint.getOrientationConfidence() > 0.0)
        actualRot = tf::Quaternion(actual_joint.getOrientation().x, actual_joint.getOrientation().y, actual_joint.getOrientation().z, actual_joint.getOrientation().w);
      else
        actualRot = tf::Quaternion(0,0,0,1);
      
      tf::Quaternion parentRot;
      if(parent_joint.getOrientationConfidence() > 0.0)
        parentRot = tf::Quaternion(parent_joint.getOrientation().x, parent_joint.getOrientation().y, parent_joint.getOrientation().z, parent_joint.getOrientation().w);
      else
        parentRot = tf::Quaternion(0,0,0,1);

      tf::Transform transform;
      if(actual_joint_name!="torso")
      {
        transform.setOrigin(actualPos);
        transform.setRotation(actualRot);
        tf::Transform transform_parent;
        transform_parent.setOrigin(parentPos);
        transform_parent.setRotation(parentRot);
        transform = transform_parent.inverse()*transform;
      }
      else
      {
        transform.setOrigin(actualPos);
        transform.setRotation(actualRot);
      }

      std::stringstream actual_frame_id_stream;
      actual_frame_id_stream << "/" << tf_prefix_ << "/user_" << uid << "/" << actual_joint_name;
      std::string actual_frame_id = actual_frame_id_stream.str();

      std::stringstream parent_frame_id_stream;
      parent_frame_id_stream << "/" << tf_prefix_ << "/user_" << uid << "/" << parent_joint_name;
      std::string parent_frame_id = parent_frame_id_stream.str();

      if(actual_joint_name!="torso")
        tfBroadcast_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent_frame_id, actual_frame_id));
      else
        tfBroadcast_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent_joint_name, actual_frame_id));

    }
    return;
  }

  /**
  * Get the skeleton's joints and the users IDs and make them all relative to the Torso joint
  */
  void getSkeleton()
  {
    skeleton_tracker::user_IDs ids;
    niteRc_ = userTracker_.readFrame(&userTrackerFrame_);
    if (niteRc_ != nite::STATUS_OK)
    {
      printf("Get next frame failed\n");
      return;
    }

    // Get all the users
    const nite::Array<nite::UserData>& users = userTrackerFrame_.getUsers();

    // Get the skeleton for every user
    for (int i = 0; i < users.getSize(); ++i)
    {
      const nite::UserData& user = users[i];
      updateUserState(user, userTrackerFrame_.getTimestamp());
      if (user.isNew())
      {
        userTracker_.startSkeletonTracking(user.getId());
      }
      else if (user.getSkeleton().getState() == nite::SKELETON_TRACKED)
      {
        JointMap named_joints;

        named_joints["torso"]          = (user.getSkeleton().getJoint(nite::JOINT_TORSO));
        named_joints["left_hip"]       = (user.getSkeleton().getJoint(nite::JOINT_LEFT_HIP));
        named_joints["right_hip"]      = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP));
        named_joints["left_knee"]      = (user.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE));
        named_joints["right_knee"]     = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE));
        named_joints["left_foot"]      = (user.getSkeleton().getJoint(nite::JOINT_LEFT_FOOT));
        named_joints["right_foot"]     = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_FOOT));
        named_joints["neck"]           = (user.getSkeleton().getJoint(nite::JOINT_NECK));  
        named_joints["head"]           = (user.getSkeleton().getJoint(nite::JOINT_HEAD));
        named_joints["left_shoulder"]  = (user.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER));
        named_joints["right_shoulder"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER));
        named_joints["left_elbow"]     = (user.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW));
        named_joints["right_elbow"]    = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW));
        named_joints["left_hand"]      = (user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND));
        named_joints["right_hand"]     = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND));

        //Publish the joint (name, niteConstruct, ConnectedJoint name, niteConstruct, User)
        publishJointTF("torso",          named_joints["torso"],          relative_frame_,  named_joints["torso"], user.getId());
        publishJointTF("left_hip",       named_joints["left_hip"],       "torso",          named_joints["torso"], user.getId());
        publishJointTF("right_hip",      named_joints["right_hip"],      "torso",          named_joints["torso"], user.getId());
        publishJointTF("neck",           named_joints["neck"],           "torso",          named_joints["torso"], user.getId());
        publishJointTF("head",           named_joints["head"],           "neck",           named_joints["neck"], user.getId());
        publishJointTF("left_shoulder",  named_joints["left_shoulder"],  "neck",           named_joints["neck"], user.getId());
        publishJointTF("right_shoulder", named_joints["right_shoulder"], "neck",           named_joints["neck"], user.getId());
        publishJointTF("left_elbow",     named_joints["left_elbow"],     "left_shoulder",  named_joints["left_shoulder"], user.getId());
        publishJointTF("right_elbow",    named_joints["right_elbow"],    "right_shoulder", named_joints["right_shoulder"], user.getId());
        publishJointTF("left_hand",      named_joints["left_hand"],      "left_elbow",     named_joints["left_elbow"], user.getId());
        publishJointTF("right_hand",     named_joints["right_hand"],     "right_elbow",    named_joints["right_elbow"], user.getId());
        /**/
        publishJointTF("left_knee",      named_joints["left_knee"],      "left_hip",       named_joints["left_hip"], user.getId());
        publishJointTF("right_knee",     named_joints["right_knee"],     "right_hip",      named_joints["right_hip"], user.getId());
        publishJointTF("left_foot",      named_joints["left_foot"],      "left_knee",      named_joints["left_knee"], user.getId());
        publishJointTF("right_foot",     named_joints["right_foot"],     "right_knee",     named_joints["right_knee"], user.getId());
        /**/

        // Add the user's ID
        ids.users.push_back(int(user.getId()));
      }
    }
    // Publish the users' IDs
    userPub_.publish(ids);
  }

  /// ROS NodeHandle
  ros::NodeHandle nh_;

  bool g_visibleUsers_[MAX_USERS] = {false};
  nite::SkeletonState g_skeletonStates_[MAX_USERS] = {nite::SKELETON_NONE};

  /// Image transport
  image_transport::ImageTransport it;
  image_transport::CameraPublisher camera_publisher;
  sensor_msgs::Image depth_img;
  sensor_msgs::CameraInfo depth_info;

  ros::Publisher image_out_publisher_;
  sensor_msgs::Image image_out_Image_msg_;

  std::string tf_prefix_, relative_frame_;

  /// Frame broadcaster
  tf::TransformBroadcaster tfBroadcast_;

  /// The openni device
  openni::Device devDevice_;
  openni::VideoFrameRef depth_frame;
  openni::VideoFrameRef color_frame;

  openni::VideoStream depth_stream;
  openni::VideoStream color_stream;

  /// Some NITE stuff
  nite::UserTracker userTracker_;
  nite::Status niteRc_;
  nite::UserTrackerFrameRef userTrackerFrame_;

  /// Users IDs publisher
  ros::Publisher userPub_;
  /// Image message
  sensor_msgs::ImagePtr msg_;

  /// Node rate
  ros::Rate* rate_;

}
;

#endif /* KINECT2_TRACKER_HPP_ */
