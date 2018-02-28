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
#include <kinect2_tracker/user_IDs.h> 
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
class kinect2_tracker_node
{
public:
  /**
  * Constructor
  */
  kinect2_tracker_node() :
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

    if (!pnh.getParam("skeleton_frame", skeleton_frame_))
    {
      ROS_FATAL("skeleton_frame not found on Param Server! Maybe you should add it to your launch file!");
      ros::shutdown();
      return;
    }
    
    if (!pnh.getParam("camera_frame", camera_frame_))
    {
      ROS_FATAL("camera_frame not found on Param Server! Maybe you should add it to your launch file!");
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
      this->depth_stream.setMirroringEnabled(true);

      if(this->depth_stream.start()!=openni::STATUS_OK)
      {
        ROS_FATAL("Impossible to start depth stream");
        ros::shutdown();
        return;
      }
      else
        ROS_INFO("Depth stream initialize successfully");
    }
    else
    {
      ROS_FATAL("Impossible to create depth stream");
      ros::shutdown();
      return;
    }
   
    if(this->color_stream.create(this->devDevice_,openni::SENSOR_COLOR)==openni::STATUS_OK)
    {
      this->color_stream.setMirroringEnabled(true);
      if(this->color_stream.start()!=openni::STATUS_OK)
      {
        ROS_FATAL("Impossible to start color stream");
        ros::shutdown();
        return;
      }
      else
        ROS_INFO("Color stream initialize successfully");
    }
    else
    {
      ROS_FATAL("Impossible to create color stream");
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
    userPub_ = nh_.advertise<kinect2_tracker::user_IDs>("/people", 1);

    //this->image_out_publisher_ = nh_.advertise<sensor_msgs::Image>("image_out/image_raw", 1);
    this->depth_camera_publisher = this->it.advertiseCamera("depth/image_raw", 1);
    this->color_camera_publisher = this->it.advertiseCamera("color/image_raw", 1);

    rate_ = new ros::Rate(100);

  }
  /**
  * Destructor
  */
  ~kinect2_tracker_node()
  {
    nite::NiTE::shutdown();
  }

  /**
  * Spinner!!!
  */
  void spinner()
  {
    this->getSkeleton();
    this->publishImages();

    rate_->sleep();
  }
  
  void publishImages()
  {
    bool color=true;
    bool depth=true;
    //std::string frame = "kinect2_link";
    std::string frame = camera_frame_;
    

    if(color)
    {
      this->color_stream.readFrame(&this->color_frame);
      this->color_img.header.stamp    = ros::Time::now();
      this->color_img.header.frame_id = frame;
      this->color_img.height          = this->color_frame.getHeight();
      this->color_img.width           = this->color_frame.getWidth();
      this->color_img.encoding        = "rgb8"; 
      this->color_img.is_bigendian    = 0;
      this->color_img.step            = this->color_img.width*3;
      this->color_img.data.resize(this->color_frame.getWidth()*this->color_frame.getHeight()*3);
      memcpy(this->color_img.data.data(),this->color_frame.getData(),this->color_frame.getWidth()*this->color_frame.getHeight()*3);
  
      fillCameraInfo(this->color_info, this->color_img);
      this->color_camera_publisher.publish(this->color_img, this->color_info); 
    }

    if(depth)
    {
      this->depth_stream.readFrame(&this->depth_frame);    
      this->depth_img.header.stamp    = ros::Time::now();
      this->depth_img.header.frame_id = frame;
      this->depth_img.height          = this->depth_frame.getHeight();
      this->depth_img.width           = this->depth_frame.getWidth();
      this->depth_img.encoding        = "mono16";
      this->depth_img.step            = this->depth_frame.getWidth()*2;
      this->depth_img.data.resize(this->depth_frame.getWidth()*this->depth_frame.getHeight()*2);
      memcpy(this->depth_img.data.data(),this->depth_frame.getData(),this->depth_frame.getWidth()*this->depth_frame.getHeight()*2);

      fillCameraInfo(this->depth_info, this->depth_img);
      this->depth_camera_publisher.publish(this->depth_img, this->depth_info);
    }
  }
  
  void fillCameraInfo(sensor_msgs::CameraInfo & info, sensor_msgs::Image & img)
  {
    info.header.stamp    = img.header.stamp;
    info.header.frame_id = img.header.frame_id;
    info.height          = img.height;
    info.width           = img.width;
    info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    
    double fx,fy,cx,cy,k1,k2,k3,p1,p2=0.0;
    //TODO: load parameters from calibration file?
    if(img.width==1920) //1920*1080
    {
      fx=1066;
      fy=1068;
      cx = (img.width-1)/2.0;
      cy = (img.height-1)/2.0;
      k1,k2,k3,p1,p2=0.0;
    }
    else if (img.width=640) //640*480
    {
      fx=365;
      fy=365;
      cx= (img.width-1)/2.0;
      cy= (img.height-1)/2.0;
      k1,k2,k3,p1,p2=0.0;
    }

    info.D.resize(5);
    info.D[0] = k1;
    info.D[1] = k2;
    info.D[2] = k3;
    info.D[3] = p1;
    info.D[4] = p2;
    
    info.K.fill(0.0);
    info.K[0] = fx;
    info.K[2] = cx;
    info.K[4] = fy;
    info.K[5] = cy;
    info.K[8] = 1.0;

    info.R.fill(0.0);
    info.R[0] = 1.0;
    info.R[4] = 1.0;
    info.R[8] = 1.0;

    info.P.fill(0.0);
    info.P[0] = fx;
    info.P[2] = cx;
    info.P[5] = fy;
    info.P[6] = cy;
    info.P[10] = 1.0; 
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
      tf::Vector3 actualPos = tf::Vector3(-actual_joint.getPosition().x / 1000.0, actual_joint.getPosition().y / 1000.0, actual_joint.getPosition().z / 1000.0);
      tf::Vector3 parentPos = tf::Vector3(-parent_joint.getPosition().x / 1000.0, parent_joint.getPosition().y / 1000.0, parent_joint.getPosition().z / 1000.0);
      
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
        //tf::Vector3 t = transform.getOrigin();
        //t.setY(t.getY()*-1.0);
        //transform.setOrigin(t);
      }
      else
      {
        transform.setOrigin(actualPos);
        transform.setRotation(actualRot);
      }

      std::stringstream actual_frame_id_stream;
      //FIXME actual_frame_id_stream << "/" << "user_" << uid << "/" << actual_joint_name;
      actual_frame_id_stream << "/" << "user_" << "1" << "/" << actual_joint_name;
      std::string actual_frame_id = actual_frame_id_stream.str();

      std::stringstream parent_frame_id_stream;
      //FIXME parent_frame_id_stream << "/" << "user_" << uid << "/" << parent_joint_name;
      parent_frame_id_stream << "/" << "user_" << "1" << "/" << parent_joint_name;
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
    kinect2_tracker::user_IDs ids;
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
        named_joints["neck"]           = (user.getSkeleton().getJoint(nite::JOINT_NECK));  
        named_joints["head"]           = (user.getSkeleton().getJoint(nite::JOINT_HEAD));
        named_joints["left_shoulder"]  = (user.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER));
        named_joints["right_shoulder"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER));
        named_joints["left_elbow"]     = (user.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW));
        named_joints["right_elbow"]    = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW));
        named_joints["left_hand"]      = (user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND));
        named_joints["right_hand"]     = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND));
        named_joints["left_knee"]      = (user.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE));
        named_joints["right_knee"]     = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE));
        named_joints["left_foot"]      = (user.getSkeleton().getJoint(nite::JOINT_LEFT_FOOT));
        named_joints["right_foot"]     = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_FOOT));

        //Publish the joint (name, niteConstruct, ConnectedJoint name, niteConstruct, User)
        publishJointTF("torso",          named_joints["torso"],          skeleton_frame_,  named_joints["torso"], user.getId());
        publishJointTF("right_hip",      named_joints["left_hip"],       "torso",          named_joints["torso"], user.getId());
        publishJointTF("left_hip",       named_joints["right_hip"],      "torso",          named_joints["torso"], user.getId());
        publishJointTF("neck",           named_joints["neck"],           "torso",          named_joints["torso"], user.getId());
        publishJointTF("head",           named_joints["head"],           "neck",           named_joints["neck"], user.getId());
        publishJointTF("right_shoulder", named_joints["left_shoulder"],  "neck",           named_joints["neck"], user.getId());
        publishJointTF("left_shoulder",  named_joints["right_shoulder"], "neck",           named_joints["neck"], user.getId());
        publishJointTF("right_elbow",    named_joints["left_elbow"],     "right_shoulder",  named_joints["left_shoulder"], user.getId());
        publishJointTF("left_elbow",     named_joints["right_elbow"],    "left_shoulder", named_joints["right_shoulder"], user.getId());
        publishJointTF("right_hand",     named_joints["left_hand"],      "right_elbow",     named_joints["left_elbow"], user.getId());
        publishJointTF("left_hand",      named_joints["right_hand"],     "left_elbow",    named_joints["right_elbow"], user.getId());
        /**/
        publishJointTF("right_knee",     named_joints["left_knee"],      "right_hip",       named_joints["left_hip"], user.getId());
        publishJointTF("left_knee",      named_joints["right_knee"],     "left_hip",      named_joints["right_hip"], user.getId());
        publishJointTF("right_foot",     named_joints["left_foot"],      "right_knee",      named_joints["left_knee"], user.getId());
        publishJointTF("left_foot",      named_joints["right_foot"],     "left_knee",     named_joints["right_knee"], user.getId());
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

  image_transport::CameraPublisher depth_camera_publisher;
  sensor_msgs::Image depth_img;
  sensor_msgs::CameraInfo depth_info;
  
  image_transport::CameraPublisher color_camera_publisher;
  sensor_msgs::Image color_img;
  sensor_msgs::CameraInfo color_info;

  //ros::Publisher image_out_publisher_;
  //sensor_msgs::Image image_out_Image_msg_;

  std::string tf_prefix_, camera_frame_, skeleton_frame_;

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
