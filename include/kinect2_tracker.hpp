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
      it_(nh_)
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
    ROS_INFO("Device opened");

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
   * @param j_name: joint name
   * @param j: the joint
   * @param r: relative joint (joint j connects to)
   * @param uid: user's ID
   */
  void publishJointTF(std::string j_name, nite::SkeletonJoint j, std::string r_name, nite::SkeletonJoint r, int uid) 
  {
    if (j.getPositionConfidence() > 0.0)
    {
      tf::Vector3 currentVec3 = tf::Vector3(j.getPosition().x / 1000.0, j.getPosition().y / 1000.0, j.getPosition().z / 1000.0);
      tf::Transform transform;
        if (j_name != "torso")
        {
            tf::Vector3 rVec3 = tf::Vector3(r.getPosition().x / 1000.0, r.getPosition().y / 1000.0, r.getPosition().z / 1000.0);
            transform.setOrigin(currentVec3 - rVec3);
            transform.setRotation(tf::Quaternion(0,0,0,1));
        }
        else
        {
            transform.setOrigin(currentVec3);
            transform.setRotation(tf::Quaternion(0,0,0,1));
        }
      
        std::stringstream j_frame_id_stream; //stringstream of frame id values
        std::string j_frame_id; // string of the stringstream
        j_frame_id_stream << "/" << tf_prefix_ << "/user_" << uid << "/" << j_name;
        j_frame_id = j_frame_id_stream.str();

        std::stringstream r_frame_id_stream; //stringstream of frame id values
        std::string r_frame_id; // string of the stringstream
        r_frame_id_stream << "/" << tf_prefix_ << "/user_" << uid << "/" << r_name;
        r_frame_id = r_frame_id_stream.str();
        
        if (j_name == "torso")
        {
	        
            tfBroadcast_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), relative_frame_, j_frame_id));
        }
        else
        {
            
            tfBroadcast_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), r_frame_id, j_frame_id));
        }
    }
    return;
  }

  //Publish the calibration tf_frame as the cross product of the shoulder vectors
  // This function publishes the calibration_space opposite the shoulders of the user
   void publishCalibrationOriginTF(nite::SkeletonJoint skelTorso, nite::SkeletonJoint skelRshoulder, nite::SkeletonJoint skelLshoulder, int uid) 
   {
     if (skelTorso.getPositionConfidence() > 0.0)
     {
       tf::Transform calibrationOriginTransform;
       tf::Transform torsoTransform;
 
             tf::Vector3 torsoVec3 = tf::Vector3(skelTorso.getPosition().x / 1000.0, skelTorso.getPosition().y / 1000.0, skelTorso.getPosition().z / 1000.0);
             torsoTransform.setOrigin(torsoVec3);
             torsoTransform.setRotation(tf::Quaternion(0,0,0,1));
 
             tf::Vector3 RshoulderVec3 = tf::Vector3(skelRshoulder.getPosition().x / 1000.0, skelRshoulder.getPosition().y / 1000.0, skelRshoulder.getPosition().z / 1000.0);                 //create a vector for the right shoulder
             RshoulderVec3 = (RshoulderVec3 - torsoVec3); //vector is the difference of the two
 
             tf::Vector3 LshoulderVec3 = tf::Vector3(skelLshoulder.getPosition().x / 1000.0, skelLshoulder.getPosition().y / 1000.0, skelLshoulder.getPosition().z / 1000.0);                 //create a vector for the left shoulder
            LshoulderVec3 = (LshoulderVec3 - torsoVec3);
            tf::Vector3 calibrationOriginVec3 = RshoulderVec3.cross(LshoulderVec3);

		//give the calibration origin some length
//	   calibrationOriginVec3 = calibrationOriginVec3 * 20;
           calibrationOriginTransform.setOrigin(calibrationOriginVec3);// set the x,y,z coordinates of the calibration transform frame

              Eigen::Vector3d eigencalibrationOriginVec3;
              tf::vectorTFToEigen(calibrationOriginVec3, eigencalibrationOriginVec3); //conversion of tf:Vec3 to eigen
              Eigen::Vector3d eigenTorsoVec3;
              tf::vectorTFToEigen(torsoVec3, eigenTorsoVec3);               //conversion of torse tf:vec3 to eigen
              Eigen::Quaterniond eigen_calibrationOrigenQuaternion;
              eigen_calibrationOrigenQuaternion.setFromTwoVectors(eigencalibrationOriginVec3, eigenTorsoVec3);
              tf::Quaternion tf_calibrationOriginQuaternion;
              tf::quaternionEigenToTF(eigen_calibrationOrigenQuaternion, tf_calibrationOriginQuaternion);

            calibrationOriginTransform.setRotation(tf_calibrationOriginQuaternion);		
      
        std::stringstream calibration_frame_id_stream; //stringstream of frame id values
        std::string calibration_frame_id; // string of the stringstream
        calibration_frame_id_stream << "/" << tf_prefix_ << "/user_" << uid << "/calibrationOrigin";
        calibration_frame_id = calibration_frame_id_stream.str();

	std::stringstream r_frame_id_stream; //stringstream of frame id values
        std::string r_frame_id; // string of the stringstream
        r_frame_id_stream << "/" << tf_prefix_ << "/user_" << uid << "/torso";
        r_frame_id = r_frame_id_stream.str();
            
        tfBroadcast_.sendTransform(tf::StampedTransform(calibrationOriginTransform, ros::Time::now(), r_frame_id, calibration_frame_id));
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

        named_joints["torso"] = (user.getSkeleton().getJoint(nite::JOINT_TORSO));// this value is Joint_name, position & orientation & confidence, userid
        named_joints["left_hip"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_HIP));
        named_joints["right_hip"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP));
        named_joints["left_knee"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE));
        named_joints["right_knee"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE));
        named_joints["left_foot"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_FOOT));
        named_joints["right_foot"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_FOOT));
        named_joints["neck"] = (user.getSkeleton().getJoint(nite::JOINT_NECK));  
        named_joints["head"] = (user.getSkeleton().getJoint(nite::JOINT_HEAD));
        named_joints["left_shoulder"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER));
        named_joints["right_shoulder"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER));
        named_joints["left_elbow"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW));
        named_joints["right_elbow"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW));
        named_joints["left_hand"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND));
        named_joints["right_hand"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND));
//Publish the joint (name, niteConstruct, ConnectedJoint name, niteConstruct, User
        publishJointTF("torso", named_joints["torso"], "torso", named_joints["torso"], user.getId());
        publishJointTF("left_hip", named_joints["left_hip"], "torso", named_joints["torso"], user.getId());
        publishJointTF("right_hip", named_joints["right_hip"], "torso", named_joints["torso"], user.getId());
        publishJointTF("left_knee", named_joints["left_knee"], "left_hip", named_joints["left_hip"], user.getId());
        publishJointTF("right_knee", named_joints["right_knee"], "right_hip", named_joints["right_hip"], user.getId());
        publishJointTF("left_foot", named_joints["left_foot"], "left_knee", named_joints["left_knee"], user.getId());
        publishJointTF("right_foot", named_joints["right_foot"], "right_knee", named_joints["right_knee"], user.getId());
        publishJointTF("neck", named_joints["neck"], "torso", named_joints["torso"], user.getId());
        publishJointTF("head", named_joints["head"], "neck", named_joints["neck"], user.getId());
        publishJointTF("left_shoulder", named_joints["left_shoulder"], "torso", named_joints["torso"], user.getId());
        publishJointTF("right_shoulder", named_joints["right_shoulder"], "torso", named_joints["torso"], user.getId());
        publishJointTF("left_elbow", named_joints["left_elbow"], "left_shoulder", named_joints["left_shoulder"], user.getId());
        publishJointTF("right_elbow", named_joints["right_elbow"], "right_shoulder", named_joints["right_shoulder"], user.getId());
        publishJointTF("left_hand", named_joints["left_hand"], "left_elbow", named_joints["left_elbow"], user.getId());
        publishJointTF("right_hand", named_joints["right_hand"], "right_elbow", named_joints["right_elbow"], user.getId());

//publishes the funny normal vector from the users chest
        publishCalibrationOriginTF(named_joints["torso"], named_joints["left_shoulder"], named_joints["right_shoulder"], user.getId());

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
  image_transport::ImageTransport it_;
  std::string tf_prefix_, relative_frame_;

  /// Frame broadcaster
  tf::TransformBroadcaster tfBroadcast_;

  /// The openni device
  openni::Device devDevice_;

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
