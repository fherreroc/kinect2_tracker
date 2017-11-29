/*******************************************************************************
*                                                                              *
*   PrimeSense NiTE 2.0 - User Viewer Sample                                   *
*   Copyright (C) 2012 PrimeSense Ltd.                                         *
*                                                                              *
*******************************************************************************/

#include "../include/kinect2_tracker.hpp"

int main(int argc, char** argv)
{
//initiate ros skeleton_tracker
  ros::init(argc, argv, "skeleton_tracker");

  kinect2_tracker_node* skeleton_tracker = new kinect2_tracker_node();

  while (ros::ok())
  {
    skeleton_tracker->spinner();
  }

  delete skeleton_tracker;

}
