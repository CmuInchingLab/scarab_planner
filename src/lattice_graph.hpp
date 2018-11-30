#pragma once

#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

// ROS stuff
#include "ros/ros.h"
#include "std_msgs/String.h"

// Eigen stuff
#include "Eigen/Dense"

using namespace std;

// all motion definitions are right-handed
// using the east-north-up convention as per ROS / Gazebo
// heading is 0 degrees at east, increases in counter clockwise direction
// distance units are in meters
// angle units are in radians

// pose: x, y, theta
// can be used in relative frame or global frame
struct pose {
  double x, y, theta;
};

// motion_prim
struct motion_primitive {
  pose final_pose;
  double turn_radius;
  double arc_length;
  uint8_t motion_index;
};

// class for getting successors
// pass in a vector of turning radius, exclude the forward moving center branch
class LatticeMotion {
 public:
  LatticeMotion(const vector<double>& turn_radius, double arc_length);
  ~LatticeMotion();

  // some math
  pose get_after_motion_pose(double radius);
  pose to_global_frame(const pose& global_pose, const pose& relative_pose);

  // getting the global successors
  bool get_global_successors(const pose& global_pose,
                             vector<motion_primitive>& global_successors);

  // no need for edit functions, just create another LatticeMotion object
  // get functions
  vector<double> get_radius() { return this->turn_radius_; }
  int get_n_branches() { return this->turn_radius_.size() + 1; }
  double get_arc_length() { return this->arc_length_; }

 private:
  vector<double> turn_radius_;
  double arc_length_;

  // populated in constructor
  vector<motion_primitive> relative_motion_primitives_;
};