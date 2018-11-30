#include "lattice_graph.hpp"

using namespace std;
using namespace Eigen;

LatticeMotion::LatticeMotion(const vector<double>& turn_radius,
                             double arc_length)
    : turn_radius_(turn_radius), arc_length_(arc_length) {
  relative_motion_primitives_.clear();

  uint8_t mid_motion_index = ((this->get_n_branches() - 1) / 2) + 1;
  uint8_t curr_motion_index = 1;
  for (double radius : turn_radius) {
    pose relative_p = this->get_after_motion_pose(radius);
    motion_primitive relative_motion_mp = {.final_pose = relative_p,
                                           .turn_radius = radius,
                                           .arc_length = this->arc_length_,
                                           .motion_index = curr_motion_index};
    relative_motion_primitives_.push_back(relative_motion_mp);

    // update curr_motion_index
    curr_motion_index++;
    // if middle trajectory, add the middle motion
    if (curr_motion_index == mid_motion_index) {
      pose relative_mid = {.x = this->arc_length_, .y = 0, .theta = 0};
      motion_primitive relative_motion_mp = {.final_pose = relative_mid,
                                             .turn_radius = DBL_MAX,
                                             .arc_length = this->arc_length_,
                                             .motion_index = curr_motion_index};
      relative_motion_primitives_.push_back(relative_motion_mp);

      // go to the next branch
      curr_motion_index++;
    }
  }

  cout << "LatticeMotion created." << endl;
  cout << "Relative motion primitives: " << endl;
  for (motion_primitive mp : this->relative_motion_primitives_)
    cout << (int)mp.motion_index << ": r = " << mp.turn_radius
         << "  |  S = " << mp.arc_length
         << "  |  Pose(x, y, theta) = " << mp.final_pose.x << " "
         << mp.final_pose.y << " " << mp.final_pose.theta << endl;
  cout << "---------------------------------------------------" << endl << endl;
}

LatticeMotion::~LatticeMotion() { cout << "LatticeMotion killed." << endl; }

pose LatticeMotion::get_after_motion_pose(double radius) {
  double relative_x = radius * sin(this->arc_length_ / radius);
  double relative_y = radius - radius * cos(this->arc_length_ / radius);
  double relative_theta = this->arc_length_ / radius;
  pose after_motion_pose = {
      .x = relative_x, .y = relative_y, .theta = relative_theta};
  return after_motion_pose;
}

pose LatticeMotion::to_global_frame(const pose& global_pose,
                                    const pose& relative_pose) {
  MatrixXd T(3, 3);
  T << cos(global_pose.theta), -sin(global_pose.theta), global_pose.x,
      sin(global_pose.theta), cos(global_pose.theta), global_pose.y, 0, 0, 1;
  Vector3d v(relative_pose.x, relative_pose.y, 1);

  Vector3d v_next = T * v;

  pose next_pose = {.x = v_next(0),
                    .y = v_next(1),
                    .theta = global_pose.theta + relative_pose.theta};
  return next_pose;
}

bool LatticeMotion::get_global_successors(
    const pose& global_pose, vector<motion_primitive>& global_successors) {
  global_successors.clear();

  // get the relative motion primitives in global frame
  for (motion_primitive mp : relative_motion_primitives_) {
    pose global_p = this->to_global_frame(global_pose, mp.final_pose);
    motion_primitive global_mp = {.final_pose = global_p,
                                  .turn_radius = mp.turn_radius,
                                  .arc_length = mp.arc_length,
                                  .motion_index = mp.motion_index};
    global_successors.push_back(global_mp);
  }
  return true;
}