#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Point.h>
#include <scarab_gazebo/PointArr.h>
#include "scarab_gazebo/control.h"
#include "std_msgs/Bool.h"
#include "kd_tree.hpp"

using namespace std;

KDTree kd_tree("/home/andrew/planning_ws/src/scarab_planner/victoria_crater.xyz");

void getPos(const geometry_msgs::Point& pt){
  //subscribe to the model states and check x y location 
  // vector<double>xy_pos{currpos.x, currpos.y};
  double x = pt.x;
  double y = pt.y;
  cout <<  "QUERYING POINT: "<<  x << "," << y << endl;
  vector<double>xy_pos{x, y};
  cout << kd_tree.query(xy_pos) << endl;
}


int main(int argc, char ** argv){

  ros::init(argc, argv, "kd_tree_test");
  ros::NodeHandle n;

  // ros::Publisher command = n.advertise<scarab_gazebo::control>("commands", 100);
  ros::Subscriber Location = n.subscribe("test_kdtree", 10, getPos); 
  
  ros::Rate loop_rate(1000);
  ros::Duration(1).sleep();

  while(ros::ok()){


    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}