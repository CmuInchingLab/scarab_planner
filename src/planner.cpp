#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>

using namespace std;
// Get current X and Y coordinates of scarab robot. 
// This is for testing whether we can get Z coordinates given X and Y
void pose_cb(const gazebo_msgs::ModelStates& gazebo_msg){ 
    // scarab robot is 2nd element in model position array
    geometry_msgs::Point position = gazebo_msg.pose[1].position;
    cout << "Got callback! X:" << position.x << " Y:" << position.y << endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "main_scarab");
  ros::NodeHandle n;
//   ros::Publisher chatter_pub = n.advertise<std_msgs::String>("cost", 1000);
  ros::Subscriber sub = n.subscribe("/gazebo/model_states", 100, pose_cb);
  ros::Rate loop_rate(10);

  int count = 0;


  while (ros::ok())
  {
    // chatter_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
    // ++count;
  }


  return 0;
}