#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Point.h>
#include <scarab_gazebo/PointArr.h>

using namespace std;

struct Point{
  float x;
  float y;

  Point(float x, float y){
    this->x = x;
    this->y = y;
  }
};

class Planner{
private:
  ros::NodeHandle* n;
  int counter = 0;
  ros::Publisher request_Z_pub;
  ros::Subscriber receive_Z_sub;
public:
  Planner(ros::NodeHandle* nh){
    n = nh;
    request_Z_pub = n->advertise<scarab_gazebo::PointArr>("/scarab_gazebo/request_z", 10);
    receive_Z_sub = n->subscribe("/scarab_gazebo/response_z", 10, &Planner::receiveZ, this);
  }

  //get Z values from Gazebo node
  void requestZ(vector<Point> points){
    scarab_gazebo::PointArr query_pts;
    query_pts.id = counter;
    for (const auto& p: points){
      geometry_msgs::Point query_p;
      query_p.x = p.x;
      query_p.y = p.y;
      query_pts.points.push_back(query_p);
    }
    request_Z_pub.publish(query_pts);
    counter++;
  }

  // Callback for getting array of Z coordinates given X and Y coordinates
  void receiveZ(const scarab_gazebo::PointArr receivedPts){
      // print out coordinates for debugging
      // if (receivedPts.id == counter-1){
        cout << "POINT(S) RECEIVED!" << endl; 
        for (const auto& rp: receivedPts.points){
          cout << "X:" << rp.x << "  Y:" << rp.y << "  Z:" << rp.z << endl; 
        }
      // }
      // else
      //   ROS_ERROR("ID errror in planner.cpp receiveZ( ): query points and received points don't match!");
        
  }
};
int main(int argc, char **argv)
{
  ros::init(argc, argv, "planner");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  Planner plan(&n);

  Point p1(-1.05,2);  //test points
  Point p2(2,3.14159);
  Point p3(-45.61109,-32.34);
  vector<Point> pts = {p1,p2, p3};

  while (ros::ok())
  {
    plan.requestZ(pts);
    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}