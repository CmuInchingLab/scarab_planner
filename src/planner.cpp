#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <scarab_gazebo/PointArr.h>
#include "scarab_gazebo/control.h"
#include "std_msgs/Bool.h"
#include "noros_planner.hpp"
#include <tf2/LinearMath/Quaternion.h>

using namespace std;


class sendControlInput
{

public:
  geometry_msgs::Point currpos;
  bool conBool=0;
  void getPos(const gazebo_msgs::ModelStates& msg){
    //subscribe to the model states and check x y location 
    currpos = msg.pose[1].position;

  }

  void getControlBool(const std_msgs::Bool& msg){
    //subscribe to the model states and check x y location 
    conBool = msg.data;

  }

  double checkDistance(geometry_msgs::Point goal){ //check distance between goal and current
    
    double dist = sqrt(pow(currpos.x-goal.x,2.0)+pow(currpos.y-goal.y,2.0));
    return dist;

  }

};



int main(int argc, char **argv){

  ros::init(argc, argv, "scarab_planner");
  ros::NodeHandle n;

  //this is the a-star search from ricky's node


  KDTree* tree = new KDTree("/home/divyak/Documents/Fall2018/Planning/Project/catkin_ws/src/scarab_planner/victoria_crater.xyz"); 
  LatticeMotion* motion_handler = new LatticeMotion({1.0, 2.0, -2.0, -1.0}, 1.0,tree); 


  State* start = new State(0,0,0, tree);
  State* goalfinal = new State(20, 0,0, tree); 


  a_star_search* planner = new a_star_search();
  vector<tuple<State*,Action*,Info*>> path;
  planner->get_plan(start,goalfinal,path,tree,motion_handler);
  int goalindex = 0 ;

  cout<<"Printing Plan"<<"\n";

  for(auto element:path)
  {
    cout<<get<0>(element)<<get<1>(element)<<(get<2>(element))<<"\n";
    
    cout << "Z: from State: " << (get<0>(element))->z<< '\n';
    cout << "Query tree now: " << tree->query(get<0>(element)->x, get<0>(element)->y) <<'\n';
    cout << "Cost: " <<  get<2>(element)->transition_cost <<'\n';
  }

  //vector<State> plan;
  //need to get plan maybe make a vector or whatever then give me?
  //but I'll make up a message now 
  sendControlInput c;
  int commandnow = 3; // the straight case is default

  //publish to the visualization topic
  ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 10 );

  //publisher for the controller node 
  //ros::Publisher command = n.advertise<scarab_gazebo::control>("commands", 100);

  //subsriber to get current location of robot from gazebo
  //ros::Subscriber Location = n.subscribe("/gazebo/model_states", 10, &sendControlInput::getPos, &c); 
  
  //publisher and subscriber to set/get a control bool parameter that says if controller is executing an action
  ros::Publisher setConBool = n.advertise<std_msgs::Bool>("ControlBool", 100);
  ros::Subscriber getConBool = n.subscribe("/ControlBool", 10, &sendControlInput::getControlBool, &c); 
  

  ros::Rate loop_rate(10);
  ros::Duration(1.5).sleep();

  while(ros::ok()){

    //scarab_gazebo::control msg;
    visualization_msgs::Marker marker;
    // markers for visualization initialization
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.ns = "scarab";
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 1;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    tf2::Quaternion quaternion_for_marker;

    if(goalindex < path.size()){
      quaternion_for_marker.setRPY( 0, 0, get<0>(path[goalindex])->theta);
      marker.id = goalindex;
      marker.pose.position.x = get<0>(path[goalindex])->x;
      marker.pose.position.y = get<0>(path[goalindex])->y;
      marker.pose.position.z =tree->query(get<0>(path[goalindex])->x, get<0>(path[goalindex])->y);
      marker.pose.orientation.x = quaternion_for_marker[0];
      marker.pose.orientation.y = quaternion_for_marker[1];
      marker.pose.orientation.z = quaternion_for_marker[2];
      marker.pose.orientation.w = quaternion_for_marker[3];
      marker.color.r = 0.0; //TODO Change this to be real r value based on cost
      marker.color.g = 0.0; //TODO Change this to be real g value based on cost
      marker.color.b = 0.0; //TODO Change this to be real b value based on cost
      goalindex++;
      vis_pub.publish(marker);
    }


    // OLD code used with controller
    /*//if the controller is not executing a task give it the next point in the plan 
    if(c.conBool == 0){
      //get the next part of the plan
      std_msgs::Bool msgB;
      msgB.data = 1;
      setConBool.publish(msgB);

     commandnow = (get<1>(path[goalindex]))->motion_index;
     goalindex++;
    }


    //this is the info sent to the control node. 
    msg.turn = commandnow;
    msg.inching = (get<2>(path[goalindex]))->transition_cost; //this will need some multiplier
    command.publish(msg);



    cout << "GOAL INDEX" << goalindex <<'\n';*/

    //general ROS Stuff
    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}