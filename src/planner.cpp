#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Point.h>
#include <scarab_gazebo/PointArr.h>
#include "scarab_gazebo/control.h"
#include "std_msgs/Bool.h"
#include "noros_planner.hpp"

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

  State* start = new State(0,0,0);
  State* goalfinal = new State(10,0,0); 

  // a_star_search a;
  a_star_search* planner = new a_star_search();
  vector<tuple<State*,Action*,Info*>> path;
  planner->get_plan(start,goalfinal,path);
  int goalindex = 0 ;

  KDTree* tree = new KDTree("/home/divyak/Documents/Fall2018/Planning/Project/catkin_ws/src/scarab_planner/victoria_crater.xyz"); 

  cout<<"Printing Plan"<<"\n";
  for(auto element:path)
  {
    path[0]
    cout<<get<0>(element)<<get<1>(element)<<(get<2>(element))->transition_cost<<"\n";
    
    cout << "Z: from Info: " << (get<2>(element))->curr_z << '\n';
    cout << "Query tree now: " << tree->query(get<0>(element)->x, get<0>(element)->y) <<'\n';
  }

  //vector<State> plan;
  //need to get plan maybe make a vector or whatever then give me?
  //but I'll make up a message now 
  sendControlInput c;
  int commandnow = 3; // the straight case is default

  //publisher for the controller node 
  ros::Publisher command = n.advertise<scarab_gazebo::control>("commands", 100);

  //subsriber to get current location of robot from gazebo
  ros::Subscriber Location = n.subscribe("/gazebo/model_states", 10, &sendControlInput::getPos, &c); 
  
  //publisher and subscriber to set/get a control bool parameter that says if controller is executing an action
  ros::Publisher setConBool = n.advertise<std_msgs::Bool>("ControlBool", 100);
  ros::Subscriber getConBool = n.subscribe("/ControlBool", 10, &sendControlInput::getControlBool, &c); 
  

  ros::Rate loop_rate(10);
  ros::Duration(1.5).sleep();

  while(ros::ok()){

    scarab_gazebo::control msg;


    //if the controller is not executing a task give it the next point in the plan 
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

    cout << "GOAL INDEX" << goalindex <<'\n';

    //general ROS Stuff
    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}