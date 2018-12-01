#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Point.h>
#include <scarab_gazebo/PointArr.h>
#include "scarab_gazebo/control.h"
#include "std_msgs/Bool.h"
#include "noros_planner.hpp"
#include "kd_tree.hpp"

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

  //handles the sending/receiving of messages to get Z value of map given an X and Y
  int counter = 0;
  ros::Publisher request_Z_pub;
  ros::Subscriber receive_Z_sub;
  scarab_gazebo::PointArr received; // received points 

public:
  bool got_msg = true;     //true if we received the last Z value we requested

  Planner(ros::NodeHandle* nh){
    n = nh;
    request_Z_pub = n->advertise<scarab_gazebo::PointArr>("/scarab_gazebo/request_z", 100);
    receive_Z_sub = n->subscribe("/scarab_gazebo/response_z", 100, &Planner::receiveZ, this);
  }

  //get Z values from Gazebo node
  void requestZ(vector<Point> points){
    while(!got_msg){  //wait until we've received previous message before sending this one  
      // cout << "Waiting for message" << endl;
      ros::spinOnce();
    }
    scarab_gazebo::PointArr query_pts;
    query_pts.id = counter;
    for (const auto& p: points){
      geometry_msgs::Point query_p;
      query_p.x = p.x;
      query_p.y = p.y;
      query_pts.points.push_back(query_p);
    }
    request_Z_pub.publish(query_pts);
    got_msg = false;
    // cout << "Requested Z values" << endl;

  }


  //pointers to states,

  double getCost(State* current, State* goal){ //need to send a vector of two points 
    
    Point p1(current->x,current->y);  //test points
    Point p2(goal->x, goal->y);
    vector<Point> pts = {p1,p2};

    requestZ(pts);

    cout << "GOT MSG" << got_msg << '\n';
    while(!got_msg){
      ros::spinOnce();
   }
    double diff = received.points[0].z - received.points[1].z;
    return diff;
  } 

  // Callback for getting array of Z coordinates given X and Y coordinates
  void receiveZ(const scarab_gazebo::PointArr receivedPts){
      // print out coordinates for debugging
      if (receivedPts.id == counter){
        cout << "POINT(S) RECEIVED!" << endl; 
        for (const auto& rp: receivedPts.points){
          cout << "X:" << rp.x << "  Y:" << rp.y << "  Z:" << rp.z << endl; 
        }
        counter++;
        got_msg = true;
        received = receivedPts;
        //return receivedPts;
      }
      else{
        // cout << "EXPECTED ID:" << counter << " BUT GOT:" << receivedPts.id << endl;
        ROS_ERROR("ID errror in planner.cpp receiveZ( ): query points and received points don't match!");
      }

        
  }
};


class sendControlInput
{

public:
  geometry_msgs::Point currpos;
  bool conBool;
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
  Planner plan(&n);


  //this is the a-star search from ricky's node

  State* start = new State(0,0,0);
  State* goalfinal = new State(4,4,0); 

  // a_star_search a;
  a_star_search* planner = new a_star_search();
  vector<tuple<State*,Action*,Info*>> path;
  planner->get_plan(start,goalfinal,path);
  int goalindex = 0 ;

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

    //general ROS Stuff
    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}