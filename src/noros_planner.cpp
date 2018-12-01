#pragma once

#include <iostream>
#include <fstream>
#include <regex>
#include <unordered_set>
#include <set>
#include <vector>
#include <list>
#include <unordered_map>
#include <algorithm>
#include <stdexcept>
#include <queue>
#include <utility>
#include <tuple>
#include <string>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>


// ROS stuff
#include "ros/ros.h"
#include "std_msgs/String.h"

// Eigen stuff
#include "Eigen/Dense"

#define MAX_COST 1e+15;
using namespace std;
using namespace Eigen;

struct State
{
	double x;
	double y;
	double theta;
	State(double a,double b,double c):x(a), y(b), theta(c){}
	
	friend ostream& operator<<(ostream& os, const State* s)
	{
		os << s->toString() << " ";
		return os;
	}
	
	string toString() const
	{
		return (to_string(this->x) + " " + to_string(this->y) + " " + to_string(this->theta));
	}
	
	bool operator==(const State* rhs) const
	{ 
		double x_tol = 1; double y_tol = 1; double theta_tol = 1e-3;
		return ((abs(this->x - rhs->x) < x_tol) && 
				(abs(this->y - rhs->y) < y_tol) && 
				(abs(this->theta - rhs->theta) < theta_tol));		
	}

};

// struct Action2
// {
// 	// Define your action Struct Here
// 	char dir;
// 	Action2(char d):dir(d){}
// 	friend ostream& operator<<(ostream& os, const Action* a)
// 	{
// 		os << a->dir << " ";
// 		return os;
// 	}
// };
struct Action
{
	// Define your action Struct Here
	uint8_t motion_index;
	Action(uint8_t index):motion_index(index){}
	friend ostream& operator<<(ostream& os, const Action* a)
	{
		os << to_string(a->motion_index) << " ";
		return os;
	}
};

// struct Info2
// {
// 	// Define your Info struct Here
// 	string info;
// 	Info2(){
// 		this->info = "NO INFO MOFO";
// 	}
// };

struct Info
{
	// Define your Info struct Here
	double turn radius;
	double arc_length;
	double transition_cost;
	double diff_height;
	Info(double t_r = 0.0,double a_l = 0.0, double t_c =0.0, double d_h = 0.0){
		this->turn_radius = t_r;
		this->arc_length = a_l;
		this->transition_cost = t_c;
		this->diff_height =d_h;
	}

};
class LatticeMotion 
{
public:
	LatticeMotion(const vector<double>& turn_radius, double arc_length);
	~LatticeMotion();

	// some math
	pose get_after_motion_pose(double radius);
	pose to_global_frame(const State* global_state, const State* relative_state);

	// getting the global successors
	bool get_global_successors(const State* global_state,
							 vector<tuple<State*,Action*,Info*>> global_successors);

	// no need for edit functions, just create another LatticeMotion object
	// get functions
	vector<double> get_radius() { return this->turn_radius_; }
	int get_n_branches() { return this->turn_radius_.size() + 1; }
	double get_arc_length() { return this->arc_length_; }

private:
	vector<double> turn_radius_;
	double arc_length_;

	// populated in constructor
	vector<tuple<State*,Action*,Info*>> relative_motion_primitives_;
};

LatticeMotion::LatticeMotion(const vector<double>& turn_radius,
                             double arc_length)
    : turn_radius_(turn_radius), arc_length_(arc_length) 
{
	relative_motion_primitives_.clear();

	uint8_t mid_motion_index = ((this->get_n_branches() - 1) / 2) + 1;
	uint8_t curr_motion_index = 1;
	for (double radius : turn_radius) 
	{
	State* relative_p = this->get_after_motion_pose(radius);
	Action* curr_action = new Action(curr_motion_index);
	Info* curr_info = new Info(radius,this->arc_length_);
	tuple<State*,Action*,Info*> relative_motion_mp = make_tuple(relative_p,curr_action,curr_info);

	// motion_primitive relative_motion_mp = {.final_pose = relative_p,
	//                                        .turn_radius = radius,
	//                                        .arc_length = this->arc_length_,
	//                                        .motion_index = curr_motion_index};
	relative_motion_primitives_.push_back(relative_motion_mp);

	// update curr_motion_index
	curr_motion_index++;
	// if middle trajectory, add the middle motion
	if (curr_motion_index == mid_motion_index) 
	{
		State* relative_mid = new State(this->arc_length_,0,0);
		Action* curr_action = new Action(curr_motion_index);
		Info* curr_info = new Info(DBL_MAX,this->arc_length_);
		tuple<State*,Action*,Info*> relative_motion_mp = make_tuple(relative_mid, curr_action,curr_info);
		relative_motion_primitives_.push_back(relative_motion_mp);
		// go to the next branch
		curr_motion_index++;
	}
  }

  // cout << "LatticeMotion created." << endl;
  // cout << "Relative motion primitives: " << endl;
  // for (motion_primitive mp : this->relative_motion_primitives_)
  //   cout << (int)mp.motion_index << ": r = " << mp.turn_radius
  //        << "  |  S = " << mp.arc_length
  //        << "  |  Pose(x, y, theta) = " << mp.final_pose.x << " "
  //        << mp.final_pose.y << " " << mp.final_pose.theta << endl;
  // cout << "---------------------------------------------------" << endl << endl;
}

LatticeMotion::~LatticeMotion() { cout << "LatticeMotion killed." << endl; }

State* LatticeMotion::get_after_motion_pose(double radius) {
  double relative_x = radius * sin(this->arc_length_ / radius);
  double relative_y = radius - radius * cos(this->arc_length_ / radius);
  double relative_theta = this->arc_length_ / radius;
  State* after_motion_pose = new State(relative_x, relative_y, theta);
  return after_motion_pose;
}

State* LatticeMotion::to_global_frame(const State* global_pose,
                                    const State* relative_pose) {
  MatrixXd T(3, 3);
  T << cos(global_pose->theta), -sin(global_pose->theta), global_pose->x,
      sin(global_pose->theta), cos(global_pose->theta), global_pose->y, 0, 0, 1;
  Vector3d v(relative_pose->x, relative_pose->y, 1);

  Vector3d v_next = T * v;

  State* next_pose = new State(v_next(0),v_next(1),global_pose->theta + relative_pose->theta);
  return next_pose;
}

bool LatticeMotion::get_global_successors(
    const State* global_pose, vector<tuple<State*,Action*,Info*>> global_successors) {
  global_successors.clear();

  // get the relative motion primitives in global frame
  for (motion_primitive mp : relative_motion_primitives_) {
  	State* global_p = this->to_global_frame(global_pose,get<0>(mp));
    global_successors.push_back(make_tuple(global_p,get<1>(mp),get<2>(mp) ) );
  }
  return true;
}

// struct StateHasher
// {
// 	std::size_t operator()(const State* n) const
// 	{

// 		hash<string> hasher;
// 		std::size_t seed = 3;
// 		double x = n->x; double y = n->y; double theta = n->theta;
// 		seed ^= hasher(to_string(n->x))     + 0x9e3779b9 + (seed << 6) + (seed >> 2);
// 		seed ^= hasher(to_string(n->y))     + 0x9e3779b9 + (seed << 6) + (seed >> 2);
// 		seed ^= hasher(to_string(n->theta)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
// 		return seed;
// 	}
// };
// struct StateComparator
// {
// 	bool operator()(const State* lhs, const State* rhs) const
// 	{
// 		return (*lhs == rhs);
// 	}
// };

// // template<typename State,typename Action,typename Info>
// class a_star_search{
// public:
// 	unordered_map<State*,tuple<State*,Action*,Info*>> came_from;
// 	unordered_set<State*,StateHasher,StateComparator> visited;
// 	double get_cost(State* current, State* next);
// 	double get_heuristic(State* current,State* goal);
// 	bool get_successors(State* current, vector<tuple<State*,Action*,Info*>>& successors);
// 	bool get_plan(State* start,State* goal, vector<tuple<State*,Action*,Info*>>& path);
// };

// double a_star_search::get_cost(State* current, State* next)
// {
// 	// Call your custom cost function here 
// 	return abs(current->x-next->x) + abs(current->y - next->y);
// }
// double a_star_search::get_heuristic(State* current,State* goal)
// {
// 	// Call your custom heuristic function here
// 	return abs(current->x-goal->x) + abs(current->y - goal->y);
// }
// bool a_star_search::get_successors(State* current, vector<tuple<State*,Action*,Info*>>& successors)
// {	
// 	// Call your custom get successors here
// 	double M = 5;
// 	Info* I = new Info();

// 	vector<vector<double>> dirs = {{-1,0},{0,1},{1,0},{0,-1}};
// 	vector<char> actions = {'N','E','S','W'};
// 	for(int i=0;i<4;++i){
// 		Action* A = new Action(actions[i]);
// 		// cout<<dirs[i][0];
// 		// cout<<min(max(current->x+dirs[i][0],0.0),M-1);
// 		// cout<<min(max(current->y+dirs[i][1],0.0),M-1);
// 		State* S = new State(min(max(current->x+dirs[i][0],0.0),M-1),min(max(current->y+dirs[i][1],0.0),M-1),0.0);

// 		if(*current == S)
// 		{
// 			continue;
// 		}
// 		else
// 		{
// 			successors.push_back(make_tuple(S,A,I));
// 		}
// 	}
// 	if(successors.size()){return true;}
// 	return false;
// }

// bool a_star_search::get_plan(State* start,State* goal, vector<tuple<State*,Action*,Info*>>& path)
// {
// 	// Most Generic A Star Planner Ever
// 	unordered_map<State*,double, StateHasher, StateComparator> cost_so_far;
// 	// priority_queue<pair<double, State*>, vector< pair<double,State*> >, greater< pair<double,State*> > > f_state;
// 	// priority_queue<pair<double, State*>, vector<double>, greater<double>> f_state;

// 	priority_queue<pair<double, State*>> f_state;

// 	f_state.push({get_heuristic(start,goal), start});
// 	came_from[start] = make_tuple(nullptr,nullptr,nullptr);
// 	cost_so_far[start] = 0;
// 	unordered_set<State*,StateHasher,StateComparator> visited;

// 	while (!f_state.empty())
// 	{
// 		pair<double,State*> node = f_state.top(); f_state.pop();
// 		State* current = node.second;

// 		if (visited.find(current)!=visited.end())
// 		{
// 			continue;
// 		}
// 		// cout<<"Insert State into Visited = "<<current<<"\n";
// 		// cout<<"Current Priority Queue Size = "<<f_state.size()<<"\n";
// 		visited.insert(current);
// 		if (*current == goal) 
// 		{
// 			while(get<0>(came_from[current])!=nullptr)
// 			{
// 				tuple<State*,Action*,Info*> step_tuple = came_from[current];
// 				path.push_back(step_tuple);
// 				current = get<0>(step_tuple);
// 			}
// 			reverse(path.begin(),path.end());
// 			cout<<"GOAL REACHED!!!"<<"\n";
// 			cout<<"Number of Expanded States = "<<visited.size()<<"\n";
// 			return true;
// 		}
// 		vector<tuple<State*,Action*,Info*>> successors;
// 		if(get_successors(current,successors)){
// 			for (auto next : successors) 
// 			{
// 				State* next_state = get<0>(next);
// 				Action* next_action = get<1>(next);
// 				Info* next_info = get<2>(next);

// 				if(visited.find(next_state)!=visited.end())
// 				{
// 					continue;
// 				}
// 				if(cost_so_far.find(current) == cost_so_far.end())
// 				{
// 					cost_so_far[current] = MAX_COST;
// 				}
// 				double new_cost = cost_so_far[current] + get_cost(current, next_state);
// 				if (cost_so_far.find(next_state) == cost_so_far.end() || new_cost < cost_so_far[next_state]) 
// 				{
// 					cost_so_far[next_state] = new_cost;
// 					double f_cost = new_cost + get_heuristic(next_state, goal);
// 					f_state.push({-f_cost,next_state});
// 					came_from[next_state] = make_tuple(current,next_action,next_info);
// 				}
// 			}
// 		}
// 	}
// 	cout<<"Planning Failed; Please Try Again in a Few Moments"<<"\n";
// 	return false;
// }


// int main(){

// 	// This is just a test for the Most Generic A Star Planner Ever on a 5X5 Grid

// 	State* start = new State(0,0,0);
// 	State* goal = new State(4,4,0);		

// 	a_star_search* planner = new a_star_search();
// 	vector<tuple<State*,Action*,Info*>> path;
// 	planner->get_plan(start,goal,path);
// 	cout<<"Printing Plan"<<"\n";
// 	for(auto element:path)
// 	{
// 		cout<<get<0>(element)<<get<1>(element)<<"\n";
// 	}	
// }