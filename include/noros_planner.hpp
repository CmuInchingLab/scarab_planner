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

// // Eigen stuff
#include "Eigen/Dense"


using namespace std;

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


struct Info
{
	// Define your Info struct Here
	double turn_radius;
	double arc_length;
	double transition_cost;
	Info(double t_r = 0.0,double a_l = 0.0, double t_c =0.0){
		this->turn_radius = t_r;
		this->arc_length = a_l;
		this->transition_cost = t_c;
	}
	// string toString() const
	// {
	// 	return (to_string(this->x) + " " + to_string(this->y) + " " + to_string(this->theta));
	// }

};

class LatticeMotion 
{
public:
	LatticeMotion(const vector<double>& turn_radius, double arc_length);
	~LatticeMotion();

	// some math
	State* get_after_motion_pose(double radius);
	State* to_global_frame(const State* global_state, const State* relative_state);

	// getting the global successors
	bool get_global_successors(const State* global_state,vector<tuple<State*,Action*,Info*>>& global_successors);

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



struct StateHasher
{
	std::size_t operator()(const State* n) const
	{

		hash<string> hasher;
		std::size_t seed = 3;
		double x = n->x; double y = n->y; double theta = n->theta;
		seed ^= hasher(to_string(n->x))     + 0x9e3779b9 + (seed << 6) + (seed >> 2);
		seed ^= hasher(to_string(n->y))     + 0x9e3779b9 + (seed << 6) + (seed >> 2);
		seed ^= hasher(to_string(n->theta)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
		return seed;
	}
};
struct StateComparator
{
	bool operator()(const State* lhs, const State* rhs) const
	{
		return (*lhs == rhs);
	}
};

// template<typename State,typename Action,typename Info>
class a_star_search{
public:
	a_star_search();
  	LatticeMotion* motion_handler = new LatticeMotion({1.0, 2.0, -2.0, -1.0}, 1.0);	
	unordered_map<State*,tuple<State*,Action*,Info*>> came_from;
	unordered_set<State*,StateHasher,StateComparator> visited;
	double get_cost(State* current, State* next);
	double get_heuristic(State* current,State* goal);
	bool get_successors(State* current, vector<tuple<State*,Action*,Info*>>& successors);
	bool get_plan(State* start,State* goal, vector<tuple<State*,Action*,Info*>>& path);
};
