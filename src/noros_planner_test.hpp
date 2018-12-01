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
	char dir;
	Action(char d):dir(d){}
	friend ostream& operator<<(ostream& os, const Action* a)
	{
		os << a->dir << " ";
		return os;
	}
};


struct Info
{
	// Define your Info struct Here
	string info;
	Info(){
		this->info = "NO INFO MOFO";
	}
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

class a_star_search{
public:
	unordered_map<State*,tuple<State*,Action*,Info*>> came_from;
	unordered_set<State*,StateHasher,StateComparator> visited;
	double get_cost(State* current, State* next);
	double get_heuristic(State* current,State* goal);
	bool get_successors(State* current, vector<tuple<State*,Action*,Info*>>& successors);
	bool get_plan(State* start,State* goal, vector<tuple<State*,Action*,Info*>>& path);
};
