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

#define MAX_COST 1e+15;
using namespace std;

struct State
{
	double x,double y,double theta;
	State(double a,double b,double c):x(a), y(b), theta(c){}
};

struct Action
{
	char dir;
	Action(char d):dir(d){}
};

struct Info
{
	char info:
	Info():info("No Info MOFO"){}
};

struct StateHasher
{
	std::size_t operator()(const State* n) const 
	{
		hash<double> hasher;
		std::size_t seed = 3;
		seed ^= hasher((n->x).toString())     + 0x9e3779b9 + (seed << 6) + (seed >> 2);
		seed ^= hasher((n->y).toString())     + 0x9e3779b9 + (seed << 6) + (seed >> 2);
		seed ^= hasher((n->theta).toString()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
		return seed;
	}
};
struct StateComparator
{
	bool operator()(const State* lhs, const State* rhs) const
	{
		double x_tol = 1; double y_tol = 1; double theta_tol = 1e-3;
		return ((abs(lhs->x - rhs->x) < x_tol) && 
				(abs(lhs->y - rhs->y) < y_tol) && 
				(abs(lhs->theta - rhs->theta) < theta_tol));
	}
};

template<typename State,typename Action,typename Info>
class a_star_search{
public:
	unordered_map<State*,tuple<State*,Action*,Info*>> came_from;
	unordered_set<State*,StateHasher,StateComparator> visited;
	double get_cost(State* current, State* next);
	double get_heuristic(State* current,State* goal);
	bool get_successors(State* current, vector<tuple<State*,Action*,Info*>>& successors);
	bool get_plan(State* start,State* goal, vector<tuple<State*,Action*,Info*>>& path);
};
double a_star_search::get_cost(State* current, State* next)
{
	return (current->x-next->x + current->y - next->y);
}
double a_star_search::get_heuristic(State* current,State* goal)
{
	return 0;
}
bool a_star_search::get_successors(State* current, vector<tuple<State*,Action*,Info*>>& successors)
{
	State* a = new State(0,0,0);
	State* b = new State(0,1,0);
	State* c = new State(1,0,0);
	State* d = new State(1,1,0);
	
	Info* i = new Info();

	if(current == a)
	{
		Action* a1 = new Action('E');
		State* s1 = new State(0,1,0);
		successors.push_back(make_tuple(s1,a1,i));

		Action* a2 = new Action('S');
		State* s2 = new State(1,0,0);
		successors.push_back(make_tuple(s2,a2,i));

		return true;

	}
	else if(current == b)
	{
		Action* a1 = new Action('W');
		State* s1 = new State(0,0,0);
		successors.push_back(make_tuple(s1,a1,i));

		Action* a2 = new Action('S');
		State* s2 = new State(1,1,0);
		successors.push_back(make_tuple(s2,a2,i));

		return true;

	}
	else if(current == c)
	{
		Action* a1 = new Action('E');
		State* s1 = new State(1,1,0);
		successors.push_back(make_tuple(s1,a1,i));

		Action* a2 = new Action('N');
		State* s2 = new State(0,0,0);
		successors.push_back(make_tuple(s2,a2,i));
		
		return true;
	} 
	else if(current == d)
	{
		Action* a1 = new Action('W');
		State* s1 = new State(0,1,0);
		successors.push_back(make_tuple(s1,a1,i));

		Action* a2 = new Action('N');
		State* s2 = new State(1,0,0);
		successors.push_back(make_tuple(s2,a2,i));

		return true;
	}

	return false;
}

bool a_star_search::get_plan(State* start,State* goal, vector<tuple<State*,Action*,Info*>>& path)
{
	unordered_map<State*,double,StateHasher, StateComparator> cost_so_far;
	priority_queue<pair<double, State*>> f_state;
	f_state.push({get_heuristic(start,goal), start});
	came_from[start] = nullptr;
	cost_so_far[start] = 0;
	unordered_set<State*,StateHasher,StateComparator> visited;

	while (!f_state.empty())
	{
		pair<double,State*> node = f_state.top(); f_state.pop();
		State* current = node.second;

		if (visited.find(current)!=visited.end())
		{
			continue;
		}
		if (current == goal) 
			{

			while(came_from[current]!=nullptr)
			{
				tuple<State*,Action*,Info*> step_tuple = came_from[current];
				path.push_back(step_tuple);
				current = get<0>(step_tuple);
			}
			
			tuple<State*,Action*,Info*> step_tuple = came_from[current];
			path.push_back(step_tuple);

			reverse(path.begin(),path.end());
			cout<<"\n";
			cout<<"GOAL REACHED!!!"<<"\n";
			cout<<"Number of Expanded States = "<<visited.size();
			return true;
		}
		vector<tuple<State*,Action*,Info*>> successors;
		if(get_successors(current,successors)){
			for (auto next : successors) 
			{
				State* next_state = get<0>(next);
				Action* next_action = get<1>(next);
				Info* next_info = get<2>(next);

				if(visited.find(next_state)!=visited.end())
				{
					continue;
				}
				if(cost_so_far.find(current) == cost_so_far.end())
				{
					cost_so_far[current] = MAX_COST;
				}
				double new_cost = cost_so_far[current] + get_cost(current, next_state);
				if (cost_so_far.find(next_state) == cost_so_far.end() || new_cost < cost_so_far[next_state]) 
				{
					cost_so_far[next_state] = new_cost;
					double f_cost = new_cost + get_heuristic(next_state, goal);
					f_state.push({f_cost,next_state});
					came_from[next] = make_tuple(current,next_action,next_info);
				}
			}
		}
		visited.insert(current);
	}
	cout<<"Planning Failed; Please Try Again in a Few Moments";
	return false;
}

int main(){

	State* start = new State(0,0,0);
	State* one = new State(0,1,0);
	State* two = new State(1,0,0);
	State* goal = new State(1,1,0);		
	a_star_search* planner = new a_star_search();
	vector<tuple<State*,Action*,Info*>> path;
	planner->get_plan(start,goal,path);
}