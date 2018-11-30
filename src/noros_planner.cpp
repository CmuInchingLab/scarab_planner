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

#define MAX_COST 1e+15;
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

// template<typename State,typename Action,typename Info>
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
	return abs(current->x-next->x) + abs(current->y - next->y);
}
double a_star_search::get_heuristic(State* current,State* goal)
{
	return abs(current->x-goal->x) + abs(current->y - goal->y);
}
bool a_star_search::get_successors(State* current, vector<tuple<State*,Action*,Info*>>& successors)
{	
	double M = 5;
	Info* I = new Info();

	vector<vector<double>> dirs = {{-1,0},{0,1},{1,0},{0,-1}};
	vector<char> actions = {'N','E','S','W'};
	for(int i=0;i<4;++i){
		Action* A = new Action(actions[i]);
		// cout<<dirs[i][0];
		// cout<<min(max(current->x+dirs[i][0],0.0),M-1);
		// cout<<min(max(current->y+dirs[i][1],0.0),M-1);
		State* S = new State(min(max(current->x+dirs[i][0],0.0),M-1),min(max(current->y+dirs[i][1],0.0),M-1),0.0);

		if(*current == S)
		{
			continue;
		}
		else
		{
			successors.push_back(make_tuple(S,A,I));
		}
	}
	if(successors.size()){return true;}
	return false;
}

bool a_star_search::get_plan(State* start,State* goal, vector<tuple<State*,Action*,Info*>>& path)
{
	unordered_map<State*,double,StateHasher,StateComparator> cost_so_far;
	priority_queue<pair<double, State*>> f_state;
	f_state.push({get_heuristic(start,goal), start});
	came_from[start] = make_tuple(nullptr,nullptr,nullptr);
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
		// cout<<"Insert State into Visited = "<<current<<"\n";
		// cout<<"Current Priority Queue Size = "<<f_state.size()<<"\n";
		visited.insert(current);
		if (*current == goal) 
			{
			while(get<0>(came_from[current])!=nullptr)
			{
				tuple<State*,Action*,Info*> step_tuple = came_from[current];
				path.push_back(step_tuple);
				current = get<0>(step_tuple);
			}
			reverse(path.begin(),path.end());
			cout<<"GOAL REACHED!!!"<<"\n";
			cout<<"Number of Expanded States = "<<visited.size()<<"\n";
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
					f_state.push({-f_cost,next_state});
					came_from[next_state] = make_tuple(current,next_action,next_info);
				}
			}
		}
	}
	cout<<"Planning Failed; Please Try Again in a Few Moments"<<"\n";
	return false;
}

int main(){

	State* start = new State(0,0,0);
	State* goal = new State(4,4,0);		

	a_star_search* planner = new a_star_search();
	vector<tuple<State*,Action*,Info*>> path;
	planner->get_plan(start,goal,path);
	cout<<"Printing Plan"<<"\n";
	for(auto element:path)
	{
		cout<<get<0>(element)<<get<1>(element)<<"\n";
	}

	
}