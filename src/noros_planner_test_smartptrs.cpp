#include "noros_planner_test_smartptrs.hpp"
#define MAX_COST 1e+15;
using namespace std;


double a_star_search::get_cost(shared_ptr<State>& current, shared_ptr<State>& next)
{
	// Call your custom cost function here 
	return abs(current->x-next->x) + abs(current->y - next->y);
}
double a_star_search::get_heuristic(shared_ptr<State>& current,shared_ptr<State>& goal)
{
	// Call your custom heuristic function here
	return abs(current->x-goal->x) + abs(current->y - goal->y);
}
bool a_star_search::get_successors(shared_ptr<State>& current, vector<tuple<shared_ptr<State>,shared_ptr<Action>,shared_ptr<Info>>>& successors)
{	
	// Call your custom get successors here
	double M = 20;
	shared_ptr<Info> I = make_shared<Info>();

	vector<vector<double>> dirs = {{-1,0},{0,1},{1,0},{0,-1}};
	vector<char> actions = {'N','E','S','W'};
	for(int i=0;i<4;++i){
		shared_ptr<Action> A = make_shared<Action>(actions[i]);
		// cout<<dirs[i][0];
		// cout<<min(max(current->x+dirs[i][0],0.0),M-1);
		// cout<<min(max(current->y+dirs[i][1],0.0),M-1);
		shared_ptr<State> S = make_shared<State>(min(max(current->x+dirs[i][0],0.0),M-1),min(max(current->y+dirs[i][1],0.0),M-1),0.0);

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

bool a_star_search::get_plan(shared_ptr<State>& start,shared_ptr<State>& goal, vector<tuple<shared_ptr<State>,shared_ptr<Action>,shared_ptr<Info>>>& path)
{
	// Most Generic A Star Planner Ever
	unordered_map<shared_ptr<State>,double, StateHasher, StateComparator> cost_so_far;
	priority_queue<pair<double, shared_ptr<State>>> f_state;

	f_state.push({get_heuristic(start,goal), start});
	came_from[start] = make_tuple(nullptr,nullptr,nullptr);
	cost_so_far[start] = 0;
	unordered_set<shared_ptr<State>,StateHasher,StateComparator> visited;

	while (!f_state.empty())
	{
		pair<double,shared_ptr<State>> node = f_state.top();f_state.pop();
		shared_ptr<State> current = node.second;

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
				tuple<shared_ptr<State>,shared_ptr<Action>,shared_ptr<Info>> step_tuple = came_from[current];
				path.push_back(step_tuple);
				current = get<0>(step_tuple);
			}
			reverse(path.begin(),path.end());
			cout<<"GOAL REACHED!!!"<<"\n";
			cout<<"Number of Expanded States = "<<visited.size()<<"\n";
			return true;
		}
		vector<tuple<shared_ptr<State>,shared_ptr<Action>,shared_ptr<Info>>> successors;
		if(get_successors(current,successors)){
			for (auto next : successors) 
			{
				shared_ptr<State> next_state = get<0>(next);
				shared_ptr<Action> next_action = get<1>(next);
				shared_ptr<Info> next_info = get<2>(next);

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

	// This is just a test for the Most Generic A Star Planner Ever on a 5X5 Grid

	shared_ptr<State> start = make_shared<State>(0,0,0);
	shared_ptr<State> goal = make_shared<State>(19,19,0);		

	a_star_search* planner = new a_star_search();
	vector<tuple<shared_ptr<State>,shared_ptr<Action>,shared_ptr<Info>>> path;
	planner->get_plan(start,goal,path);
	cout<<"Printing Plan"<<"\n";
	for(auto element:path)
	{
		cout<<get<0>(element)<<get<1>(element)<<"\n";
	}	
}