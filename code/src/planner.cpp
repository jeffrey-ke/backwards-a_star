#include <algorithm>
#include <cassert>
#include <iterator>
#include <vector>
#include <set>

#include <boost/functional/hash.hpp>

#include "../include/map.h"
#include "../include/planner.h"
#include "../include/openlist.h"
#include "../include/state.h"
#include "../include/heuristic.h"
#include "../include/helper.h"

#define NUMOFDIRS 8
using std::set;
using std::vector;

vector<State> backtrack(const State& end, const State& start, const Map& map) {
	vector<State> soln;
	soln.push_back(end);

	auto cur = end;
	while (cur != start) {
		const vector<StateCost> preds = map[cur]; 
		auto& [next_state, cost] = (
			*
			std::min_element(preds.begin(), preds.end(), OpenList::StateLT())
		);
		soln.push_back(next_state);
		cur = next_state;
	}
	return soln;
}
void expand_state(OpenList& open, const State& state, const set<State>& closed, const Map& map, const Heuristic& heuristic) {
	vector<StateCost> sucs_costs = map[state];
	for (const auto& [state, gval] : sucs_costs) {
		if (closed.find(state) == closed.end()) {
			open.insert_update(state, gval, heuristic);
		}
	}
}
vector<State> soln;
void planner(
	int* raw_map,
	int collision_thresh,
	int x_size,
	int y_size,
	int robotposeX,
	int robotposeY,
	int target_steps,
	int* target_traj,
	int targetposeX,
	int targetposeY,
	int curr_time,
	int* action_ptr
) 
{
	/*
     */
	if (curr_time <= soln.size())  {
		const auto& [x, y, t] = soln[curr_time - 1];
		action_ptr[0] = x;
		action_ptr[1] = y;
		return;
	}
	assert(curr_time == 0);
	OpenList open;
	set<State> closed;

	const State start{robotposeX, robotposeY, curr_time};
	const vector<State> concrete_goals = helper::parse_goals(target_traj, target_steps, targetposeX, targetposeY, curr_time); // returns a set of states and a Target object
	// to make this imaginary goal an actual to the states
	// hence the need for an actual map object
	const Map map{raw_map, x_size, y_size, collision_thresh, concrete_goals};
	const DistanceHeuristic heuristic(concrete_goals, map);
	State expanded{};
	do {
		expanded = open.pop();
		expand_state(open, expanded, closed, map, heuristic);
		closed.insert(expanded);
	}
	while (expanded != Map::IMAGINARY_GOAL);
	soln = backtrack(expanded, start, map);
	const auto& [next_x, next_y, t] = soln[curr_time - 1];
	action_ptr[0] = next_x;
	action_ptr[1] = next_y;
	return;
}
