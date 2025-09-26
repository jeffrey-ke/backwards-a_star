#include <algorithm>
#include <cassert>
#include <limits>
#include <vector>
#include <unordered_set>

#include <boost/functional/hash.hpp>

#include "../include/map.h"
#include "../include/planner.h"
#include "../include/openlist.h"
#include "../include/state.h"
#include "../include/heuristic.h"
#include "../include/helper.h"

#define NUMOFDIRS 8
using std::unordered_set;
using std::vector;

vector<State> backtrack(const State& end, const State& start, Map& map) {
	vector<State> soln;
	soln.push_back(end);

	auto cur = end;
	while (cur != start) {
		const vector<Map::StateGvalPair>& preds = map.backwards(cur); 
		auto min_gval = Map::BIG_GVAL;
		State next_state;
		for (const auto& [pred, gval] : preds) {
			if (
				(gval < min_gval) 
			){
				min_gval = gval;
				next_state = pred;
			}
		}
		soln.push_back(next_state); //copies anyway, don't need to make next_state a copy.
		cur = next_state;
	}
	return soln;
}
void expand_state(OpenList& open, const State& state, unordered_set<State, State::Hasher>& closed, Map& map, Heuristic& heuristic) {
	if (state == Map::IMAGINARY_GOAL) {
		return;
	}
	vector<Map::StateGvalPair> sucs_costs = map[state];
	for (const auto& [s, gval] : sucs_costs) {
		if (closed.find(s) == closed.end()) {
			open.insert_update(s, gval, heuristic);
		}
	}
	closed.insert(state);
}
static vector<State> soln;
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
	if (soln.size() > 0)  {
		const auto& [x, y, t] = soln[curr_time - 1];
		action_ptr[0] = x;
		action_ptr[1] = y;
		return;
	}
	assert(curr_time == 0);
	OpenList open;
	unordered_set<State, State::Hasher> closed;

	const State start{robotposeX, robotposeY, curr_time};
	const vector<State> concrete_goals = helper::parse_goals(target_traj, target_steps, targetposeX, targetposeY, curr_time);
	Map map{raw_map, x_size, y_size, collision_thresh, concrete_goals};
	map.set_start(start);
	OctileHeuristic heuristic(concrete_goals, 20000.0, 600);
	open.insert_update(start, 0.0, heuristic);
	State expanded{};
	do {
		expanded = open.pop();
		expand_state(open, expanded, closed, map, heuristic);
	}
	while (expanded != Map::IMAGINARY_GOAL);
	soln = backtrack(expanded, start, map);
	std::reverse(soln.begin(), soln.end());
	const auto& [next_x, next_y, t] = soln[curr_time + 1];
	action_ptr[0] = next_x;
	action_ptr[1] = next_y;
	return;
}
