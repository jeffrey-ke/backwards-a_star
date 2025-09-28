#include <cassert>
#include <vector>
#include <unordered_set>
#include <stack>

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
using std::stack;

stack<State> backtrack(const State& end, const State& start, Map& map) {
	stack<State> soln;

	auto cur = end;
	while (cur != start) {
		const vector<Map::StateGvalPair>& preds = map.predecessors(cur); 
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
		soln.push(next_state); //copies anyway, don't need to make next_state a copy.
		cur = next_state;
	}
	return soln;
}

void expand_state(OpenList& open, const State& state, unordered_set<State, State::Hasher>& closed, Map& map, Heuristic& heuristic) {
	const auto& sucs_costs = map.successors(state);
	for (const auto& [s, gval] : sucs_costs) {
		if (closed.find(s) == closed.end()) {
			open.insert_update(s, gval, heuristic);
		}
	}
	closed.insert(state);
}
static stack<State> soln;
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
	if (soln.size() > 0)  {
		const auto& [x, y, t] = soln.top();
		soln.pop();
		action_ptr[0] = x;
		action_ptr[1] = y;
		return;
	}
	assert(curr_time == 0);
	OpenList open;
	unordered_set<State, State::Hasher> closed;

	auto A_STAR_MODE{MODE::BACKWARD};

	const State robot_init{robotposeX, robotposeY, curr_time};
	const vector<State> concrete_goals = helper::parse_goals(target_traj, target_steps, targetposeX, targetposeY, curr_time);

	State goal{robot_init};
	State start{Map::IMAGINARY_GOAL};

	Map map{raw_map, x_size, y_size, collision_thresh, concrete_goals, start, A_STAR_MODE};
	OctileHeuristic octile({goal}, 9000000.0, 9000, A_STAR_MODE);
	WallHeuristic wall(map, 8000);
	WeightedCombinationHeuristic heuristic(octile, octile, wall, 0, 1, 1);
	open.insert_update(Map::IMAGINARY_GOAL, 0.0, heuristic);

	State expanded{};
	while (true) {
		expanded = open.pop();
		if (expanded == goal) {
			break;
		}
		expand_state(open, expanded, closed, map, heuristic);
	}
	soln = backtrack(expanded, start, map);
	const auto& [next_x, next_y, t] = soln.top();
	soln.pop();
	action_ptr[0] = next_x;
	action_ptr[1] = next_y;
	return;
}
