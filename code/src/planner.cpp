#include <cassert>
#include <thread>
#include <vector>
#include <unordered_set>
#include <deque>
#include <chrono>

#include <boost/functional/hash.hpp>

#include "../include/map.h"
#include "../include/planner.h"
#include "../include/openlist.h"
#include "../include/state.h"
#include "../include/heuristic.h"
#include "../include/helper.h"

#define NUMOFDIRS 8
using namespace std::chrono;
using namespace std::chrono_literals;
using std::unordered_set;
using std::vector;
using std::deque;

deque<State> backtrack(const State& goal, const State& start, Map& map) {
	deque<State> soln;

	auto cur = goal;
	if (goal != Map::IMAGINARY_GOAL) 
		soln.push_back(goal);

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
		if (next_state != Map::IMAGINARY_GOAL)
			soln.push_back(next_state); //copies anyway, don't need to make next_state a copy.
		cur = next_state;
	}
	soln.pop_front(); // the front is always the goal: for forward, it's the imaginary goal, for backwards, it's the starting position.
	// in either case, this doesn't belong here
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
static deque<State> soln;
State get_next(deque<State>& soln, MODE a_star_mode) {
	auto next_state = (a_star_mode == MODE::BACKWARD) ? soln.front() : soln.back();
	if (a_star_mode == MODE::BACKWARD) {
		soln.pop_front();
	}
	else {
		soln.pop_back();
	}
	return next_state;
}

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
	auto A_STAR_MODE{MODE::BACKWARD};

	if (soln.size() > 0)  {
		const auto& [x, y, t] = get_next(soln, A_STAR_MODE);
		action_ptr[0] = x;
		action_ptr[1] = y;
		return;
	}
	auto time_budget = 40s;
	auto time_start = steady_clock::now();
	OpenList open;
	unordered_set<State, State::Hasher> closed;

	const State robot_init{robotposeX, robotposeY, static_cast<int>(time_budget.count())};
	vector<State> concrete_goals = helper::parse_goals(target_traj, target_steps, targetposeX, targetposeY, curr_time);

	State start{Map::IMAGINARY_GOAL};
	State goal{robot_init};

	DijkstraHeuristic heuristic(raw_map, x_size, y_size, collision_thresh, concrete_goals, goal, 100000, target_steps);
	open.insert_update(start, 0.0, heuristic);

	Map map{raw_map, x_size, y_size, collision_thresh, concrete_goals, start, A_STAR_MODE};
	State expanded{};
	while (true) {
		expanded = open.pop();
		if (expanded == goal) {
			break;
		}
		expand_state(open, expanded, closed, map, heuristic);
	}
	soln = backtrack(expanded, start, map);
	auto time_now = steady_clock::now();
	auto time_left = time_budget - duration_cast<seconds>(time_now - time_start);
	std::this_thread::sleep_until(time_now + time_left);
	const auto& [next_x, next_y, next_t] = get_next(soln, A_STAR_MODE);
	action_ptr[0] = next_x;
	action_ptr[1] = next_y;
	return;
}
