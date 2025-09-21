#include <cmath>
#include <algorithm>
#include <vector>
#include <set>

#include <boost/functional/hash.hpp>

#include "../include/planner.h"
#include "../include/structs.h"
#include "../include/helper.h"

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8
using std::size_t;
using std::set;
using std::vector;
using std::unordered_map;
using gval = size_t;

/*
 * adjacency list representation of the map
 */

struct Solution {
	vector<State> states;
	const State& operator[] (size_t time) const {
		if (time > states.size()) {
			throw std::out_of_range("Time index out of range [" << states.at(0).t << ", " << states.at(states.size() - 1).t << "]");
		}
		return states.at(time);
	}
	void push_back(const State& s) {
		states.push_back(s);
	}
}
Solution backtrack(const State& end, const State& start, const Map& map) {
	const auto& cur = end;
	Solution soln;
	while (cur != start) {
		const auto& preds = map[cur]; // so this needs to return an adj list: a vector of StateCost
		const auto& next_state = std::min_element(preds.begin(), preds.end(), OpenList::StateLT);
		// how ought I to include time here?
		soln.push_back(next_state);
		cur = next_state;
	}
}
void expand_state(const OpenList& open, const State& state, const set<State>& closed, const Map& map, const Heuristic& heuristic) {
	using OpenList::StateCost;
	set<StateCost> sucs_costs = map[state];
	for (const auto& [state, cost] : sucs_costs) {
		if (closed.find(state) == closed.end()) {
			open.insert_update(state, cost, heuristic);
		}
	}

}
std::optional<Solution> soln;
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
	if (soln.has_value()) {
		*action_ptr = soln[curr_time];
		return;
	}
	OpenList open;
	set<State> closed;

	const State start{robotposeX, robotposeY, curr_time};
	const auto& target = helper::parse_goals(target_traj, target_poseX, targetposeY, curr_time); // returns a set of states and a Target object
	// to make this imaginary goal an actual to the states
	// hence the need for an actual map object
	const Map map = helper::init_map(imaginary_goal, raw_map);
	do {
		expanded = open.pop();
		expand_state(open, expanded, closed, map);
		closed.push(expanded)
	}
	while (expanded != imaginary_goal);
	soln = backtrack(expanded, start);
	*action_ptr = soln[curr_time];
}
