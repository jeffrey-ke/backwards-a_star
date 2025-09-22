#ifndef map_h
#define map_h
#include <algorithm>
#include <limits>

#include "planner.h"
#include "state.h"
using std::vector;
struct Map {
	static const vector<Action> ACTIONS;
	static const State IMAGINARY_GOAL;


	int* _raw;
	int _x_size, _y_size, _thres;
	vector<State> _concrete_goals;

	Map(int* raw, int x_size, int y_size, int thres, const vector<State>& concrete_goals): 
		_raw(raw),
		_x_size(x_size),
		_y_size(y_size),
		_thres(thres),
		_concrete_goals(concrete_goals) 
	{};

	bool is_valid(const State& s) const {
		auto index = GETMAPINDEX(s.x, s.y, _x_size, _y_size);
		return (
			s.x >= 0 and
			s.x < _x_size - 1 and
			s.y >= 0 and
			s.y < _y_size - 1 and
			static_cast<int>(_raw[index]) < _thres
		);
	};
	using Cost = int;
	using StateCostPair = std::pair<State, Cost>;
	const vector<StateCostPair> operator[] (const State& cur) const {
		vector<StateCostPair> legal_states;
		if (cur == IMAGINARY_GOAL) {
			// TODO: this is tricky, because not all
			// the goals are actually traversable predecessors
			// of the imaginary goal; the only traversable 
			// predecessors are the ones that are temporally
			// adjacent.
		}

		for (const auto& action : ACTIONS) {
			State next_state = cur + action;
			if (is_valid(next_state)) {
				auto index = GETMAPINDEX(next_state.x, next_state.y, _x_size, _y_size);
				auto cost = static_cast<int>(_raw[index]);
				next_state.gval = cur.gval + cost;
				legal_states.push_back({next_state, cost});
			}
		}
		// If the expanded state is one of the concrete goals,
		// then add the imaginary goal to the successors 
		// ah, then add as predecessor to the imaginary goal this
		// state
		if (std::find(_concrete_goals.begin(), _concrete_goals.end(), cur) != _concrete_goals.end()) {
			legal_states.push_back(
				{IMAGINARY_GOAL, 0}
			);
		}
		return legal_states;
	};
};
const State Map::IMAGINARY_GOAL{0, 0, 0, std::numeric_limits<int>::max()};
const vector<Action> Map::ACTIONS = {
	Action{-1, -1, 1},
	Action{-1, 0, 1},
	Action{-1, 1, 1},
	Action{0, -1, 1},
	Action{0, 0, 1},
	Action{0, 1, 1},
	Action{1, -1, 1},
	Action{1, 0, 1},
	Action{1, 1, 1},
};
#endif
