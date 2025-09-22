#ifndef map_h
#define map_h
#include <algorithm>

#include "planner.h"
#include "state.h"
using std::vector;
struct Map {
	static const vector<State> ACTIONS;
	static const State IMAGINARY_GOAL;

	using AdjList = vector<StateCost>;

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
	const AdjList operator[] (const State& cur) const {
		AdjList legal_states;
		int cur_cost = _raw[GETMAPINDEX(cur.x, cur.y, _x_size, _y_size)];

		for (const auto& action : ACTIONS) {
			State next_state = cur + action;
			if (is_valid(next_state)) {
				auto index = GETMAPINDEX(next_state.x, next_state.y, _x_size, _y_size);
				auto cost = static_cast<int>(_raw[index]);
				legal_states.push_back({next_state, cur_cost + cost});
			}
		}
		if (std::find(_concrete_goals.begin(), _concrete_goals.end(), cur) != _concrete_goals.end()) {
			legal_states.push_back(
				{IMAGINARY_GOAL, 0}
			);
		}
		return legal_states;
	};
};
const State Map::IMAGINARY_GOAL{-1, -1, -1};
const vector<State> Map::ACTIONS = {
		State{1, 1, 1},
		State{1, -1, 1},
		State{-1, 1, 1},
		State{-1, -1, 1},
		State{0, 0, 1},
};
#endif
