#ifndef map_h
#define map_h
#include <algorithm>
#include <limits>
#include <unordered_map>

#include "planner.h"
#include "state.h"
using std::vector;
using std::unordered_map;
struct Map {
	static const vector<Action> ACTIONS;
	static const State IMAGINARY_GOAL;


	int* _raw;
	int _x_size, _y_size, _thres;
	vector<State> _concrete_goals;
	using gval = double;
	unordered_map<State, gval, State::Hasher> _state_gval_table;

	Map(int* raw, int x_size, int y_size, int thres, const vector<State>& concrete_goals): 
		_raw(raw),
		_x_size(x_size),
		_y_size(y_size),
		_thres(thres),
		_concrete_goals(concrete_goals) 
	{
		for (const auto& goal: _concrete_goals) {
			update_gval(goal, std::numeric_limits<double>::max());
			assert(get_gval(goal) == std::numeric_limits<double>::max());
		}
	};

	void set_start(const State& state) {
		update_gval(state, 0.0);
	};
	void update_gval(const State& state, gval val) {
		auto state_iter = _state_gval_table.find(state);
		if (state_iter == _state_gval_table.end()) {
			_state_gval_table[state] = val;
		}
		else if (const auto& [s, cur_gval] = *state_iter; val < cur_gval) {
			_state_gval_table.at(s) = val;
		}
	};

	gval get_gval(const State& state) const {
		return _state_gval_table.at(state);
	};

	bool is_valid(const State& s) const {
		const auto index = GETMAPINDEX(s.x, s.y, _x_size, _y_size);
		return (
			s.x >= 0 and
			s.x < _x_size - 1 and
			s.y >= 0 and
			s.y < _y_size - 1 and
			static_cast<int>(_raw[index]) < _thres
		);
	};
	using Cost = double;
	using StateCostPair = std::pair<State, Cost>;
	const vector<StateCostPair> operator[] (const State& cur)  {
		vector<StateCostPair> legal_states;
		if (cur == IMAGINARY_GOAL) {
			for (const auto& goal : _concrete_goals) {
				legal_states.push_back({goal, 0});
			}
			return legal_states;
		}
		assert(get_gval(cur) < std::numeric_limits<double>::max());

		for (const auto& action : ACTIONS) {
			State next_state = cur + action;
			if (is_valid(next_state)) {
				auto cur_gval = get_gval(cur); //of current! not of next state
				assert(cur_gval < std::numeric_limits<double>::max());
				auto cost = static_cast<double>(
					_raw[GETMAPINDEX(next_state.x, next_state.y, _x_size, _y_size)]
				);
				update_gval(next_state, cur_gval + cost);
				legal_states.push_back({next_state, cost});
			}
		}
		// If the expanded state is one of the concrete goals,
		// then add the imaginary goal to the successors 
		// ah, then add as predecessor to the imaginary goal this
		// state
		if (std::find(_concrete_goals.begin(), _concrete_goals.end(), cur) != _concrete_goals.end()) {
			update_gval(IMAGINARY_GOAL, get_gval(cur));
			legal_states.push_back(
				{IMAGINARY_GOAL, 0}
			);
		}
		return legal_states;
	};
};
inline const State Map::IMAGINARY_GOAL{0, 0, 0};
inline const vector<Action> Map::ACTIONS = {
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
