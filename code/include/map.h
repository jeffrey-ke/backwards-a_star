#ifndef map_h
#define map_h
#include <algorithm>
#include <iterator>
#include <limits>
#include <unordered_map>
#include <unordered_set>

#include "planner.h"
#include "state.h"
using std::vector;
using std::unordered_map;
using std::unordered_set;

struct Map {
	static const vector<Action> ACTIONS;
	static const State IMAGINARY_GOAL;
	static constexpr double BIG_GVAL = std::numeric_limits<double>::infinity();


	int* _raw;
	int _x_size, _y_size, _thres;
	unordered_set<State, State::Hasher> _concrete_goals;
	using gval = double;
	unordered_map<State, gval, State::Hasher> _state_gval_table;

	Map(int* raw, int x_size, int y_size, int thres, const vector<State>& concrete_goals): 
		_raw(raw),
		_x_size(x_size),
		_y_size(y_size),
		_thres(thres)
	{
		for (const auto& goal: concrete_goals) {
			update_gval(goal, BIG_GVAL);
			_concrete_goals.insert(goal);

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

	const gval get_gval(const State& state) const {
		if (_state_gval_table.find(state) != _state_gval_table.end())
			return _state_gval_table.at(state);
		else {
			return std::numeric_limits<double>::infinity();
		}
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
	using Gval = double;
	using StateGvalPair = std::pair<State, Gval>;
	const vector<StateGvalPair> operator[] (const State& cur)  {
		vector<StateGvalPair> legal_states;
		for (const auto& action : ACTIONS) {
			State next_state = cur + action;
			if (is_valid(next_state)) {
				auto cur_gval = get_gval(cur); //of current! not of next state
				auto cost = static_cast<double>(
					_raw[GETMAPINDEX(next_state.x, next_state.y, _x_size, _y_size)]
				);
				update_gval(next_state, cur_gval + cost);
				legal_states.push_back({next_state, cur_gval + cost});
			}
		}
		if (_concrete_goals.find(cur) != _concrete_goals.end()) {
			auto cur_gval = get_gval(cur);
			update_gval(IMAGINARY_GOAL, cur_gval + 1);
			legal_states.push_back(
				{IMAGINARY_GOAL, cur_gval + 1}
			);
		}
		return legal_states;
	};
	const vector<StateGvalPair> backwards(const State& s) {
		vector<StateGvalPair> backwards_preds;
		const vector<Action> actions = {
			Action{-1, -1, -1},
			Action{-1, 0, -1},
			Action{-1, 1, -1},
			Action{0, -1, -1},
			Action{0, 0, -1},
			Action{0, 1, -1},
			Action{1, -1, -1},
			Action{1, 0, -1},
			Action{1, 1, -1},
		};
		if (s == IMAGINARY_GOAL) {
			for (const auto& goal : _concrete_goals) {
				auto goal_gval = get_gval(goal);
				backwards_preds.push_back({goal, goal_gval});
			}
			return backwards_preds;
		}
		for (const auto& a : actions) {
			const auto next_state = s + a;
			if (is_valid(next_state)) {
				auto cost = static_cast<double>(
					_raw[
					GETMAPINDEX(next_state.x, next_state.y, _x_size, _y_size)
					]
				);
				auto next_gval = get_gval(next_state); // this MUST exist
				backwards_preds.push_back({next_state, next_gval + cost});
			}
		}
		return backwards_preds;
	}
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
