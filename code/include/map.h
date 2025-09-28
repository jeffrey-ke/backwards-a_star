#ifndef map_h
#define map_h
#include <algorithm>
#include <cassert>
#include <iterator>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <cstdint>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

#include "planner.h"
#include "state.h"
using std::array;
using std::vector;
using std::unordered_map;
using std::unordered_set;

struct Map {

	using gval = double;
	using StateGvalPair = std::pair<State, gval>;

	static const State IMAGINARY_GOAL;
	static constexpr double BIG_GVAL = std::numeric_limits<double>::infinity();

	MODE _mode;
	int* _raw;
	int _x_size, _y_size, _thres;
	State _start;
	array<Action, 9> _actions;
	unordered_set<State, State::Hasher> _concrete_goals;
	unordered_map<State, gval, State::Hasher> _state_gval_table;
	mutable vector<StateGvalPair> _legal_states;
	mutable cv::Mat _distance_map;

	Map(int* raw, int x_size, int y_size, int thres, const vector<State>& concrete_goals, State start, MODE mode): 
		_raw(raw),
		_x_size(x_size),
		_y_size(y_size),
		_thres(thres),
		_start(start),
		_mode(mode)
	{
		for (const auto& goal: concrete_goals) {
			update_gval(
				goal, 
				(mode == MODE::FORWARD) ? BIG_GVAL : 1
			);
			_concrete_goals.insert(goal);

		}
		auto dt = (mode == MODE::FORWARD) ? 1 : -1;
		_actions = create_actions(dt);
		_legal_states.reserve(_actions.size() + 1);
		create_dmap(_raw, _x_size, _y_size, _thres);
	};

	array<Action, 9> create_actions(int dt) {
		assert(std::abs(dt) == 1);
		return {
			Action{-1, -1, dt},
			Action{-1, 0, dt},
			Action{-1, 1, dt},
			Action{0, -1, dt},
			Action{0, 0, dt},
			Action{0, 1, dt},
			Action{1, -1, dt},
			Action{1, 0, dt},
			Action{1, 1, dt},
		};
	}

	void create_dmap(int* raw_map, int x_size, int y_size, int thresh) const {
		cv::Mat is_obstacle(y_size, x_size, CV_8UC1);
		for (int y = 1; y <= y_size; ++y) {
			for (int x = 1; x <= x_size; ++x) {
       				auto cost = static_cast<int>(raw_map[GETMAPINDEX(x, y, x_size, y_size)]);
				is_obstacle.at<uint8_t>(y - 1, x - 1) = (cost > thresh) ? 0 : 255;
			}
		}
		cv::distanceTransform(is_obstacle, _distance_map, cv::DIST_L2, cv::DIST_MASK_PRECISE);
	}

	double distance_to_obs(const State& state) const {
		return (is_valid(state)) ? _distance_map.at<float>(state.y - 1, state.x - 1) : 0.0;
	}

	void update_gval(const State& state, gval val) {
		auto [iter, inserted] = _state_gval_table.try_emplace(state, val);
		if (!inserted and val < iter->second) {
			iter->second = val;
		}
	};

	const gval get_gval(const State& state) const {
		auto it = _state_gval_table.find(state);
		return (it != _state_gval_table.end()) ? it->second : BIG_GVAL;
	};

	bool is_valid(const State& s) const {
		const auto index = GETMAPINDEX(s.x, s.y, _x_size, _y_size);
		return (
			s.x >= 0 and
			s.x < _x_size - 1 and
			s.y >= 0 and
			s.y < _y_size - 1 and
			static_cast<int>(_raw[index]) < _thres and
			s.t >= 0
		);
	};

	const vector<StateGvalPair>& successors(const State& cur) {
		return (_mode == MODE::FORWARD) ? forwards_succ(cur) : backwards_succ(cur);
	};

	const vector<StateGvalPair>& forwards_succ(const State& cur)  {
		_legal_states.clear();
		auto cur_gval = get_gval(cur);
		assert(cur_gval < BIG_GVAL);
		for (const auto& action : _actions) {
			State next_state = cur + action;
			if (is_valid(next_state)) {
				auto cost = static_cast<double>(
					_raw[GETMAPINDEX(next_state.x, next_state.y, _x_size, _y_size)]
				);
				update_gval(next_state, cur_gval + cost);
				_legal_states.push_back({next_state, cur_gval + cost});
			}
		}
		if (_concrete_goals.find(cur) != _concrete_goals.end()) {
			update_gval(IMAGINARY_GOAL, cur_gval + 1);
			_legal_states.push_back(
				{IMAGINARY_GOAL, cur_gval + 1}
			);
		}
		return _legal_states;
	};

	const vector<StateGvalPair>& backwards_succ(const State& cur) {
		_legal_states.clear();
		auto cur_g = get_gval(cur);
		if (cur == IMAGINARY_GOAL) {
			assert(get_gval(cur) == 0);
			for (const auto& g : _concrete_goals){
				auto goal_g = get_gval(g);
				assert(goal_g == 1);
				_legal_states.push_back({g, get_gval(g)});
			}
		}
		else {
			for (const auto& act : _actions) {
				State next = cur + act;
				if (is_valid(next)) {
					auto cost = static_cast<double>(
						_raw[GETMAPINDEX(next.x, next.y, _x_size, _y_size)]
					);
					auto next_g = cur_g + cost;
					update_gval(next, next_g); //this is actually necessary, because it will
					//make states that it's just traveled a higher gvalue, but this update gvalue
					//only changes it if it's lower
					_legal_states.push_back({next, next_g});
				}
			}
		}
		return _legal_states;
	};

	mutable vector<StateGvalPair> backwards_preds;
	const vector<StateGvalPair>& predecessors(const State& s) {
		backwards_preds.clear();
		return (_mode == MODE::FORWARD) ? backtrack_forward(s) : backtrack_backward(s);
	};

	const vector<StateGvalPair>& backtrack_forward(const State& s) {
		auto actions = create_actions(-1);
		if (s == IMAGINARY_GOAL) {
			for (const auto& goal : _concrete_goals) {
				auto goal_gval = get_gval(goal);
				backwards_preds.push_back({goal, goal_gval});
			}
			return backwards_preds;
		}
		for (const auto& a : actions) {
			auto next_state = s + a;
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
	};

	const vector<StateGvalPair>& backtrack_backward(const State& s) {
		auto actions = create_actions(1);
		for (const auto& act : actions)  {
			State next = s + act;
			if (is_valid(next)) {
				auto cost = static_cast<double>(
					_raw[GETMAPINDEX(next.x, next.y, _x_size, _y_size)]
				);
				auto next_g = get_gval(next);
				backwards_preds.push_back({next, next_g + cost});
			}
		}
		return backwards_preds;
	}
};
inline const State Map::IMAGINARY_GOAL{0, 0, 0};
#endif
