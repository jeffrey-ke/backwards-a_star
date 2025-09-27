#ifndef map_h
#define map_h
#include <algorithm>
#include <iterator>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <cstdint>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "planner.h"
#include "state.h"
using std::array;
using std::vector;
using std::unordered_map;
using std::unordered_set;

struct Map {
	static const array<Action, 9> ACTIONS;
	static const State IMAGINARY_GOAL;
	static constexpr double BIG_GVAL = std::numeric_limits<double>::infinity();


	int* _raw;
	int _x_size, _y_size, _thres;
	unordered_set<State, State::Hasher> _concrete_goals;
	using gval = double;
	unordered_map<State, gval, State::Hasher> _state_gval_table;
	mutable cv::Mat _distance_map;

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
		_legal_states.reserve(ACTIONS.size() + 1);
		createDistanceMap(_raw, _x_size, _y_size, _thres);
	};

	void createDistanceMap(int* raw_map, int x_size, int y_size, int thresh) const {
		cv::Mat is_obstacle(y_size, x_size, CV_8UC1);
		for (int y = 0; y < y_size; ++y) {
			for (int x = 0; x < x_size; ++x) {
       				auto cost = static_cast<int>(raw_map[GETMAPINDEX(x, y, x_size, y_size)]);
				is_obstacle.at<uint8_t>(y, x) = (cost > thresh) ? 0 : 255;
			}
		}
		cv::distanceTransform(is_obstacle, _distance_map, cv::DIST_L2, cv::DIST_MASK_PRECISE);
	}

	double distanceToWall(const State& state) const {
		return (is_valid(state)) ? _distance_map.at<float>(state.y, state.x) : 0.0;
	}

	void set_start(const State& state) {
		update_gval(state, 0.0);
	};
	void update_gval(const State& state, gval val) {
		auto [iter, inserted] = _state_gval_table.try_emplace(state, val);
		if (!inserted and val < iter->second) {
			iter->second = val;
		}
	};

	const gval get_gval(const State& state) const {
		auto it = _state_gval_table.find(state);
		return (it != _state_gval_table.end()) ? it->second : std::numeric_limits<double>::infinity();
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
	mutable vector<StateGvalPair> _legal_states;
	const vector<StateGvalPair>& operator[] (const State& cur)  {
		_legal_states.clear();
		auto cur_gval = get_gval(cur);
		for (const auto& action : ACTIONS) {
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
	mutable vector<StateGvalPair> backwards_preds;
	const vector<StateGvalPair>& backwards(const State& s) {
		backwards_preds.clear();
		const array<Action, 9> actions = {
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
inline const array<Action, 9> Map::ACTIONS = {
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
