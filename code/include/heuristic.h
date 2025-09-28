#ifndef heuris_h
#define heuris_h
#include <cstddef>
#include <cmath>
#include <limits>
#include <utility>
#include <algorithm>

#include "state.h"
#include "planner.h"
#include "map.h"

using std::vector;
struct Heuristic {
	// using inheritance
	virtual double operator() (const State& state) const = 0;
};
//
struct DistanceHeuristic: public Heuristic {

	const vector<State>& _target;
	double _weight;

	DistanceHeuristic(const vector<State>& target, double weight): _target(target), _weight(weight){};

	double operator() (const State& state) const override {
		if (state == Map::IMAGINARY_GOAL) {
			return 0;
		}
		const auto& [cur_x, cur_y, cur_t] = state;
		const auto& [tar_x, tar_y, tar_t] = _target.at(cur_t);  //is it guaranteed that _target will be at least steps long? return std::sqrt(
		double dx = std::abs(cur_x - tar_x);
		double dy = std::abs(cur_y - tar_y);
		return _weight * std::sqrt(
			std::pow(dx, 2) +
			std::pow(dy, 2)
		);
	};
};
template <typename... Heuris>
struct SumHeuristic: public Heuristic {
	using WrappedHeuristic = std::reference_wrapper<const Heuristic>;

	std::array<WrappedHeuristic, sizeof...(Heuris)> _heuristics;
	SumHeuristic(const Heuris&... heuris): _heuristics{heuris...}{}
	double operator() (const State& state) const override {
		double sum{0.0};
		for (const auto& h : _heuristics) {
			sum += h(state);
		}
		return sum;
	};
};

struct WallHeuristic: public Heuristic {
	const Map& _map_ref;
	int _weight;
	WallHeuristic(const Map& map, int weight): _map_ref(map), _weight(weight){};
	double operator() (const State& state) const override {
		return _weight * 1.0 / _map_ref.distance_to_obs(state);
	};
};
struct OctileHeuristic: public Heuristic {
	const vector<State>& _target;
	double _weight;
	int _look_ahead;
	MODE _mode;

	OctileHeuristic(const vector<State>& target, double weight, int look_ahead, MODE mode): _target(target), _weight(weight), _look_ahead(look_ahead), _mode(mode) {};

	double operator() (const State& state) const override {
		if (_mode == MODE::FORWARD and state == Map::IMAGINARY_GOAL) {
			return 0;
		}
		else if (_mode == MODE::BACKWARD and state == _target.at(0)) {
			return 0;
		}
		const auto& [cur_x, cur_y, cur_t] = state;
		int index = std::min<int>(cur_t + _look_ahead, _target.size() - 1);
		const auto& [tar_x, tar_y, tar_t] = _target.at(index);  //is it guaranteed that _target will be at least steps long? return std::sqrt(
		double dx = std::abs(cur_x - tar_x);
		double dy = std::abs(cur_y - tar_y);
		return _weight * (
			std::sqrt(2) * std::min(dx, dy) + 
			std::abs(dx - dy)
		);
	}
};

struct RepulsionHeuristic: public Heuristic {
	const State& _robot_start;
	int _weight;
	RepulsionHeuristic(const State& robot_start, int weight): _robot_start(robot_start), _weight(weight){};
	double operator() (const State& state) const override {
		const auto& [cur_x, cur_y, cur_t] = state;
		const auto& [start_x, start_y, start_t] = _robot_start;
		auto dx = cur_x - start_x;
		auto dy = cur_y - start_y;
		double eps = 1.0e-6;
		auto distance = std::max(
			std::sqrt(dx * dx + dy * dy),
			eps
		);
		return _weight * 1.0 / distance;
	};
};
#endif
