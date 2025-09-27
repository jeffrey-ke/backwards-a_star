#ifndef heuris_h
#define heuris_h
#include <cstddef>
#include <cmath>
#include <limits>
#include <utility>
#include <algorithm>

#include "state.h"
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
struct WeightedCombinationHeuristic: public Heuristic {
	const Heuristic& _h1;
	const Heuristic& _h2;
	const Heuristic& _h3;
	int _w1, _w2, _w3;
	WeightedCombinationHeuristic(const Heuristic& h1, const Heuristic& h2, const Heuristic& h3, int weight1, int weight2, int weight3): _h1(h1), _h2(h2), _h3(h3), _w1(weight1), _w2(weight2), _w3(weight3){
	}
	double operator() (const State& state) const override {
		return _w1 * _h1(state) + _w2 * _h2(state) + _w3 * _h3(state);
	}
};

struct WallHeuristic: public Heuristic {
	const Map& _map_ref;
	int _weight;
	WallHeuristic(const Map& map, int weight): _map_ref(map), _weight(weight){};
	double operator() (const State& state) const override {
		return _weight * 1.0 / _map_ref.distanceToWall(state);
	};
};
struct OctileHeuristic: public Heuristic {
	const vector<State>& _target;
	double _weight;
	int _look_ahead;

	OctileHeuristic(const vector<State>& target, double weight, int look_ahead): _target(target), _weight(weight), _look_ahead(look_ahead){};

	double operator() (const State& state) const override {
		if (state == Map::IMAGINARY_GOAL) {
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
