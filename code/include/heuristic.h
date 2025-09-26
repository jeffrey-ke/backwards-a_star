#ifndef heuris_h
#define heuris_h
#include <cstddef>
#include <cmath>
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
#endif
