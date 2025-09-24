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
	virtual int operator() (const State& state, Map &) const = 0;
};
//
struct DistanceHeuristic: public Heuristic {

	const vector<State>& _target;

	DistanceHeuristic(const vector<State>& target): _target(target){};

	int operator() (const State& state, Map& map) const override {
		if (state == Map::IMAGINARY_GOAL) {
			return 0;
		}
		const auto& [cur_x, cur_y, cur_t] = state;
		const auto cur_t_clamped = std::min<int>(cur_t, _target.size() - 1);
		const auto& [tar_x, tar_y, tar_t] = _target.at(cur_t_clamped);  //is it guaranteed that _target will be at least steps long? return std::sqrt(
		return std::sqrt(
			std::pow(cur_x - tar_x, 2) +
			std::pow(cur_y - tar_y, 2)
		);
	};
};
#endif
