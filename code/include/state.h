#ifndef STATE_H
#define STATE_H
#include <utility>
#include <cstddef>

#include <boost/heap/fibonacci_heap.hpp>
#include "boost/functional/hash_fwd.hpp"

using std::size_t;
using StateCost = std::pair<struct State, size_t>;
struct State{
	size_t x,y,t;
	bool operator==(const State& rhs) const {
		return (
		(x == rhs.x) and
		(y == rhs.y) and
		(t == rhs.t)
	);
	};
	struct Hasher {
		size_t operator() (const State& s) {
			size_t seed{};
			boost::hash_combine(seed, s.x);
			boost::hash_combine(seed, s.y);
			boost::hash_combine(seed, s.t);
			return seed;
		};
	};
};

struct Heuristic {
	// using inheritance
	using h_val = size_t;
	Map _map;
	Heuristic(Map map, Type type): _map(map) {}; // these are just non-owning pointers, it's fine to copy by value
	virtual h_val operator() (const State& state) = 0;
};

struct DistanceHeuristic: Heuristic {
	using Heuristic::h_val;

	Target _target;

	DistanceHeuristic(Map map, Type type, Target target): Heuristic(map, type), _target(target) {};
	h_val operator() (const State& state) override {
		const auto& [cur_x, cur_y, cur_t] = state;
		const auto& [tar_x, tar_y] = _target[cur_t];
		return std::sqrt(
			std::pow(cur_x - tar_x, 2) +
			std::pow(cur_y - tar_y, 2)
		);
	};
};
#endif
