#ifndef STATE_H
#define STATE_H
#include <utility>
#include <cstddef>

#include <boost/heap/fibonacci_heap.hpp>
#include "boost/functional/hash_fwd.hpp"

using StateCost = std::pair<struct State, int>;
struct State{
	int x,y,t;
	State& operator+(const State& rhs) {
		x += rhs.x;
		y += rhs.y;
		t += rhs.t;
		return *this;
	};
	bool operator==(const State& rhs) const {
		return (
		(x == rhs.x) and
		(y == rhs.y) and
		(t == rhs.t)
		);
	};
	bool operator!= (const State& rhs) const {
		return !(*this == rhs);
	};
	struct Hasher {
		int operator() (const State& s) const{
			size_t seed{};
			boost::hash_combine(seed, s.x);
			boost::hash_combine(seed, s.y);
			boost::hash_combine(seed, s.t);
			return static_cast<int>(seed);
		};
	};
};

State operator+(const State& lhs, const State& rhs) {
	return lhs + rhs;
};

#endif
