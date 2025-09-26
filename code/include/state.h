#ifndef STATE_H
#define STATE_H
#include <utility>
#include <cstddef>
#include <limits>

#include "boost/functional/hash_fwd.hpp"

struct Action {
	int dx,dy,dt;
};
struct State{
	int x,y,t;
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

inline State operator+(const State& lhs, const Action& rhs) {
	return State{lhs.x + rhs.dx, lhs.y + rhs.dy, lhs.t  + rhs.dt};
};

#endif
