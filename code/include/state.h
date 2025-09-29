#ifndef STATE_H
#define STATE_H
#include <ostream>
#include <utility>
#include <cstddef>
#include <limits>

#include "boost/functional/hash_fwd.hpp"

struct Action {
	int dx,dy,dt;
};
struct State{
	int x,y,t;
	State(): x(0), y(0), t(0) {};
	State(int _x, int _y, int _t): x(_x), y(_y), t(_t){};
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
	bool operator< (const State& rhs) const {
		// as if x and y were a two digit number
		if (x < rhs.x){ // 1y, 2y
			return true;
		}
		else if (x > rhs.x) { //2y, 1y
			return false;
		}
		else { //xy1, xy2
			if (y < rhs.y)  // x1 x2
				return true;
			else //x2 x1
				return false;
		}
	}
	struct Hasher {
		size_t operator() (const State& s) const{
			size_t seed{};
			boost::hash_combine(seed, s.x);
			boost::hash_combine(seed, s.y);
			boost::hash_combine(seed, s.t);
			return seed;
		};
	};
};

inline State operator+(const State& lhs, const Action& rhs) {
	return State{lhs.x + rhs.dx, lhs.y + rhs.dy, lhs.t  + rhs.dt};
};
inline std::ostream& operator<<(std::ostream& os, const State& s) {
	return os << "(" << s.x << ", " << s.y << ", " << s.t << ")";
};

#endif
