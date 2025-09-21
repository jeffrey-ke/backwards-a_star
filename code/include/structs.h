#ifndef STATE_H
#define STATE_H
#include <utility>
#include <cstddef>

#include <boost/heap/fibonacci_heap.hpp>
#include "boost/functional/hash_fwd.hpp"

using std::size_t;
using StateCost = std::pair<class State, size_t>;
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

struct OpenList {
	using g_val = size_t;
	struct StateLT {
		bool operator() (const StateCost& lhs, const StateCost& rhs) const {
			return lhs.second < rhs.second;
		}
	};

	struct StateGT {
		bool operator() (const StateCost& lhs, const StateCost& rhs) const{
			return lhs.second > rhs.second;
		}
	};
	using Heap = boost::heap::fibonacci_heap<
	StateCost,
	boost::heap::stable<false>,
	boost::heap::compare<StateGT>
	>;
	using Index = std::unordered_map<State, Heap::handle_type>;

	Heap _min_heap;
	Index _state_handle_index;

	OpenList(): _min_heap(), _state_handle_index() {};
	void insert_update(const State& state, size_t g_val, const Heuristic& h) {
		if (_state_handle_index.find(state) == _state_handle_index.end()) {
			auto handle = _min_heap.push(
				StateCost(state, g_val + h(state))
			);
			_state_handle_index[state] = handle;
		}
		else {
			auto handle = _state_handle_index[state];
			_min_heap.update(handle, g_val + h(state));
		}
	};
	State pop() {
		auto [state, cost] = _min_heap.top();
		_min_heap.pop();
		_state_handle_index.erase(state);
		return state;

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
