#ifndef open_h
#define open_h
#include "state.h"
#include "heuristic.h"
#include <cassert>
#include <limits>
#include <unordered_map>
using std::unordered_map;
struct OpenList {
	using StateFvalPair = std::pair<State, int>;
	struct LT {
		bool operator() (const StateFvalPair& lhs, const StateFvalPair& rhs) const {
			return lhs.second < rhs.second;
		}
	};

	struct GT {
		bool operator() (const StateFvalPair& lhs, const StateFvalPair& rhs) const{
			return lhs.second > rhs.second;
		}
	};
	using Heap = boost::heap::fibonacci_heap<
		StateFvalPair,
		boost::heap::stable<false>,
		boost::heap::compare<GT>
	>;

	Heap _min_heap;
	unordered_map<State, Heap::handle_type, State::Hasher> _state_handle_index;

	/*
	* param state is the discovered state with noninf gvalue.
	* It is a successor of a expanded state, found via map[expanded], which returns a
	* vector of states 
	* It assumes that the caller has created a new non inf state
	* from calling ma
	*/
	void insert_update(const State& state, Heuristic& h, Map& map) {
		// state is NOT in the openlist, insert it.
		if (_state_handle_index.find(state) == _state_handle_index.end()) {
			int cur_gval = map.get_gval(state);
			assert(cur_gval != std::numeric_limits<int>::max());
			auto handle = _min_heap.push(
				StateFvalPair(state, cur_gval + h(state, map))
			);
			_state_handle_index[state] = handle;
		}
		// state IS in the openlist, update it.
		else { 
			const auto& handle = _state_handle_index[state];
			const auto& [cur_state, fval] = *handle;
			assert(h(cur_state, map) == h(state, map)); // this very much needs to be true
			_min_heap.update(handle, StateFvalPair{state, map.get_gval(state) + h(state, map)});
		}
	};
	State pop() {
		auto [state, cost] = _min_heap.top();
		_min_heap.pop();
		_state_handle_index.erase(state);
		return state;

	};

};
#endif
