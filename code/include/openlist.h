#ifndef open_h
#define open_h
#include <cassert>
#include <limits>
#include <unordered_map>
#include <iostream>

#include "state.h"
#include "heuristic.h"
using std::unordered_map;
struct OpenList {
	using StateFvalPair = std::pair<State, double>;
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
	void insert_update(const State& state, double gval, Heuristic& h) {
		// state is NOT in the openlist, insert it.
		const auto hval = h(state);
		if (_state_handle_index.find(state) == _state_handle_index.end()) {
			auto handle = _min_heap.push(
				StateFvalPair(state, gval + hval)
			);
			_state_handle_index[state] = handle;
		}
		// state IS in the openlist, update it.
		else { 
			const auto& handle = _state_handle_index[state];
			const auto& [cur_state, fval] = *handle;
			if (gval + hval < fval) {
				_min_heap.decrease(handle, StateFvalPair{state, gval + hval});
			}
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
