#ifndef open_h
#define open_h
#include <cassert>
#include <limits>
#include <queue>
#include <unordered_map>
#include <iostream>
#include <tuple>

#include <boost/heap/fibonacci_heap.hpp>
#include <vector>

#include "state.h"
#include "heuristic.h"
using std::unordered_map;
struct OpenList {
	using StateGvalHval = std::tuple<State, double, double>;
	struct LT {
		bool operator() (const StateGvalHval& lhs, const StateGvalHval& rhs) const {
			return std::get<1>(lhs) + std::get<2>(lhs) < std::get<1>(rhs) + std::get<2>(rhs);
		}
	};

	struct GT {
		bool operator() (const StateGvalHval& lhs, const StateGvalHval& rhs) const{
			return std::get<1>(lhs) + std::get<2>(lhs) > std::get<1>(rhs) + std::get<2>(rhs);
		}
	};
	using Heap = std::priority_queue<
		StateGvalHval,
		vector<StateGvalHval>,
		GT
	>;

	Heap _min_heap;
	unordered_map<State, int, State::Hasher> _best_gval;


	/*
	* param state is the discovered state with noninf gvalue.
	* It is a successor of a expanded state, found via map[expanded], which returns a
	* vector of states 
	* It assumes that the caller has created a new non inf state
	* from calling ma
	*/
	void insert_update(const State& state, double gval, Heuristic& h) {
		// state is NOT in the openlist, insert it.
		auto hval = h(state);
		auto [iter, inserted] = _best_gval.try_emplace(state, gval);
		if (!inserted and gval < iter->second) {
			iter->second = gval;
		}
		_min_heap.emplace(state, gval, hval);
	};
	State pop() {
		while (true) {
			auto [state, g, h] = _min_heap.top();
			_min_heap.pop();
			auto best_gval = _best_gval.at(state);
			if (best_gval == g) {
				return state;
			}
		}
		
	};

};
#endif
