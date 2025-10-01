#ifndef open_h
#define open_h
#include <cassert>
#include <cstdint>
#include <limits>
#include <memory>
#include <queue>
#include <sys/_types/_sigaltstack.h>
#include <unordered_map>
#include <iostream>
#include <tuple>
#include <utility>
#include <vector>

#include <boost/heap/d_ary_heap.hpp>

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
	using Heap = boost::heap::d_ary_heap<
		StateGvalHval,
		boost::heap::arity<4>,
		boost::heap::compare<GT>,
		boost::heap::mutable_<true>
	>;
	using StateHandleMap = unordered_map<State, Heap::handle_type, State::Hasher>;

	Heap _min_heap;
	StateHandleMap _state_handle_map;

	void insert_update(const State& state, double gval, Heuristic& h) {
		const auto& iter = _state_handle_map.find(state);
		auto hval = h(state);
		if (iter == _state_handle_map.end()) {
			auto handle = _min_heap.push(StateGvalHval{state, gval, hval});

			_state_handle_map.emplace(
				std::piecewise_construct,
				std::forward_as_tuple(state.x, state.y, state.t),
				std::forward_as_tuple(handle)
			);

		}
		else {
			auto handle = iter->second;
			auto [s, old_g, old_h] = *handle;
			assert(hval == old_h);

			if (gval < old_g) { //it's priority, not numerical value
				_min_heap.increase(handle, StateGvalHval{state, gval, hval});
				// this node now has higher priority!
			}
		}
	};

	State pop() {

		auto [state, g, h] = _min_heap.top();
		_min_heap.pop();
		_state_handle_map.erase(state);
		return state;
	};

};
#endif
