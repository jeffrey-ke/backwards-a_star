#ifndef open_h
#define open_h
#include "state.h"
#include "heuristic.h"
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
	using Index = std::unordered_map<State, Heap::handle_type, State::Hasher>;

	Heap _min_heap;
	Index _state_handle_index;

	/*
	* param state is the discovered state with noninf gvalue.
	* It is a successor of a expanded state, found via map[expanded], which returns a
	* vector of states 
	* It assumes that the caller has created a new non inf state
	* from calling ma
	* /
	void insert_update(const State& state, const Heuristic& h) {
		assert(! state.isinf());

		// state is NOT in the openlist, insert it.
		if (_state_handle_index.find(state) == _state_handle_index.end()) {
			auto handle = _min_heap.push(
				StateFvalPair(state, state.gval+ h(state))
			);
			_state_handle_index[state] = handle;
		}
		// state IS in the openlist, update it. OpenList becomes the table of all states and their current gvalues
		else { 
			const auto& handle = _state_handle_index[state];
			const auto& [cur_state, fval] = *handle;
			if (state.gval < cur_state.gval){
				assert(state.gval + h(state) < cur_state.gval + h(cur_state));
				_min_heap.decrease(handle, StateFvalPair{state, g_val + h(state)});
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
