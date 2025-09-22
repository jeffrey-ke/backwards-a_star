#ifndef open_h
#define open_h
#include "state.h"
#include "heuristic.h"
struct OpenList {
	using g_val = int;
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
	using Index = std::unordered_map<State, Heap::handle_type, State::Hasher>;

	Heap _min_heap;
	Index _state_handle_index;

	void insert_update(const State& state, int g_val, const Heuristic& h) {
		if (_state_handle_index.find(state) == _state_handle_index.end()) {
			auto handle = _min_heap.push(
				StateCost(state, g_val + h(state))
			);
			_state_handle_index[state] = handle;
		}
		else {
			const auto& handle = _state_handle_index[state];
			_min_heap.update(handle, StateCost{state, g_val + h(state)});
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
