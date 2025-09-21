#ifndef open_h
#define open_h
#include "state.h"
#include "map.h"
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
#endif
