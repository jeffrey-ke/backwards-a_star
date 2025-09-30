#ifndef heuris_h
#define heuris_h
#include <array>
#include <cstddef>
#include <cmath>
#include <filesystem>
#include <functional>
#include <limits>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <utility>
#include <algorithm>
#include <iostream>

#include <boost/heap/d_ary_heap.hpp>
#include <variant>
#include <vector>

#include "state.h"
#include "planner.h"
#include "map.h"


using std::vector;
using std::unordered_map;
using std::tuple;
struct Heuristic {
	// using inheritance
	virtual double operator() (const State& state) const = 0;
	virtual ~Heuristic() = default;   // <-- important

};
//
struct DijkstraHeuristic: public Heuristic {
	using StateDistanceStep = tuple<State, double, int>;
	struct StateDistanceGT {
		bool operator() (const StateDistanceStep& lhs, const StateDistanceStep& rhs) const {
			return std::get<1>(lhs) > std::get<1>(rhs);
		}
	};
	using Heap = boost::heap::d_ary_heap<
		StateDistanceStep,
		boost::heap::arity<4>,
		boost::heap::compare<StateDistanceGT>,
		boost::heap::mutable_<true>
	>;
	using StateSet = std::set<State>;

	Map _map;
	State _src;
	// StateDistanceMap _distance_from_src; this is inside map
	Heap _unvisited;
	StateSet  _visited;
	unordered_map<State, Heap::handle_type, State::Hasher> _state_handle_index;
	int _weight;
	int _max_path_len;

	DijkstraHeuristic(int* raw_map, int x_size, int y_size, int thres, const vector<State>& concrete_goals, const State& src, int weight, int max_path_len): 
		_src(src),
		_map(raw_map, x_size, y_size, thres, concrete_goals, src, MODE::DIJK),
		_weight(weight),
		_max_path_len(max_path_len)
		{
		auto handle = _unvisited.push({src, 0.0, 0});
		
		while (_unvisited.size() > 0) {
			auto [cur_state, cost, pathlen] = _unvisited.top();
			_unvisited.pop();
			_visited.insert(cur_state);
			if (pathlen > max_path_len)
				continue;

			auto neighbors = _map.successors(cur_state);
			for (const auto& [n, dist] : neighbors) {
				if (_visited.find(n) == _visited.end()) {
					insert_update(_unvisited, n, dist, pathlen + 1);
				}
			}
		}
		for (const auto& g : concrete_goals) {
			// assert(_map.get_gval(State{g.x, g.y, 0}) < Map::BIG_GVAL);
			if (_visited.find(g) == _visited.end())
				std::cout << "Dijkstra Warning! Goal: " << g.x << ", " << g.y << ", " << g.t << " not found!" << std::endl;
		}
	}
	void insert_update(Heap& unvisited, const State& s, double dist, int pathlen){
		auto iter = _state_handle_index.find(s);
		if (iter == _state_handle_index.end()) {
			auto handle = unvisited.push(StateDistanceStep{s, dist, pathlen});
			_state_handle_index.emplace(
				std::piecewise_construct,
				std::forward_as_tuple(s.x, s.y, s.t),
				std::forward_as_tuple(handle)
			);
		}
		else {
			auto handle = iter->second;
			auto [state, old_dist, old_pathlen] = *handle;
			if (dist < old_dist) {
				unvisited.increase(handle, StateDistanceStep{state, dist, pathlen});
			}
		}
	}
	double operator() (const State& state) const override {
		if (state == Map::IMAGINARY_GOAL) {
			return 0;
		}
		auto steps = _map.get_steps(State{state.x, state.y, _src.t});
		if (steps > (state.t - _src.t)) {
			return Map::BIG_GVAL;
		}
		auto dist = _map.get_gval(State{state.x, state.y, _src.t});
		// std::cout << "dist: " << dist << std::endl;
		// assert (dist < Map::BIG_GVAL);
		return _weight * (dist + state.t);
	}
};
struct DistanceHeuristic: public Heuristic {

	const vector<State>& _target;
	double _weight;

	DistanceHeuristic(const vector<State>& target, double weight): _target(target), _weight(weight){};

	double operator() (const State& state) const override {
		if (state == Map::IMAGINARY_GOAL) {
			return 0;
		}
		const auto& [cur_x, cur_y, cur_t] = state;
		const auto& [tar_x, tar_y, tar_t] = _target.at(cur_t);  //is it guaranteed that _target will be at least steps long? return std::sqrt(
		double dx = std::abs(cur_x - tar_x);
		double dy = std::abs(cur_y - tar_y);
		return _weight * std::sqrt(
			std::pow(dx, 2) +
			std::pow(dy, 2)
		);
	};
};
template <typename... Heuris>
struct SumHeuristic: public Heuristic {
	using WrappedHeuristic = std::reference_wrapper<const Heuristic>;

	std::array<WrappedHeuristic, sizeof...(Heuris)> _heuristics;
	SumHeuristic(const Heuris&... heuris): _heuristics{heuris...}{}
	double operator() (const State& state) const override {
		double sum{0.0};
		for (const auto& h : _heuristics) {
			sum += h(state);
		}
		return sum;
	};
};

struct WallHeuristic: public Heuristic {
	const Map& _map_ref;
	int _weight;
	WallHeuristic(const Map& map, int weight): _map_ref(map), _weight(weight){};
	double operator() (const State& state) const override {
		if (state == Map::IMAGINARY_GOAL) {
			return 0;
		}
		return _weight * 1.0 / _map_ref.distance_to_obs(state);
	};
};

struct OctileHeuristic: public Heuristic {
	const vector<State> _target;
	double _weight;
	int _look_ahead;
	MODE _mode;

	OctileHeuristic(const vector<State>& target, double weight, int look_ahead, MODE mode): _target(target), _weight(weight), _look_ahead(look_ahead), _mode(mode) {};

	double operator() (const State& state) const override {
		if (_mode == MODE::FORWARD and state == Map::IMAGINARY_GOAL) {
			return 0;
		}
		else if (_mode == MODE::BACKWARD and state == _target.at(0)) {
			return 0;
		}
		const auto& [cur_x, cur_y, cur_t] = state;
		int index = std::min<int>(cur_t + _look_ahead, _target.size() - 1);
		const auto& [tar_x, tar_y, tar_t] = _target.at(index);  //is it guaranteed that _target will be at least steps long? return std::sqrt(
		double dx = std::abs(cur_x - tar_x);
		double dy = std::abs(cur_y - tar_y);
		auto distance = (
			std::sqrt(2) * std::min(dx, dy) + 
			std::abs(dx - dy)
		);
		if (_mode == MODE::BACKWARD and state.t < distance) {
			return Map::BIG_GVAL;
		}
		return _weight * distance;
	}
};

struct RepulsionHeuristic: public Heuristic {
	const State& _robot_start;
	int _weight;
	RepulsionHeuristic(const State& robot_start, int weight): _robot_start(robot_start), _weight(weight){};
	double operator() (const State& state) const override {
		const auto& [cur_x, cur_y, cur_t] = state;
		const auto& [start_x, start_y, start_t] = _robot_start;
		auto dx = cur_x - start_x;
		auto dy = cur_y - start_y;
		double eps = 1.0e-6;
		auto distance = std::max(
			std::sqrt(dx * dx + dy * dy),
			eps
		);
		return _weight * 1.0 / distance;
	};
};
#endif
