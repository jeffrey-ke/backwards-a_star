#ifndef helper_h
#define helper_h
#include <vector>

#include "state.h"

using Target = std::vector<State>;
namespace helper {
inline Target parse_goals(int* target_traj, int target_steps, int target_poseX, int target_poseY, int curr_time){
	Target target;
	target.resize(target_steps);
	for (int time_step = 0; time_step < target_steps; ++time_step) {
		target.at(time_step) = State{target_traj[time_step], target_traj[time_step + target_steps], time_step};
	}
	return target;
}
};
#endif
