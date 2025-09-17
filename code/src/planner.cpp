#include "../include/planner.h"
#include <math.h>
#include <algorithm>
#include <vector>
#include <queue>

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#if !defined(SUCC)
#define SUCC
#endif

#define NUMOFDIRS 8
void planner(
    int* map,
    int collision_thresh,
    int x_size,
    int y_size,
    int robotposeX,
    int robotposeY,
    int target_steps,
    int* target_traj,
    int targetposeX,
    int targetposeY,
    int curr_time,
    int* action_ptr
    ) {
    /*
     * todo:
     * implement StateXYT
     * how will I implement succ
     * don't forget to implement operator== and operator!=
     * parse goals helper with namespace helper::
     * multigoal_create
     * backtrack
     * parse action_ptr
     * how will state transitions work? How do I calculate the cost between state transitions?
     *  it's very possible I can update the open set with the current times.
     * What are my g values? What are my h values?
     * clearly my g values are the time and distance it will take to get to the target
     * will my g values be time elapsed?
     */
    std::priority_queue<
        StateXYT, 
        std::vector<StateXYT>, 
        std::greater<StateXYT>
    > open;

    const StateXYT start{robotposeX, robotposeY, curr_time};
    const std::vector<StateXYT>& goals = helper::parse_goals(target_traj, target_poseX, targetposeY, curr_time);
    const StateXYT& imaginary_goal = helper::multigoal_create(goals);

    do{
        expanded = open.pop();
        expand_state(open, expanded);
    }
    while (expanded != imaginary_goal);
    helper::backtrack(expanded, start)
    *action_ptr = step
}
