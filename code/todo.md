# Completed constructs (due for review)
- Map: 
- parse_goals
- (not needed), parse goals now does this. init map, in helper, so I need to add the imaginary goal, s.t. the map adds as successor one of the imaginary goals to the concrete goals
- backtrack

# doing now:
- heuristic

# what planner needs:
- easy place to get all the aliases (look up the best practices for this)


































conclusions:
* ARA* with N seconds of computation time (use chrono), with all succ of current
getting that time


Thoughts:
* it's a bit weird how you don't have a hashmap to represent your graph yet. Maybe make that first jeff for an easier interface to the world jeff.
* What does the OPEN list need to do? look up, pop, update the g-values
* what exactly is my g? distance. Don't really need to include time (unless I do
* what exactly beyond a struct holding 3 values is a statexyt?
* how will I calculate the heuristic? A functor that takes in a statexyt



 * todo:
 * high level I want to implement a multi heuristic A*, but for now I'll implement vanilla weighted A
 * how to implement different heuristics? probably as a functor that takes a statexyt

 * implement your priorityqueue with lookup
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
