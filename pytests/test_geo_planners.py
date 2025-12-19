import sys
import pytest

from ompl import base as ob
from ompl import geometric as og
import time

from geo_env import create_simple_setup


def test_rrt_planner():
    # 1) Create SimpleSetup from environment
    ss = create_simple_setup()
    si = ss.getSpaceInformation()

    # 2) Create the RRT planner.
    # Let's choose addIntermediateStates=True for demonstration.
    rrt_planner = og.RRT(si, True)
    # 3) Configure some parameters
    rrt_planner.setGoalBias(0.1)
    rrt_planner.setRange(0.2)

    ss.setPlanner(rrt_planner)

    # Print them out
    print("Goal bias:", rrt_planner.getGoalBias())
    print("Range:", rrt_planner.getRange())
    print("Intermediate states:", rrt_planner.getIntermediateStates())


    # 7) Construct a PlannerTerminationCondition that stops after 1 second.
    ptc = ob.PlannerTerminationCondition(lambda: True)

    start_time = time.time()
    # Create a termination condition that returns True after 5 seconds
    ptc = ob.PlannerTerminationCondition(
        lambda: (time.time() - start_time) > 5
    )

    # 8) Solve
    # result = ss.solve(1.0)
    result = ss.solve(ptc)
    print("Planner result:", result)

    # 9) The `result` is a PlannerStatus object. Check if solution found:
    if result:
        print("Solution found!")
        # Optionally, retrieve the PathGeometric:
        solutionPath = ss.getSolutionPath()
        if solutionPath:
            print("Solution path length:", solutionPath.length())
            print("Solution path states:", solutionPath.getStateCount())
    else:
        print("No solution found within 1 second of planning time.")

if __name__ == "__main__":
    test_rrt_planner()