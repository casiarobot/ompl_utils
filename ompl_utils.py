from ompl import base as ob
from ompl import geometric as og
import numpy as np

def block(xmax, xmin, ymax, ymin, zmax, zmin):
    """
    Returns a function that represents a rectangular prism that returns true if the inputs are inside the block.
    """
    return lambda x, y, z: (xmin <= x <= xmax and ymin <= y <= ymax and zmin <= z <= zmax)

def sphere(xc, yc, zc, r):
    """
    Returns a function that represents a sphere that returns true if the inputs are inside the sphere.
    """
    return lambda x, y, z: np.linalg.norm(np.array((xc - x, yc - y, zc - z))) <= r

def axis_limit(lim, maximum=True, axis='x'):
    """
    Returns a function that represents axis limits. Takes in a limit and whether or not it represents a maximum or minimum limit and which axis it corresponds to.
    """
    if axis == 'x':
        if maximum:
            return lambda x, y, z: x >= lim
        else:
            return lambda x, y, z: x <= lim
    elif axis == 'y':
        if maximum:
            return lambda x, y, z: y >= lim
        else:
            return lambda x, y, z: y <= lim
    elif axis == 'z':
        if maximum:
            return lambda x, y, z: z >= lim
        else:
            return lambda x, y, z: z <= lim

def valid_fn(constraints):
    """
    Returns a function that represents a list of the constraints defined above and returns true if the input state is not violating any of them.
    """
    return lambda state: not any([constraint(state.getX(), state.getY(), state.getZ()) for constraint in constraints])

def get_trajectory(constraints, start_state, goal_state, start_rotation=[0,0,0,1], goal_rotation=[0,0,0,1], min_bounds=-5, max_bounds=5, t=1.0):
    """
    Takes in a list of constraints, a start, a goal, bounds, and a maximum computation time, and returns a list of points that form a trajectory to the goal, or None if no such trajectory is found.
    """

    # Return a function that represents spatial constraints
    fn = valid_fn(constraints)

    space = ob.SE3StateSpace()
    bounds = ob.RealVectorBounds(3)

    # Set limits for the x, y, z axes
    bounds.setLow(min_bounds)
    bounds.setHigh(max_bounds)
    space.setBounds(bounds)
    ss = og.SimpleSetup(space)

    ss.setStateValidityChecker(ob.StateValidityCheckerFn(fn))
    
    # Create the start state
    start = ob.State(space)
    start.random()
    start[0] = start_state[0]
    start[1] = start_state[1]
    start[2] = start_state[2]
    start[3] = start_rotation[0]
    start[4] = start_rotation[1]
    start[5] = start_rotation[2]
    start[6] = start_rotation[3]
    # Create the goal state
    goal = ob.State(space)
    goal.random()
    goal[0] = goal_state[0]
    goal[1] = goal_state[1]
    goal[2] = goal_state[2]
    goal[3] = goal_rotation[0]
    goal[4] = goal_rotation[1]
    goal[5] = goal_rotation[2]
    goal[6] = goal_rotation[3]

    # Compute a trajectory using the default parameters
    ss.setStartAndGoalStates(start, goal)
    solved = ss.solve(t)

    if solved:
        ss.simplifySolution()
        path = ss.getSolutionPath()
        length = path.getStateCount()
        lst = []
        print "Solution found with length ", length
        for i in range(length):
            lst.append([path.getState(i).getX(), path.getState(i).getY(), path.getState(i).getZ(), path.getState(i).rotation().x, path.getState(i).rotation().y, path.getState(i).rotation().z, path.getState(i).rotation().w])
            print path.getState(i).rotation().x
        return np.matrix(lst)
    print "No solution found"
    return None

if __name__ == "__main__":
    b0 = block(1, 0.1, 1, 0.1, 1, 0.1)
    b1 = block(3, 2, 3, 2, 3, 2)
    b2 = axis_limit(4.5, True, 'x')
    b3 = axis_limit(-1, False, 'y')
    b4 = axis_limit(-2, False, 'z')
    b5 = sphere(2, 2, 2, 1)
    constraints = [b0, b1, b2, b3, b4, b5]

    print get_trajectory(constraints, [0,0,0], [4, 4, 4])
    
