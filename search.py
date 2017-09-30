# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    #print("Start:", problem.getStartState())
    #print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    #print("Start's successors:", problem.getSuccessors(problem.getStartState()))

    from util import Stack
    s = Stack()
    current = problem.getStartState()
    s.push(current)
    visited = set()
    parent = {}
    direction = {}
    parent[current] = -1


    while(not s.isEmpty()):
        #s.pop(), if this is goal, break and return path
        current = s.pop()
        if(problem.isGoalState(current)):
            break

        visited.add(current)
        #for each succesor of current, if it is not visted, add in the stack
        for adj,dir,cost in problem.getSuccessors(current):
            if(not adj in visited):
                s.push(adj)
                parent[adj] = current
                direction[adj] = dir

    #path = trace_path(parent, current)
    #return getPathDirections(path)
    return trace_direction(parent, direction, current)

def trace_direction(parent, direction, goalState):
    current = goalState
    path = []
    i=0
    while(not parent[current] == -1):
        path.append(direction[current])
        i = i+1
        current = parent[current]
    path.reverse()
    return path

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    from util import Queue
    q = Queue()
    current_state = problem.getStartState()
    q.push(current_state)
    visited = set()
    state_path_dict = {}
    state_path_dict[current_state] = []

    while( not q.isEmpty()):
        current_state = q.pop();
        if(problem.isGoalState(current_state)):
            break

        if current_state not in visited:
            visited.add(current_state)
            for adj,dir,cost in problem.getSuccessors(current_state):
                q.push(adj)
                if adj not in state_path_dict:
                    path = state_path_dict[current_state]
                    new_path = list(path)
                    new_path.append(dir)
                    state_path_dict[adj] = new_path


    """from util import Queue
    s = Queue()
    current = problem.getStartState()
    s.push(current)
    visited = set()
    parent = {}
    direction = {}
    parent[current] = -1
    visited.add(current)
    while(not s.isEmpty()):
        #s.pop(), if this is goal, break and return path
        current = s.pop()
        if(problem.isGoalState(current)):
            break
        #for each succesor of current, if it is not visted, add in the stack
        for adj,dir,cost in problem.getSuccessors(current):
            if(not adj in visited):
                s.push(adj)
                parent[adj] = current
                direction[adj] = dir
                visited.add(adj)
    return trace_direction(parent, direction, current)"""

    return state_path_dict[current_state]

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    from util import PriorityQueue
    s = PriorityQueue()
    current = problem.getStartState()
    s.push(current,0)
    visited = set()
    parent = {}
    direction = {}
    parent[current] = -1
    visited.add(current)
    costDict = {}
    costDict[current] = 0.0

    while(not s.isEmpty()):
        #s.pop(), if this is goal, break and return path
        current = s.pop()
        cost_till_now = costDict[current];
        if(problem.isGoalState(current)):
            break
        #for each succesor of current, if it is not visted, add in the stack
        for adj,dir,cost in problem.getSuccessors(current):
            if(not adj in visited):
                s.push(adj, cost_till_now + cost)
                costDict[adj] = cost_till_now + cost
                parent[adj] = current
                direction[adj] = dir
                visited.add(adj)
            else:
                if(costDict[adj] > cost_till_now + cost):
                    s.update(adj, cost_till_now + cost)
                    costDict[adj] = cost_till_now + cost
                    parent[adj] = current
                    direction[adj] = dir
    return trace_direction(parent, direction, current)

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    from util import PriorityQueue
    s = PriorityQueue()
    current = problem.getStartState()
    s.push(current,0)
    visited = set()
    parent = {}
    direction = {}
    parent[current] = -1
    visited.add(current)
    costDict = {}
    costDict[current] = 0.0

    while(not s.isEmpty()):
        #s.pop(), if this is goal, break and return path
        current = s.pop()
        cost_till_now = costDict[current];
        if(problem.isGoalState(current)):
            break
        #for each succesor of current, if it is not visted, add in the stack
        for adj,dir,cost in problem.getSuccessors(current):
            Hcost = cost + heuristic(adj,problem)
            if(not adj in visited):
                s.push(adj, cost_till_now + Hcost)
                costDict[adj] = cost_till_now + cost
                parent[adj] = current
                direction[adj] = dir
                visited.add(adj)
            else:
                if(costDict[adj] > cost_till_now + cost):
                    s.update(adj, cost_till_now + cost)
                    costDict[adj] = cost_till_now + cost
                    parent[adj] = current
                    direction[adj] = dir
    return trace_direction(parent, direction, current)


# def aStarSearch(problem, heuristic=nullHeuristic):
#     """Search the node that has the lowest combined cost and heuristic first."""
#     "*** YOUR CODE HERE ***"
#     from util import PriorityQueue
#     s = PriorityQueue()
#     s.push(problem.getStartState(), heuristic(problem.getStartState(),problem))
#     visited = set()
#     parent = {}
#     direction = {}
#     parent[problem.getStartState()] = -1
#     current = problem.getStartState()
#
#     while(not s.isEmpty()):
#         #s.pop(), if this is goal, break and return path
#         current = s.pop()
#         visited.add(current)
#         if(problem.isGoalState(current)):
#             break
#         #for each succesor of current, if it is not visted, add in the stack
#         for adj,dir,cost in problem.getSuccessors(current):
#             if(not adj in visited):
#                 s.push(adj, heuristic(adj,problem))
#                 parent[adj] = current
#                 direction[adj] = dir
#
#     return trace_direction(parent, direction, current)

def trace_path(parent, goalState):
    current = goalState
    path = []
    i=0
    while(not parent[current] == -1):
        path.append(current)
        i = i+1
        current = parent[current]
    path.append(current)
    path.reverse()
    return path

def getPathDirections(path):
    dirctions = []
    for i in range(len(path)-1):
        dirctions.append(getDirection(path[i], path[i+1]))
    return dirctions

def getDirection(fromNode, toNode):
    from game import Directions
    xdiff = int(fromNode[0]) - int(toNode[0])
    if(xdiff == 1):
        return Directions.WEST
    if(xdiff == -1):
        return Directions.EAST
    xdiff = int(fromNode[1]) - int(toNode[1])
    if(xdiff == 1):
        return Directions.SOUTH
    if(xdiff == -1):
        return Directions.NORTH
    return 0


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
