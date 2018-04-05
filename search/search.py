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
    mapToGoal = {}
    currentState = problem.getStartState()
    visited = set()
    fringe = util.Stack()
    fringe.push(currentState)
    # print(currentState)
    while fringe:
        currentState = fringe.pop()
        # print(currentState)
        if problem.isGoalState(currentState):
            # print(mapToGoal)
            return directionsToGoal(currentState, mapToGoal)
        visited.add(currentState)
        for child in problem.getSuccessors(currentState):
            if child[0] not in visited:
                fringe.push(child[0])
                # print(child)
                mapToGoal[child[0]] = (child[1], currentState)
            # print("all successors")
    return "DID NOT WORK"

def directionsToGoal(target, map):
    directions = []
    directionTo = map[target][0]
    curr = map[target][1]
    # print(map)
    while curr in map:
        directions.append(directionTo)
        directionTo = map[curr][0]
        curr = map[curr][1]
    directions.append(directionTo)
    return list(reversed(directions))


    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    mapToGoal = {}
    currentState = problem.getStartState()
    visited = set()
    fringe = util.Queue()
    fringe.push(currentState)
    # print(fringe)
    # print(currentState)
    while fringe.isEmpty() == False:
        # print(fringe)
        currentState = fringe.pop()
        # print(currentState)
        if problem.isGoalState(currentState):
            # print(mapToGoal)
            return directionsToGoal(currentState, mapToGoal)
        visited.add(currentState)
        for child in problem.getSuccessors(currentState):
            if child[0] not in visited and child[0] not in mapToGoal:
                fringe.push(child[0])
                # print(child)
                mapToGoal[child[0]] = (child[1], currentState)
                # print(mapToGoal)
            # print("all successors")
    return "DID NOT WORK"

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    mapToGoal = {}
    distanceDictionary = {}
    currentState = problem.getStartState()
    visited = set()
    fringe = util.PriorityQueue()
    distanceDictionary[currentState] = 0
    fringe.push(currentState, 0)
    while fringe:
        currentState = fringe.pop()
        if problem.isGoalState(currentState):
            return directionsToGoal(currentState, mapToGoal)
        visited.add(currentState)
        for child in problem.getSuccessors(currentState):
            if child[0] not in visited:
                disFromStart = distanceDictionary[currentState] + child[2]
                fringe.update(child[0], disFromStart)
                if child[0] not in mapToGoal or distanceDictionary[child[0]] > disFromStart: 
                    mapToGoal[child[0]] = (child[1], currentState)
                    distanceDictionary[child[0]] = disFromStart
    return "DID NOT WORK"

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    mapToGoal = {}
    distanceDictionary = {}
    currentState = problem.getStartState()
    visited = set()
    fringe = util.PriorityQueue()
    distanceDictionary[currentState] = 0
    fringe.push(currentState, 0)
    while fringe:
        currentState = fringe.pop()
        if problem.isGoalState(currentState):
            return directionsToGoal(currentState, mapToGoal)
        visited.add(currentState)
        for child in problem.getSuccessors(currentState):
            if child[0] not in visited:
                disFromStart = distanceDictionary[currentState] + child[2]
                disWithHeuristic = disFromStart + heuristic(child[0], problem)
                fringe.update(child[0], disWithHeuristic)
                if child[0] not in mapToGoal or distanceDictionary[child[0]] > disFromStart: 
                    mapToGoal[child[0]] = (child[1], currentState)
                    distanceDictionary[child[0]] = disFromStart
    return "DID NOT WORK"


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
