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
    front = util.Stack()                #arxikopoiw to frontier san stoiva (LIFO)
    explored_states = []                #kani lista me eksereunimeno monopati
    path = []                           #keni lista gia to prwto path
    start_state = problem.getStartState()

    front.push((start_state, path))

    while not front.isEmpty():
        node = front.pop()              #vgazw to teleutaio stoixeio tou frontier
        path = node[1]                  #krataw to monopati mexri auto to stoixeio

        if problem.isGoalState(node[0]):    #elegxw n einai goal kai an nai epistefw to path
            return path

        explored_states.append(node[0])     #vazw to state pou exw perasei sta explored

        #successor=(state, action, cost)
        for successor in problem.getSuccessors(node[0]):  #gia kathe paidi tou stoixeiou
            if successor[0] not in explored_states:       #an den einai explored
                if successor[0] not in front.list:        #kai an den einai sto frontier
                    path = node[1] + [successor[1]]       #vazw ston successor to path + action tou
                    front.push((successor[0], path))      #vazw to paidi sto frontier me to monopati tou

    util.raiseNotDefined()


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    path = []
    start_state = problem.getStartState()

    if problem.isGoalState(start_state):        #elegxw an to start state einai goal
        return path

    front = util.Queue()                        #arxikopoiw to frontier san oura (FIFO)

    front.push((start_state, path, 0))
    explored_states = []

    while not front.isEmpty():

        node = front.pop()

        if node[0] not in explored_states:
            explored_states.append(node[0])
            for successor in problem.getSuccessors(node[0]):    #gia kathe paidi tou stoixeiou
                if successor[0] not in explored_states:         #an den einai explored
                    if successor[0] not in front.list:          #kai an den einai sto frontier
                        path = node[1] + [successor[1]]         #vazw ston successor to path + action tou
                        if problem.isGoalState(successor[0]):   #elegxw an to paidi einai goal kai an nai to epistrefw
                            return path
                        front.push((successor[0], path, successor[2]))   #vazw to paidi sto frontier me to monopati tou
    return -1


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    start_state = problem.getStartState()
    front = util.PriorityQueue()            #arxikopoiw frontier me priorityQueue
    front.push((start_state, [], 0), 0)     #vazw to prwto node me priority 0

    explored_states = []

    while not front.isEmpty():
        node = front.pop()

        if problem.isGoalState(node[0]):
            return node[1]

        if node[0] not in explored_states:
            explored_states.append(node[0])

            for successor in problem.getSuccessors(node[0]):
                if successor[0] not in explored_states:     #an den einai explored
                    path = node[1] + [successor[1]]         #vazw ston successor to path + action tou
                    path_cost = node[2] + successor[2]      #auksanw to path_cost gia na to dwsw san priority
                    if successor[0] not in front.heap:
                        front.push((successor[0], path, path_cost), path_cost)
                    else:
                        front.update((successor[0], path, path_cost), path_cost)    #kanw update me to neo cost an xreiastei

    util.raiseNotDefined()


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""

    def HandG((state, path, path_cost)):        #sunartisi sumfwna me thn opoia tha taksinomountai ta nodes
        h = heuristic(state, problem)
        g = problem.getCostOfActions(path)
        return h+g

    start_state = problem.getStartState()

    front = util.PriorityQueueWithFunction(HandG)   #vazei ta nodes se auksousa seira sumfwna me thn f(x) = h(x) + g(x)
    front.push((start_state, [], 0))

    explored_states = []

    while not front.isEmpty():          #kanei graph search
        node = front.pop()

        if problem.isGoalState(node[0]):
            return node[1]

        if node[0] not in explored_states:
            explored_states.append(node[0])

            for successor in problem.getSuccessors(node[0]):
                if successor[0] not in explored_states:     #an den einai explored
                    path = node[1] + [successor[1]]         #vazw ston successor to path + action tou
                    path_cost = node[2] + successor[2]      #auksanw to path_cost gia na to dwsw san priority

                    if successor[0] not in front.heap:
                        front.push((successor[0], path, path_cost))

    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
