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
    return graphSearch(problem, 'DFS')
    
    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    return graphSearch(problem, 'BFS')
    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    return graphSearch(problem, 'UCS')
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    return graphSearch(problem, 'ASTAR', heuristic)
    util.raiseNotDefined()


def graphSearch(problem, strategy, heuristic=nullHeuristic):
    """ Search in a graph code """
    
    #print "Start:", problem.getStartState()
    #print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    #print "Start's successors:", problem.getSuccessors(problem.getStartState())

    # closed <- an empty set
    closedNodes = set()
    # closedNodes.add((5,5))
    # print closedNodes
    # print type(closedNodes)
    # print dir(closedNodes)

    #util.pause()
    
    # List that contains the moves that are required reach the goal
    move_list = []

    # strategies according to the algorithms
    if strategy == 'DFS':
        fringe = util.Stack()
    elif strategy == 'BFS':
        fringe = util.Queue()
    elif strategy == 'UCS':
        fringe = util.PriorityQueue()
    elif strategy == 'ASTAR':
        fringe = util.PriorityQueue()


    # Initial cost is 0 because we have not moved
    # start fringe with initial problem state, movements, cumulative cost
    if (strategy is not 'UCS') and (strategy is not 'ASTAR'):
        fringe.push([problem.getStartState(),move_list, 0])
    else:
        fringe.push([problem.getStartState(),move_list, 0], 0)

    # loop do
    while (True):
        # if fringe is empty, return failure
        if fringe.isEmpty():
     #       print 'Fail'
            return None
        
        # node <-- get_node(strategy, fringe)
        node = fringe.pop()
        # print node[0] State
        # print node[1] move_list to get there
        # print node[2] cummulative cost
        #print "Successors:", problem.getSuccessors(node[0])
        #util.pause()

        # if state(node) is goal, return node
        if problem.isGoalState(node[0]): 
            move_list = node[1][:]
      #      print move_list
       #     print
            return move_list
        
        #print
        #print node[0]
        #print type(node[0])
        #print 'oka'
        
        # if state(node) is not in closed
        if ( node[0] not in closedNodes):
            # add state(node) to closed
            closedNodes.add(node[0])
            # for child-node in expand(node) add child node to fringe
            childNodes = problem.getSuccessors(node[0])
            for i in range(len(childNodes)):
                state = childNodes[i][0]
                move_list = node[1][:]
                # print childNodes[i]
                # print state
                # print i
                # print move_list
                # print 
                move_list.append(childNodes[i][1])
                
                cummulativeCost = node[2] + childNodes[i][2]
                if (strategy is not 'UCS') and (strategy is not 'ASTAR'):
                    fringe.push([state,move_list, cummulativeCost])
                else:
                    totalCost = cummulativeCost
                    if strategy is 'ASTAR':
                        totalCost +=  heuristic(state, problem)
                    fringe.push([state,move_list, cummulativeCost], totalCost)
    
    # You should not get here anyway
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
