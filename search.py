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
    """
    #print("Start:", problem.getStartState())
    #print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    #print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    
    "*** YOUR CODE HERE ***"
    #print(problem)
    from util import Stack
    height=problem.walls.height
    width=problem.walls.width
    visited = [[0 for y in range(height)] for x in range(width)]
    pos=problem.getStartState()
    visited[pos[0]][pos[1]]=1
    paths=[]
    positions=Stack()
    positions.push(pos)
    while True:
        pos=positions.list[-1]
        if problem.isGoalState(pos):
            return paths
        suc=problem.getSuccessors(pos)
        flag=0
        for i in suc:
            if visited[i[0][0]][i[0][1]] == 0:
                newpos=i[0]
                visited[newpos[0]][newpos[1]] = 1
                paths.append(i[1])
                positions.push(newpos)
                flag=1
                break
        if flag==0:
            if len(paths)==0:
                return []
            paths.pop()
            positions.pop()  

    return []
    #util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    from util import Queue
    positions=Queue()
    pos=problem.getStartState()
    positions.push(pos)
    visited=[]
    paths=Queue()
    paths.push([])
    while True:
        pos=positions.pop()
        visited.append(pos)
        path=paths.pop()
        #print(pos,len(path))
        if problem.isGoalState(pos):
            return path
        suc=problem.getSuccessors(pos)
        
        for i in suc:
            newpos=i[0]
            if not newpos in visited:
                path2=path.copy()
                path2.append(i[1])
                positions.push(newpos)
                paths.push(path2)

        if positions.isEmpty():
            break

    return []    
    #util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"

    # Implemented a slightly different priority queue
    # to enable an output indicating
    # whether the update is successful.
    # It can make path-output-ing very convenient.
    import heapq
    class PriorityQueue2:

        def  __init__(self):
            self.heap = []
            self.count = 0

        def push(self, item, priority):
            entry = (priority, self.count, item)
            heapq.heappush(self.heap, entry)
            self.count += 1

        def pop(self):
            (_, _, item) = heapq.heappop(self.heap)
            return item

        def isEmpty(self):
            return len(self.heap) == 0

        def update(self, item, priority):
            # If item already in priority queue with higher priority, update its priority and rebuild the heap.
            # If item already in priority queue with equal or lower priority, do nothing.
            # If item not in priority queue, do the same thing as self.push.
            for index, (p, c, i) in enumerate(self.heap):
                if i == item:
                    if p <= priority:
                        return 0
                    del self.heap[index]
                    self.heap.append((priority, c, item))
                    heapq.heapify(self.heap)
                    return 1
            else:
                self.push(item, priority)
                return -1
            
    positions=PriorityQueue2()
    pos=problem.getStartState()
    positions.push(pos,0)
    last={pos:[]}
    visited=[]
    while True:
        pos=positions.heap[0][2]
        cost=positions.heap[0][0]
        positions.pop()
        visited.append(pos)
        
        if problem.isGoalState(pos):
            path=[]
            while last[pos]!=[]:
                path=[last[pos][1]]+path
                pos=last[pos][0]
            return path
        suc=problem.getSuccessors(pos)
        
        for i in suc:
            newpos=i[0]
            if not newpos in visited:
                upd=positions.update(newpos,cost+i[2])
                if upd!=0:
                    last[newpos]=[pos,i[1]]

        if positions.isEmpty():
            break

    return []   
    #util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
