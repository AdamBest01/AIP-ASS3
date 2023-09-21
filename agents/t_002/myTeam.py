from captureAgents import CaptureAgent
import random, time, util, game
from game import Directions, Actions
from util import manhattanDistance
from util import Queue

def createTeam(firstIndex, secondIndex, isRed, first='OffensiveAgent', second='DefensiveAgent'):
    return [eval(first)(firstIndex), eval(second)(secondIndex)]

class BaseAgent(CaptureAgent):
    
    current_targets = dict()
    def registerInitialState(self, gameState):
        CaptureAgent.registerInitialState(self, gameState)
        self.start = gameState.getAgentPosition(self.index)
        self.carrying = 0
        self.current_target = None
        self.boundary = self.getBoundary(gameState)
        self.maxCapacity = 6
        self.othersLocation = next(iter(set(self.getTeam(gameState)) - {self.index}))
        
    def determineCurrentTarget(self, gameState):
        foodGrid = self.getFood(gameState)
        foodList = foodGrid.asList()
        Positions = list(filter(lambda pos: pos not in BaseAgent.current_targets.values(), foodList))
        
        if self.carrying >= self.maxCapacity or len(foodList) <= 2:
            self.current_target = self.getClosestPos(gameState, self.boundary)
        else:
            enemy_positions = [gameState.getAgentPosition(i) for i in self.getOpponents(gameState) if gameState.getAgentPosition(i) is not None]
            
            # Step 1: Consider food density
            food_density = {pos: sum(1 for other_pos in Positions if self.getMazeDistance(pos, other_pos) <= 2) for pos in Positions}
            
            # Step 2: Consider safety (distance from enemies)
            safe_positions = list(filter(lambda pos: all(self.getMazeDistance(pos, enemy_pos) > 5 for enemy_pos in enemy_positions), Positions))
            
            # Step 3: Consider distance from current position
            my_pos = gameState.getAgentState(self.index).getPosition()
            dist_from_current_pos = {pos: self.getMazeDistance(my_pos, pos) for pos in Positions}

            # Composite score based on the factors
            score = lambda pos: food_density.get(pos, 0) - 0.5 * dist_from_current_pos.get(pos, 0)
            
            # Find the position with the highest score
            if safe_positions:
                self.current_target = max(safe_positions, key=score)
            else:
                self.current_target = max(Positions, key=score)


    def chooseAction(self, gameState):
        if self.current_target is None:
            self.determineCurrentTarget(gameState)

        BaseAgent.current_targets[self.index] = self.current_target

        problem = PositionSearchProblem(gameState, self.current_target, self.index)
        path = self.bfsSearch(problem)
        
        if not path:
            actions = gameState.getLegalActions(self.index)
            return random.choice(actions)
        else:
            action = path[0]
            dx, dy = Actions.directionToVector(action)
            x, y = gameState.getAgentState(self.index).getPosition()
            new_x, new_y = int(x + dx), int(y + dy)
            
            if (new_x, new_y) == self.current_target:
                self.current_target = None
                BaseAgent.current_targets.pop(self.index, None)

            if self.getFood(gameState)[new_x][new_y]:
                self.carrying += 1
            elif (new_x, new_y) in self.boundary:
                self.carrying = 0

            return path[0]

    
    def getClosestPos(self,gameState,pos_list):
        if not pos_list:
            return None 
        min_length = 9999
        min_pos = None
        my_local_state = gameState.getAgentState(self.index)
        my_pos = my_local_state.getPosition()
        for pos in pos_list:
            temp_length = self.getMazeDistance(my_pos,pos)
            if temp_length < min_length:
                min_length = temp_length
                min_pos = pos
        return min_pos
  
    def getBoundary(self,gameState):
        boundary_location = []
        height = gameState.data.layout.height
        width = gameState.data.layout.width
        for i in range(height):
            if self.red:
                j = int(width/2)-1
            else:
                j = int(width/2)
            if not gameState.hasWall(j,i):
                boundary_location.append((j,i))
        return boundary_location
    
    def bfsSearch(self, problem):
    

        myQueue = util.Queue()
        startState = problem.getStartState()
        startNode = (startState, '', 0, [])
        myQueue.push(startNode)
        visited = set()

        while not myQueue.isEmpty():
            node = myQueue.pop()
            state, action, cost, path = node

            if state in visited:
                continue
            visited.add(state)

            if problem.isGoalState(state):
                path = path + [(state, action)]
                actions = [action[1] for action in path]
                del actions[0]
                return actions

            for succ in problem.getSuccessors(state):
                succState, succAction, succCost = succ
                newNode = (succState, succAction, cost + succCost, path + [(node, action)])
                myQueue.push(newNode)

        return []


class PositionSearchProblem:
    
    def __init__(self, gameState, goal, agentIndex = 0,costFn = lambda x: 1):
        self.walls = gameState.getWalls()
        self.costFn = costFn
        x,y = gameState.getAgentState(agentIndex).getPosition()
        self.startState = int(x),int(y)
        self.goal_pos = goal

    def getStartState(self):
      return self.startState

    def isGoalState(self, state):

      return state == self.goal_pos

    def getSuccessors(self, state):
        successors = []
        for action in [game.Directions.NORTH, game.Directions.SOUTH, game.Directions.EAST, game.Directions.WEST]:
            x,y = state
            dx, dy = game.Actions.directionToVector(action)
            nextx, nexty = int(x + dx), int(y + dy)
            if not self.walls[nextx][nexty]:
                nextState = (nextx, nexty)
                cost = self.costFn(nextState)
                successors.append( ( nextState, action, cost) )
        return successors

    def getCostOfActions(self, actions):
        if actions == None: return 999999
        x,y= self.getStartState()
        cost = 0
        for action in actions:
            # Check figure out the next state and see whether its' legal
            dx, dy = game.Actions.directionToVector(action)
            x, y = int(x + dx), int(y + dy)
            if self.walls[x][y]: return 999999
            cost += self.costFn((x,y))
        return cost
      
    def _manhattanDistance(self, pos):
        return manhattanDistance(pos, self.goal_pos)


class OffensiveAgent(BaseAgent):  
    pass

class DefensiveAgent(BaseAgent):

    def registerInitialState(self, gameState):
        BaseAgent.registerInitialState(self, gameState)
        self.defensiveMode = False

    def chooseAction(self, gameState):
        # Get the location of the enemy and the location of the food
        enemy_positions = [gameState.getAgentPosition(i) for i in self.getOpponents(gameState) if gameState.getAgentPosition(i) is not None]
        
        # Get own location
        my_pos = gameState.getAgentState(self.index).getPosition()
        
        # If an enemy is approaching (e.g., distance <= 6), switch to defence mode
        if any(self.getMazeDistance(my_pos, enemy_pos) <= 6 for enemy_pos in enemy_positions):
            self.defensiveMode = True
        else:
            self.defensiveMode = False

        # If in defence mode
        if self.defensiveMode:
            # Choose an approaching enemy as a target and try to track and intercept it!
            closest_enemy_pos = min(enemy_positions, key=lambda x: self.getMazeDistance(my_pos, x))
            self.current_target = closest_enemy_pos
        else:
            # If no enemies are approaching, the agent can perform its regular target selection logic
            pass

        return BaseAgent.chooseAction(self, gameState)

