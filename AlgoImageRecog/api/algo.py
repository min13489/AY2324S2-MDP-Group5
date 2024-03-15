import heapq
import numpy as np
from python_tsp.exact import solve_tsp_dynamic_programming

# variables for turning radius
fwd_l_row = 3   # 30 40 increment for FL
fwd_l_col = 4
fwd_r_row = 2   # 20 30 increment for FR
fwd_r_col = 3
bwd_l_row = 2   # 30 20 increment for BL
bwd_l_col = 2
bwd_r_row = 3   # 30 20 increment for BR
bwd_r_col = 2

# initialise map

map = []
for i in range(21):
    map.append([])
    for j in range(21):
        map[len(map)-1].append("_")

# function called by API

def generatePath(robotPos, obs):

    # insert robot
    robotPos = tuple(robotPos)

    for x in range(robotPos[1]-1, robotPos[1]+2):
        for y in range(robotPos[0]-1, robotPos[0]+2):
            map[x][y] = "R"

    # insert obstacles and danger zone
    markObs(obs)
    goalCentres = getGoalCentres(obs)
    failedGoals = markGoals(goalCentres.keys())

    # for debug visualisation
    visualiseMap()

    states = []
    states.append(robotPos)
    # if goal area does not overlap with obstacle, attempt to go
    for goal in goalCentres.keys():
        if goal not in failedGoals:
            states.append(goal)

    pathTable, costTable = getPathAndCost(states)

    # dynamic programming to determine best path
    costMatrix = np.zeros((len(states), len(states)))
    for start in range(len(states)-1):
        for end in range(start+1, len(states)):
            if (states[start],states[end]) in costTable.keys():
                costMatrix[start][end] = costTable[(states[start],states[end])]
            else:
                costMatrix[start][end] = 1e9
            costMatrix[end][start] = costMatrix[start][end]
    costMatrix[:,0] = 0
    print(costMatrix)
    sequence, distance = solve_tsp_dynamic_programming(costMatrix)
    print(sequence)

    # states pathing
    optimalPath = []
    for i in range(len(sequence)-1):
        fromState = states[sequence[i]]
        toState = states[sequence[i+1]]
        for state in pathTable[(fromState, toState)]:
            optimalPath.append(state)

    # states to commands
    commands = commandify(optimalPath, goalCentres)
    print(optimalPath)
    states = scondenser(optimalPath)

    # reset map
    resetMap()

    # return ["FW01", "BW01", "FL00", "BL00", "FR00", "BR00", "FW03", "BW02", "TP01"]
    # return ["FW06", "TP01", "BW01", "FR00", "FW10", "TP02", "BW03", "FL00", "FW04", "TP03", "BL00", "BR00", "BW01", "BL00", "BW01", "FR00", "TP04"]
    return commands, states

# path and cost related

def getPathAndCost(states):

    pathTable = dict()
    costTable = dict()

    def recordPathAndCost(start, end, parent, cost):
        costTable[(start, end)] = cost
        costTable[(end, start)] = cost

        path = []
        pointer = end

        while pointer in parent:
            path.append(pointer)
            pointer = parent[pointer]
        
        path.append(pointer) # start

        pathTable[(start,end)] = path[::-1]
        pathTable[(end,start)] = path

    def aStarSearch(start, end):

        def greedyDist(start, end):
            rowDist = start[0] - end[0]
            colDist = start[1] - end[1]

            return abs(rowDist) + abs(colDist)

        def getLegalMoves(curPos):

            def isLegalAndSafe(curPos, pos):
                
                # TODO: turning squares involved - need to know turning radius / 3pt 5pt 
                unsafeSquares = 0

                # check final position
                for i in range(pos[1]-1, pos[1]+2):
                    for j in range(pos[0]-1, pos[0]+2):
                        # legal: out of map
                        if i <= 0 or j <= 0 or i >= 21 or j >= 21:
                            return -1
                        # legal: overlap with an obstacle
                        if map[i][j] == "X":
                            return -1
                        # safe: within 10cm square boundary of obstacle
                        if map[i][j] == "D":
                            unsafeSquares += 1
                
                # check otw - for turns
                
                if curPos[2] != pos[2]: # if it is a turn
                    if curPos[2] == 'N':
                        if pos[2] == 'W':
                            if curPos[0] < pos[0]: # FL
                                for i in range(pos[1]+2, curPos[1]+2): # right-most of pos + 1, right-most of cur
                                    for j in range(curPos[0]+2, pos[0]+2): # top-most of cur + 1, top-most of pos
                                        if i <= 0 or j <= 0 or i >= 21 or j >= 21:
                                            return -1
                                        # legal: overlap with an obstacle
                                        if map[i][j] == "X":
                                            return -1
                                        # safe: within 10cm square boundary of obstacle
                                        if map[i][j] == "D":
                                            unsafeSquares += 1
                            else: # BR
                                for i in range(curPos[1]-1, pos[1]-1): # left-most of cur, left-most of pos - 1
                                    for j in range(pos[0]-1, curPos[0]-1): # bottom-most of cur + 1, bottom-most of pos
                                        if i <= 0 or j <= 0 or i >= 21 or j >= 21:
                                            return -1
                                        # legal: overlap with an obstacle
                                        if map[i][j] == "X":
                                            return -1
                                        # safe: within 10cm square boundary of obstacle
                                        if map[i][j] == "D":
                                            unsafeSquares += 1
                        else: # east
                            if curPos[0] < pos[0]: # FR
                                for i in range(curPos[1]-1, pos[1]-1): # left-most of cur, left-most of pos - 1
                                    for j in range(curPos[0]+2, pos[0]+2): # top-most of cur + 1, top-most of pos
                                        if i <= 0 or j <= 0 or i >= 21 or j >= 21:
                                            return -1
                                        # legal: overlap with an obstacle
                                        if map[i][j] == "X":
                                            return -1
                                        # safe: within 10cm square boundary of obstacle
                                        if map[i][j] == "D":
                                            unsafeSquares += 1
                            else: # BL
                                for i in range(pos[1]+2, curPos[1]+2): # right-most of pos + 1, right-most of cur
                                    for j in range(pos[0]-1, curPos[0]-1): # top-most of cur + 1, top-most of pos
                                        if i <= 0 or j <= 0 or i >= 21 or j >= 21:
                                            return -1
                                        # legal: overlap with an obstacle
                                        if map[i][j] == "X":
                                            return -1
                                        # safe: within 10cm square boundary of obstacle
                                        if map[i][j] == "D":
                                            unsafeSquares += 1
                    elif curPos[2] == 'S':
                        if pos[2] == 'E':
                            if curPos[0] > pos[0]: # FL
                                for i in range(curPos[1]-1, pos[1]-1): # left-most of curPos to left-most of pos - 1
                                    for j in range(pos[0]-1, curPos[0]-1): # bot-most of pos to bot-most of curPos - 1
                                        if i <= 0 or j <= 0 or i >= 21 or j >= 21:
                                            return -1
                                        # legal: overlap with an obstacle
                                        if map[i][j] == "X":
                                            return -1
                                        # safe: within 10cm square boundary of obstacle
                                        if map[i][j] == "D":
                                            unsafeSquares += 1
                            else: # BR
                                for i in range(pos[1]+2, curPos[1]+2): # right-most of pos + 1 to right-most of curPos
                                    for j in range(curPos[0]+2, pos[0]+2): # top-most of curPos + 1 to top-most of pos
                                        if i <= 0 or j <= 0 or i >= 21 or j >= 21:
                                            return -1
                                        # legal: overlap with an obstacle
                                        if map[i][j] == "X":
                                            return -1
                                        # safe: within 10cm square boundary of obstacle
                                        if map[i][j] == "D":
                                            unsafeSquares += 1
                        else: # west
                            if curPos[0] > pos[0]: # FR
                                for i in range(pos[1]+2, curPos[1]+2): # right-most of pos + 1, right-most of curPos
                                    for j in range(pos[0]-1, curPos[0]-1): # bot-most of pos to bot-most of curPos - 1
                                        if i <= 0 or j <= 0 or i >= 21 or j >= 21:
                                            return -1
                                        # legal: overlap with an obstacle
                                        if map[i][j] == "X":
                                            return -1
                                        # safe: within 10cm square boundary of obstacle
                                        if map[i][j] == "D":
                                            unsafeSquares += 1
                            else: # BL
                                for i in range(curPos[1]-1, pos[1]-1): # right-most of pos + 1, right-most of cur
                                    for j in range(curPos[0]+2, pos[0]+2): # top-most of cur + 1, top-most of pos
                                        if i <= 0 or j <= 0 or i >= 21 or j >= 21:
                                            return -1
                                        # legal: overlap with an obstacle
                                        if map[i][j] == "X":
                                            return -1
                                        # safe: within 10cm square boundary of obstacle
                                        if map[i][j] == "D":
                                            unsafeSquares += 1
                    elif curPos[2] == 'W':
                        if pos[2] == 'S':
                            if curPos[1] > pos[1]: # FL
                                for i in range(pos[1]-1, curPos[1]-1): # right-most of pos + 1, right-most of cur
                                    for j in range(pos[0]+2, curPos[0]+2): # top-most of cur + 1, top-most of pos
                                        if i <= 0 or j <= 0 or i >= 21 or j >= 21:
                                            return -1
                                        # legal: overlap with an obstacle
                                        if map[i][j] == "X":
                                            return -1
                                        # safe: within 10cm square boundary of obstacle
                                        if map[i][j] == "D":
                                            unsafeSquares += 1
                            else: # BR
                                for i in range(curPos[1]+2, pos[1]+2): # left-most of cur, left-most of pos - 1
                                    for j in range(curPos[0]-1, pos[0]-1): # bottom-most of cur + 1, bottom-most of pos
                                        if i <= 0 or j <= 0 or i >= 21 or j >= 21:
                                            return -1
                                        # legal: overlap with an obstacle
                                        if map[i][j] == "X":
                                            return -1
                                        # safe: within 10cm square boundary of obstacle
                                        if map[i][j] == "D":
                                            unsafeSquares += 1
                        else: # north
                            if curPos[1] > pos[1]: # FR
                                for i in range(pos[1]-1, curPos[1]-1): # left-most of cur, left-most of pos - 1
                                    for j in range(curPos[0]-1, pos[0]-1): # top-most of cur + 1, top-most of pos
                                        if i <= 0 or j <= 0 or i >= 21 or j >= 21:
                                            return -1
                                        # legal: overlap with an obstacle
                                        if map[i][j] == "X":
                                            return -1
                                        # safe: within 10cm square boundary of obstacle
                                        if map[i][j] == "D":
                                            unsafeSquares += 1
                            else: # BL
                                for i in range(curPos[1]+2, pos[1]+2): # right-most of pos + 1, right-most of cur
                                    for j in range(pos[0]+2, curPos[0]+2): # top-most of cur + 1, top-most of pos
                                        if i <= 0 or j <= 0 or i >= 21 or j >= 21:
                                            return -1
                                        # legal: overlap with an obstacle
                                        if map[i][j] == "X":
                                            return -1
                                        # safe: within 10cm square boundary of obstacle
                                        if map[i][j] == "D":
                                            unsafeSquares += 1
                    else: # east
                        if pos[2] == 'N':
                            if curPos[1] < pos[1]: # FL
                                for i in range(curPos[1]+2, pos[1]+2): # right-most of pos + 1, right-most of cur
                                    for j in range(curPos[0]-1, pos[0]-1): # top-most of cur + 1, top-most of pos
                                        if i <= 0 or j <= 0 or i >= 21 or j >= 21:
                                            return -1
                                        # legal: overlap with an obstacle
                                        if map[i][j] == "X":
                                            return -1
                                        # safe: within 10cm square boundary of obstacle
                                        if map[i][j] == "D":
                                            unsafeSquares += 1
                            else: # BR
                                for i in range(pos[1]-1, curPos[1]-1): # left-most of cur, left-most of pos - 1
                                    for j in range(pos[0]+2, curPos[0]+2): # bottom-most of cur + 1, bottom-most of pos
                                        if i <= 0 or j <= 0 or i >= 21 or j >= 21:
                                            return -1
                                        # legal: overlap with an obstacle
                                        if map[i][j] == "X":
                                            return -1
                                        # safe: within 10cm square boundary of obstacle
                                        if map[i][j] == "D":
                                            unsafeSquares += 1
                        else: # south
                            if curPos[1] < pos[1]: # FR
                                for i in range(curPos[1]+2, pos[1]+2): # left-most of cur, left-most of pos - 1
                                    for j in range(pos[0]+2, curPos[0]+2): # top-most of cur + 1, top-most of pos
                                        if i <= 0 or j <= 0 or i >= 21 or j >= 21:
                                            return -1
                                        # legal: overlap with an obstacle
                                        if map[i][j] == "X":
                                            return -1
                                        # safe: within 10cm square boundary of obstacle
                                        if map[i][j] == "D":
                                            unsafeSquares += 1
                            else: # BL
                                for i in range(pos[1]-1, curPos[1]-1): # right-most of pos + 1, right-most of cur
                                    for j in range(curPos[0]-1, pos[0]-1): # top-most of cur + 1, top-most of pos
                                        if i <= 0 or j <= 0 or i >= 21 or j >= 21:
                                            return -1
                                        # legal: overlap with an obstacle
                                        if map[i][j] == "X":
                                            return -1
                                        # safe: within 10cm square boundary of obstacle
                                        if map[i][j] == "D":
                                            unsafeSquares += 1
                
                if not unsafeSquares:
                    return 0                        # no unsafe cost
                else:
                    return unsafeSquares * 1000     # 1000 per unsafe square involved

            legalMoves = []
            newPos = []

            if curPos[2] == 'N':
                newPos.append((curPos[0]+1, curPos[1], curPos[2])) # fw
                newPos.append((curPos[0]-1, curPos[1], curPos[2])) # bw
                newPos.append((curPos[0]+fwd_l_row, curPos[1]-fwd_l_col, 'W')) # fl
                newPos.append((curPos[0]-bwd_l_row, curPos[1]-bwd_l_col, 'E')) # bl
                newPos.append((curPos[0]+fwd_r_row, curPos[1]+fwd_r_col, 'E')) # fr
                newPos.append((curPos[0]-bwd_r_row, curPos[1]+bwd_r_col, 'W')) # br
            elif curPos[2] == "S":
                newPos.append((curPos[0]-1, curPos[1], curPos[2])) # fw
                newPos.append((curPos[0]+1, curPos[1], curPos[2])) # bw
                newPos.append((curPos[0]-fwd_l_row, curPos[1]+fwd_l_col, 'E')) # fl
                newPos.append((curPos[0]+bwd_l_row, curPos[1]+bwd_l_col, 'W')) # bl
                newPos.append((curPos[0]-fwd_r_row, curPos[1]-fwd_r_col, 'W')) # fr
                newPos.append((curPos[0]+bwd_r_row, curPos[1]-bwd_r_col, 'E')) # br

            # changes in row and col swapped for E and W facing
            elif curPos[2] == "E":
                newPos.append((curPos[0], curPos[1]+1, curPos[2])) # fw
                newPos.append((curPos[0], curPos[1]-1, curPos[2])) # bw
                newPos.append((curPos[0]+fwd_l_col, curPos[1]+fwd_l_row, 'N')) # fl
                newPos.append((curPos[0]+bwd_l_col, curPos[1]-bwd_l_row, 'S')) # bl
                newPos.append((curPos[0]-fwd_r_col, curPos[1]+fwd_r_row, 'S')) # fr
                newPos.append((curPos[0]-bwd_r_col, curPos[1]-bwd_r_row, 'N')) # br
            elif curPos[2] == "W":
                newPos.append((curPos[0], curPos[1]-1, curPos[2])) # fw
                newPos.append((curPos[0], curPos[1]+1, curPos[2])) # bw
                newPos.append((curPos[0]-fwd_l_col, curPos[1]-fwd_l_row, 'S')) # fl
                newPos.append((curPos[0]-bwd_l_col, curPos[1]+bwd_l_row, 'N')) # bl
                newPos.append((curPos[0]+fwd_r_col, curPos[1]-fwd_r_row, 'N')) # fr
                newPos.append((curPos[0]+bwd_r_col, curPos[1]+bwd_r_row, 'S')) # br

            for pos in newPos:
                safeCost = isLegalAndSafe(curPos, pos)
                # illegal move
                if safeCost == -1:
                    continue
                else:
                    legalMoves.append([safeCost, pos])

            return legalMoves

        # initialise the heap, parent, visited and gDist
        heap = []
        heapq.heappush(heap, (greedyDist(start, end), start))
        parent = dict()
        visited = []
        gDist = {start: 0}

        # search
        while heap:
            # pop node with smallest g+h
            _, curPos = heapq.heappop(heap)

            if curPos in visited:
                continue

            if end == curPos:
                recordPathAndCost(start, end, parent, gDist[curPos])
                return
            
            visited.append(curPos)
            curDist = gDist[curPos]

            for safeCost, move in getLegalMoves(curPos):
                if move in visited:
                    continue
                
                # calculate cost of this move
                moveCost = safeCost
                if move[2] == curPos[2]:
                    moveCost += 1 # standard cost for movement
                else:
                    moveCost += 100 # cost for turns
                
                # calculate g+h for next move
                nextCost = curDist + moveCost + greedyDist(move, end)

                # TODO: THIS PART IS WEIRD!
                if move not in gDist or gDist[move] > curDist + moveCost:
                    gDist[move] = curDist + moveCost
                    parent[move] = curPos
                    heapq.heappush(heap, (nextCost, move))

    for i in range(len(states)-1): # reference triangle thing
        for j in range(i+1, len(states)):
            aStarSearch(states[i], states[j])

    return pathTable, costTable

# commands related

def commandify(optimalPath, goalCentres):

    def condenser(commands):
        condensed = []
        curStreak = None
        streakCount = 0

        for command in commands:
            if command[:2] == "FW":
                # no streak now
                if curStreak is None:
                    curStreak = "FW"
                    streakCount = 1
                # FW streak
                elif curStreak == "FW":
                    streakCount += 1
                # BW streak end
                else:
                    condensed.append("BW{:02d}".format(streakCount))
                    # reset 
                    curStreak = "FW"
                    streakCount = 1
            elif command[:2] == "BW":
                # no streak now
                if curStreak is None:
                    curStreak = "BW"
                    streakCount = 1
                # BW streak
                elif curStreak == "BW":
                    streakCount += 1
                # FW streak end
                else:
                    condensed.append("FW{:02d}".format(streakCount))
                    # reset 
                    curStreak = "BW"
                    streakCount = 1
            else:
                if curStreak is not None:
                    condensed.append("{}{:02d}".format(curStreak, streakCount))
                    # reset
                    curStreak = None
                    streakCount = 0
                    condensed.append(command)
                else:
                    condensed.append(command)
        
        return condensed

    commands = []

    for i in range(1, len(optimalPath)):
        
        curState = optimalPath[i]
        prevState = optimalPath[i-1]

        if curState == prevState:
            continue

        # fw and bw movements
        if curState[2] == prevState[2]:
            if curState[2] == "N":
                if curState[0] > prevState[0]:
                    commands.append("FW01")
                else:
                    commands.append("BW01")
            elif curState[2] == "S":
                if curState[0] < prevState[0]:
                    commands.append("FW01")
                else:
                    commands.append("BW01")
            elif curState[2] == "E":
                if curState[1] > prevState[1]:
                    commands.append("FW01")
                else:
                    commands.append("BW01")
            elif curState[2] == "W":
                if curState[1] < prevState[1]:
                    commands.append("FW01")
                else:
                    commands.append("BW01")

        # turning movements
        else:
            if prevState[2] == "N":
                if curState[2] == "W":
                    if curState[0] > prevState[0]:
                        commands.append("FL00")
                    else:
                        commands.append("BR00")
                elif curState[2] == "E":
                    if curState[0] > prevState[0]:
                        commands.append("FR00")
                    else:
                        commands.append("BL00")
            elif prevState[2] == "S":
                if curState[2] == "W":
                    if curState[0] < prevState[0]:
                        commands.append("FR00")
                    else:
                        commands.append("BL00")
                elif curState[2] == "E":
                    if curState[0] < prevState[0]:
                        commands.append("FL00")
                    else:
                        commands.append("BR00")
            elif prevState[2] == "E":
                if curState[2] == "N":
                    if curState[1] > prevState[1]:
                        commands.append("FL00")
                    else:
                        commands.append("BR00")
                elif curState[2] == "S":
                    if curState[1] > prevState[1]:
                        commands.append("FR00")
                    else:
                        commands.append("BL00")
            elif prevState[2] == "W":
                if curState[2] == "N":
                    if curState[1] < prevState[1]:
                        commands.append("FR00")
                    else:
                        commands.append("BL00")
                elif curState[2] == "S":
                    if curState[1] < prevState[1]:
                        commands.append("FL00")
                    else:
                        commands.append("BR00")

        # goal reached
        if curState in goalCentres.keys():
            commands.append("TP{:02d}".format(goalCentres[curState]))

    # condense path
    commands = condenser(commands)
    
    return commands

def scondenser(states):

    toRemove = []       # because they are not the interim states
    checkpoints = []    # interim states (after a command)
    diff = 0

    # loop through first time to eliminate non-interim states
    for i in range(1, len(states)):
        curState = states[i]
        prevState = states[i-1]

        # bef and aft states are the same - goal reached
        if curState == prevState:
            toRemove.append(i-1)  # remove a dup
            diff = 0
        # same direction
        elif curState[2] == prevState[2]:
            if curState[2] == 'N' or curState[2] == 'S':
                curDiff = curState[0] - prevState[0]
            else:
                curDiff = curState[1] - prevState[1]
            if diff == 0:
                diff = curDiff
            elif curDiff == diff:
                toRemove.append(i-1)  # throw away non-interim
            else:
                diff = curDiff
        # different direction
        else:
            diff = 0

    for i in range(len(states)):
        if i not in toRemove:
            checkpoints.append(states[i])

    return checkpoints

# goal related

def getGoalCentres(obs):
    goalList = dict()
    for ob in obs:
        goalCentre = []
        if ob[2] == 'N':
            goalCentre = [ob[0]+4, ob[1], 'S']
        elif ob[2] == 'S':
            goalCentre = [ob[0]-4, ob[1], 'N']
        elif ob[2] == 'E':
            goalCentre = [ob[0], ob[1]+4, 'W']
        elif ob[2] == 'W':
            goalCentre = [ob[0], ob[1]-4, 'E']
        
        # move row inwards for edge cases
        if goalCentre[0] == 1:
            goalCentre[0] = 2
        elif goalCentre[0] == 20:
            goalCentre[0] = 19

        # move col inwards for edge cases
        if goalCentre[1] == 1:
                goalCentre[1] = 2
        elif goalCentre[1] == 20:
            goalCentre[1] = 19

        if goalCentre[0] < 1 or goalCentre[0] > 20 or goalCentre[1] < 1 or goalCentre[1] > 20:
            # totally out of the qn
            continue
            
        goalList[tuple(goalCentre)] = ob[3]
    return goalList

def markGoals(goalCentres):
    def isOccupied(centre):
        for x in range(centre[1]-1, centre[1]+2):
            for y in range(centre[0]-1, centre[0]+2):
                if map[x][y] == "X":
                    return True
        return False

    failed = []

    for goal in goalCentres:
        if not isOccupied(goal):
            for x in range(goal[1]-1, goal[1]+2):
                for y in range(goal[0]-1, goal[0]+2):
                    if x == goal[1] and y == goal[0]:
                        map[x][y] = goal[2]
                    else:
                        if map[x][y] not in ["D","N","S","E","W"]:
                            map[x][y] = "G"
        else:
            failed.append(goal)
    
    return failed

# map related (N is right-side)

def markObs(obs):
    for ob in obs:
        xstart = ob[1] - 1 if ob[1] >= 2 else ob[1]
        xend = ob[1] + 2 if ob[1] <= 19 else ob[1] + 1
        ystart = ob[0] - 1 if ob[0] >= 2 else ob[0]
        yend = ob[0] + 2 if ob[0] <= 19 else ob[0] + 1
        for x in range(xstart, xend):
            for y in range(ystart, yend):
                if x == ob[1] and y == ob[0]:
                    map[x][y] = "X"
                else:
                    if map[x][y] != "X":
                        map[x][y] = "D"

def visualiseMap():
    with open("visualisation.txt", "w") as f:
        for i in range(len(map)):
            if i == 0:
                f.write("0 1 2 3 4 5 6 7 8 9 A B C D E F G H I J K\n")
            else:
                codebook = {
                    10: "A",
                    11: "B",
                    12: "C",
                    13: "D",
                    14: "E",
                    15: "F",
                    16: "G",
                    17: "H",
                    18: "I",
                    19: "J",
                    20: "K"
                }
                writeStr = " ".join(map[i]) + "\n"
                writeStr = "{} {}".format(i if i < 10 else codebook[i], writeStr[2:])
                f.write(writeStr)

def resetMap():
    for i in range(21):
        for j in range(21):
            map[i][j] = "_"