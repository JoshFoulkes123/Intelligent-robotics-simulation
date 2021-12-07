import math

completeAStarMap = []


def read_map():
    img = open('../data/cafe_map_resize.pgm', 'rb')

    #Header values before width and height
    img.readline()

    (width, height) = [int(i) for i in img.readline().split()]
    depth = int(img.readline())
    assert depth <= 255

    aStarMap = []

    for y in range(height):
        row = []
        for y in range(width):
            val = ord(img.readline(1))
            if val < 100:
                row.append(0)
            elif val == 254:
                row.append(1)
            else:
                row.append(-1)
        aStarMap.append(row)

    #print(aStarMap)
    print("map made")
    return aStarMap



class aStarNode():
    def __init__(self, parentNode = None, position = None):
        self.parentNode = parentNode
        self.position = position
        self.x = position[0]
        self.y = position[1]

        self.h = 0
        self.g = 0
        self.f = 0

    def toString(self, completeAStarMap = None):
        if completeAStarMap == None:
            return '(' + str(self.x) + ',' + str(self.y) + ') [' + str(self.g) + ',' + str(self.h) + ',' + str(self.f)+  ']'
        else:
            return '(' + str(self.x) + ',' + str(self.y) + ') [' + str(self.g) + ',' + str(self.h) + ',' + str(self.f) + '] {' + str(completeAStarMap[self.y][self.x]) + '}'

# THIS
# FUNCTION
# HERE!

def a_star_cleanup(aStarNodes):
    completeAStarMap = read_map()
    width = len(completeAStarMap[0])
    height = len(completeAStarMap)

    toKeep = [0]
    nextI = 0

    for i in range(len(aStarNodes)):
        complete = False
        if i == nextI:
            for j in (range(i+1, len(aStarNodes))):
                nodeOne = aStarNodes[i]
                nodeTwo = aStarNodes[j]

                trajectoryNormalise = (math.hypot(nodeOne['x'] - nodeTwo['x'], nodeOne['y']-nodeTwo['y']) / 10)
                trajectory = (int((nodeTwo['x'] - nodeOne['x']) / trajectoryNormalise),int((nodeTwo['y'] - nodeOne['y']) / trajectoryNormalise))

                if trajectory[0] != 0 and trajectory[1] != 0:
                    for x in range(nodeOne['x'] + 2, nodeTwo['x'], int(trajectory[0])):
                        for y in range(nodeOne['y'] + 2, nodeTwo['y'], int(trajectory[1])):
                            if completeAStarMap[int(y)][int(x)] != 1:
                                toKeep.append(j - 1)
                                nextI = j - 1
                                complete = True
                                break
                        
                        if complete:
                            break
                elif trajectory[0] == 0:
                    for y in range(nodeOne['y'] + 2, nodeTwo['y'], int(trajectory[1])):
                        if completeAStarMap[int(y)][int(nodeOne['x'] )] != 1:
                            toKeep.append(j - 1)
                            nextI = j - 1
                            complete = True
                            break
                elif trajectory[1] == 0:
                    for x in range(nodeOne['x'] + 2, nodeTwo['x'], int(trajectory[0])):
                        if completeAStarMap[int(nodeOne['y'])][int(x)] != 1:
                            toKeep.append(j - 1)
                            nextI = j - 1
                            complete = True
                            break

                if complete:
                    break

    clean_a_star = []
    for i in toKeep:
        clean_a_star.append(aStarNodes[i])
    clean_a_star.append(aStarNodes[len(aStarNodes) -1])

    return clean_a_star

                


def a_star_calculation(start, end):
    completeAStarMap = read_map()
#   print(completeAStarMap)
    width = len(completeAStarMap[0])
    height = len(completeAStarMap)

    print(width)
    print(height)

    openList = []
    closedList = []

    openList.append(aStarNode(None, (start['x'], start['y'])))

    while not (len(openList) == 0 or len(openList) > 10000000):

        currentNode = openList[0]
        currentNodeNum = 0

        for i in range(len(openList)):
            if openList[i].f < currentNode.f:
                currentNode = openList[i]
                currentNodeNum = i

        openList.pop(currentNodeNum)
        closedList.append(currentNode)

  #      print('Start Node: (' + str(start['x']) + ',' + str(start['y']) + ')')
 ##       print('End Node: (' + str(end['x']) + ',' + str(end['y']) + ')')
        #print('Current Node: (' + str(currentNode.x) + ',' + str(currentNode.y) + ')')
        #print('Current Node: ' + currentNode.toString(completeAStarMap))
        #print('Size Open List: ' + str(len(openList)))
#        print('Size Closed List: ' + str(len(closedList)))
#        print()
#        print("OPEN LIST")
#        for node in openList:
#            print('(' + str(node['x']) + ',' + str(node['y']) + ')')
#        print()
#        print("CLOSED LIST")
#        for node in closedList:
#            print('(' + str(node['x']) + ',' + str(node['y']) + ')')
#        print()
#        time.sleep(1)

        if currentNode.x == end['x'] and currentNode.y == end['y']:
            path_to_end = []
            current_node = currentNode
            while current_node != None:
                path_to_end.append({'x':current_node.x, 'y':current_node.y})
                current_node = current_node.parentNode
            return path_to_end[::-1]
        else:
            children = []

            for child_position_rel in [(-1, -1), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1)]:
                child_position = (currentNode.x + child_position_rel[0], currentNode.y + child_position_rel[1])

                if child_position[0] < 0 or child_position[0] >= width or child_position[1] < 0 or child_position[1] >= height:
                    continue

                if completeAStarMap[child_position[1]][child_position[0]] != 1:
                    continue

                child = aStarNode(currentNode, child_position)
                child.parent = currentNode

#                print(child.toString())
                children.append(child)

#            if currentNode['x'] > 0:
#                if currentNode['y'] > 0:
#                    children.append(a_star_nodes[x-1][y-1])
#                if currentNode['y'] < height - 1:
#                    children.append(a_star_nodes[x-1][y+1])
#            if currentNode['x'] < width - 1:
#                children.append(a_star_nodes[x+1][y])
#                if currentNode['y'] > 0:
#                    children.append(a_star_nodes[x+1][y-1])
#                if currentNode['y'] < height - 1:
#                    children.append(a_star_nodes[x+1][y+1])
#            if currentNode['y'] > 0:
#                children.append(a_star_nodes[x][y-1])
#            if currentNode['y'] < height - 1:
#                children.append(a_star_nodes[x][y+1])

            for i in range(len(children)):
                skip = False
                for node in closedList:
                    if node.x == children[i].x and node.y == children[i].y:
                        skip = True
                        break

                if skip:
                    continue

                children[i].g = currentNode.g + math.hypot(children[i].x - currentNode.x, children[i].y - currentNode.y)
                children[i].h = math.hypot(end['x'] - children[i].x, end['y']-children[i].y)
                children[i].f = children[i].g + children[i].h

                for open_node in openList:
                    if open_node.x == children[i].x and open_node.y == children[i].y:
                        if children[i].g >= open_node.g:
                            skip = True
                            break

                if skip:
                    continue

                openList.append(children[i])

if __name__ == "__main__":
    a_star = a_star_calculation({'x':78, 'y':155}, {'x':130, 'y': 300})
    print(a_star)
    print()
    print(a_star_cleanup(a_star))
