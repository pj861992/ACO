import random
from time import time
import pickle
import pprint
from operator import itemgetter, attrgetter

nodeCount = 200
degree = 30
maxWeight= 100
#ld = local decay coefficient
ld = 0.5

class adjacencyNode:
    def __init__(self, nodeNumber, weight):
        self.nodeNumber = nodeNumber
        self.weight = weight
        self.pheromone = 0

def generateEdges(G):
    edges = []
    i = 0
    for i in range (0, nodeCount):
        node = G[i]
        j = 0
        ii = len(node) - 1
        while ii > -1:
            neighbour = node[ii]
            ii = ii - 1
            if neighbour.nodeNumber > i:
                vertices = [i, neighbour.nodeNumber]
                weight = neighbour.weight
                edges.append([vertices, weight])
    return edges





class graph:
    adjacencyList = [[] for i in range(nodeCount)]
    nodeCount = 0
    degree = 0
    pheromone = [[0.001 for j in range(nodeCount)]for i in range(nodeCount)]

    def __init__(self, nodeCount, degree):
        self.nodeCount = nodeCount
        self.degree = degree
        self.adjacencyList = [[] for i in range(nodeCount)]
        self.pheromone = [[0.1 for j in range(nodeCount)]for i in range(nodeCount)]

    def __getitem__(self, i):
        return self.adjacencyList[i]

    def __iter__(self):
        return self.adjacencyList.itervalues()

    def localUpdatePheromone(self, i, j, newPheromone):
        self.pheromone[i][j] = (self.pheromone[i][j]) + newPheromone

    def globalUpdatePheromone(self, i, j, newPheromone):
        self.pheromone[i][j] = (self.pheromone[i][j]) + newPheromone

    def updatePheromone(self, localPherTable):
        for i in range (nodeCount):
            for j in range (nodeCount):
                self.pheromone[i][j] = (self.pheromone[i][j] + localPherTable[i][j])/2


    def printGraph(self, i):
        minWeight = 11111110
        for i in range (0, self.nodeCount):
            print ("Node number: "+str(i) +" neighbours:")
            for node in self.adjacencyList[i]:
                print ( str(node.nodeNumber) + " - " + str(node.weight))

    def initPheromone(self, x):
        i = 0
        for node in self.adjacencyList:
            for neighbour in node:
                self.pheromone[i][neighbour.nodeNumber] = 1/(neighbour.weight)
            i = i + 1
        print ("Initialized pheromones")

    def removeDuplicates(self, i):
        hashTable = []
        count = 0
        for i in range (0, self.nodeCount):
            hashTable.append(0)
        for i in range (0, self.nodeCount):
            count = count + 1
            for j in range(0, self.nodeCount):
                hashTable[j] = 0
            graveyard = []
            #populate hash table with list values
            for node in self.adjacencyList[i]:
                hashTable[node.nodeNumber] = hashTable[node.nodeNumber] + 1
            #delete duplicate nodes
            for node in self.adjacencyList[i]:
                if hashTable[node.nodeNumber] > 1:
                    graveyard.append(node)
                    hashTable[node.nodeNumber] = hashTable[node.nodeNumber] - 1
            for entry in graveyard:
                self.adjacencyList[i].remove(entry)
            
    def connect(self, source, target):
        nodes = []
        for i in range (0, self.nodeCount):
            nodes.append(i)
        currentNode = source
        maxCap = 100000
        nodes.remove(target)
        for iterator in range (0, self.nodeCount - 2):
            nodes.remove(currentNode)
            nextNodeIndex = random.randint(0, len(nodes)-1)
            nextNode = nodes[nextNodeIndex]
            weight = random.randint(1, maxWeight)
            weight2 = random.randint(1, maxWeight)
            entry1 = adjacencyNode(currentNode, weight)
            entry2 = adjacencyNode(nextNode, weight2)
            maxCap = min(maxCap, weight)
            self.adjacencyList[currentNode].append(entry2)
            self.adjacencyList[nextNode].append(entry1)
            currentNode = nextNode
        nextNode = target
        weight = random.randint(1, maxWeight)
        weight2 = random.randint(1, maxWeight)
        entry1 = adjacencyNode(currentNode, weight)
        entry2 = adjacencyNode(nextNode, weight2)
        #maxCap = min(maxCap, weight)
        self.adjacencyList[currentNode].append(entry2)
        self.adjacencyList[nextNode].append(entry1)
        

    def populateGraph(self):
        edgeCount = {}
        Count = 0
        for i in range (1, self.nodeCount):
            edgeCount[i] = 0
        print len(edgeCount)
        while len(edgeCount) > 3:
            l = len(edgeCount)
            if Count > 14000:
                if l % 1000000 == 0:
                    print l
            index1 = random.randint(1, l-1)
            index2 = random.randint(1, l-1)
            weight = random.randint(1, maxWeight)
            weight2 = random.randint(1, maxWeight)
            if index1 != index2:
                vertex1 = edgeCount.keys()[index1]
                vertex2 = edgeCount.keys()[index2]
                entry1 = adjacencyNode(vertex1, weight)
                entry2 = adjacencyNode(vertex2, weight2)
                self.adjacencyList[vertex1].append(entry2)
                self.adjacencyList[vertex2].append(entry1)
                edgeCount[vertex1] = edgeCount[vertex1] + 1
                edgeCount[vertex2] = edgeCount[vertex2] + 1
                Count = Count + 1
                if edgeCount[vertex1] >= self.degree:
                    del edgeCount[vertex1]
                if edgeCount[vertex2] >= self.degree:
                    del edgeCount[vertex2]

                if Count % 100000 == 0:
                    print Count
                if Count > 2480000:
                    break
        print("Total edge count = " + str(Count))
        print ("Deleting duplicates")

    def addEdge(self, edge):
        vertex1 = edge[0][0]
        vertex2 = edge[0][1]
        weight = edge[1]
        weight2 = random.randint(1, maxWeight)
        entry1 = adjacencyNode(vertex1, weight)
        entry2 = adjacencyNode(vertex2, weight2)
        self.adjacencyList[vertex1].append(entry2)
        self.adjacencyList[vertex2].append(entry1)

    color = []
    dad = []
    weightToDad = []

    def findPath(self, s, t):
        self.color = []
        self.dad = []
        self.weightToDad = []
        for i in range (0, nodeCount):
            self.color.append("white")
            self.dad.append(-1)
            self.weightToDad.append(0)
        self.DFS(s)
        return self.dad, self.weightToDad

    def DFS(self, v):
        self.color[v] = "gray"
        '''for i in range (0, nodeCount):
            print (len(G[i]))'''
        for i in range (0, len(self.adjacencyList[v])):
            node = self.adjacencyList[v][i]
            w = node.nodeNumber
            if self.color[w] == "white":
                self.dad[w] = v
                self.weightToDad[w] = node.weight
                self.DFS(w)
        self.color[v] = "black"

class heap:
    size = 0
    data = []
    D = []
    indexOf = []
    
    '''def __init__(self, size):
        self.size = size
        for i in range (0, nodeCount):
            self.D.append(0)
            self.indexOf.append(-1)'''

    def __init__(self, size, data, D, indexOf):
        self.size = size
        if (D != 0):
            self.data = data
            self.D = D
            self.indexOf = indexOf
        else:
            for i in range (0, nodeCount):
                self.D.append(0)
                self.indexOf.append(-1)

    def getSize(self, i):
        return self.size

    def greaterThan(self, a,b):
        value1 = self.D[a]
        value2 = self.D[b]
        if value1 > value2:
            return True
        else:
            return False

    def lessThan(self, a,b):
        value1 = self.D[a]
        value2 = self.D[b]
        if value1 < value2:
            return True
        else:
            return False

    def siftUp(self, index):
        parentIndex = (index-1)/2
        while ( index != 0):
            parentIndex = (index - 1)/2
            if self.greaterThan(self.data[parentIndex], self.data[index]):
                #sift up
                self.indexOf[self.data[index]] = parentIndex
                self.indexOf[self.data[parentIndex]] = index
                temp = self.data[index]
                self.data[index] = self.data[parentIndex]
                self.data[parentIndex] = temp
                index = parentIndex
            else:
                index = 0

    def siftDown(self, index, end):
        sift = False
        childIndex = index
        while (2*index) + 1 < end - 1:
            sift = False
            child1Index = (2*index) + 1
            child2Index = (2*index) + 2
            if self.lessThan(self.data[child1Index], self.data[index]):
                childIndex = child1Index
                sift = True
            if (2*index) + 2 < end-1:
                if self.lessThan(self.data[child2Index], self.data[childIndex]):
                    childIndex = child2Index
                    sift = True
            if sift == True:
                self.indexOf[self.data[index]] = childIndex
                self.indexOf[self.data[childIndex]] = index
                temp = self.data[index]
                self.data[index] = self.data[childIndex]
                self.data[childIndex] = temp
                index = childIndex
            else:
                index = end
        return

    def insert(self, vertex, value):
        self.D[vertex] = value
        self.indexOf[vertex] = self.size
        self.size = self.size + 1
        self.data.append(vertex)
        
        self.siftUp(self.size - 1)

    def delete(self, j):
        maximum = self.data[0]
        self.indexOf[maximum] = -1
        weight = self.D[maximum]
        self.data[0] = self.data[-1]
        self.indexOf[self.data[0]] = 0
        self.size = self.size - 1
        del self.data[-1]
        self.siftDown(0, self.size)
        return maximum, weight

    
    def displayHeap(self, i):
        print ("Printing heap:")
        i = 0
        for element in self.data:
            print (str(element) + ": " + str(self.D[element]))
            i = i + 1
        print ("Heap printed")

    def update(self, vertex, value):
        self.D[vertex] = value
        self.siftUp(self.indexOf[vertex])
        self.siftDown(self.indexOf[vertex], self.size)

    def maximum(self, i):
        return self.data[0]

    def heapify(self, i):
        for i in range (1, self.size):
            self.siftUp(i)

    def pop(self, i):
        data = self.data.pop()
        weight = self.D[data]
        return data, weight
            
            
def pickMaxFringe(fringeList, maxCap):
    maxFringe = fringeList[0]
    for fringe in fringeList:
        if maxCap[fringe.nodeNumber] < maxCap[maxFringe.nodeNumber]:
            maxFringe = fringe
    fringeList.remove(maxFringe)
    return maxFringe, fringeList


def maxCapacityHeap(G, s , t):
    status = []
    maxCap = []
    dad = []
    fringeList = heap(0, 0, 0, 0)
    for i in range (0, nodeCount):
        status.append("unseen")
        maxCap.append(0)
        dad.append(None)
        
    status[s] = "intree"
    maxCap[s] = float("inf")
    dad[s] = None

    for node in G[s]:
        #add node to fringe list
        status[node.nodeNumber] = "fringe"
        fringeList.insert(node.nodeNumber, node.weight)
        maxCap[node.nodeNumber] = node.weight
        dad[node.nodeNumber] = s

    while fringeList.getSize(0) > 0:
        nodeNumber, weight = fringeList.delete(0)
        status[nodeNumber] = "intree"
        for neighbour in G[nodeNumber]:
            if status[neighbour.nodeNumber] == "unseen":
                status[neighbour.nodeNumber] = "fringe"
                maxCap[neighbour.nodeNumber] = maxCap[nodeNumber] + neighbour.weight
                fringeList.insert(neighbour.nodeNumber, maxCap[neighbour.nodeNumber])
                dad[neighbour.nodeNumber] = nodeNumber
            else:
                if status[neighbour.nodeNumber] == "fringe" and \
                   maxCap[neighbour.nodeNumber] > maxCap[nodeNumber] + neighbour.weight:
                    maxCap[neighbour.nodeNumber] = maxCap[nodeNumber] + neighbour.weight
                    fringeList.update(neighbour.nodeNumber, maxCap[neighbour.nodeNumber])
                    dad[neighbour.nodeNumber] = nodeNumber
    return maxCap, dad
    
    

def main():
    global nodeCount
    global degree
    nodeCount = input("Enter number of nodes: ")
    degree = input("Enter degree: ")
    data = [[[0 for k in xrange(3)] for j in xrange(6)] for i in xrange(5)]
    for k in range (0, 1):
        start = time()
        G = graph(nodeCount, degree)
        G.populateGraph()

        source = []
        target = []

        sum1 = 0
        sum2 = 0
        sum3 = 0

        for i in range (0, 5):
            source.append(random.randint(1,nodeCount - 1))
            target.append(random.randint(1,nodeCount - 1))
            while (source[i] == target[i]):
                target[i] = random.randint(1,nodeCount - 1)
            G.connect(source[i], target[i])

        G.removeDuplicates(0)
        E = generateEdges(G)
        runningTime = time() - start
        
        #MSTstart = time()
        #MST = generateMST(E)
        #MSTTime = time() - MSTstart
        
        #runningTime = runningTime - MSTTime
        print ("Graph constructed in " + str(runningTime) + " seconds") 
        
        #G.printGraph(0)

        for i in range (0, 5):
            s = source[i]
            t = target[i]
            print ("\nSource " + str(i) + (": ") + str(s))
            print ("Target " + str(i) + (": ") + str(t))

            '''print("Finding max capacity without heap")
            start = time()
            maxCap, dad = maxCapacity1(G, s, t)
            runningTime = time()- start
            print ("Max capacity = " + str(maxCap[t]))
            print ("Calculated in time " + str(runningTime))
            data[k][i][0] = runningTime
            sum1 = sum1 + runningTime'''
            
            print("Finding max capacity with heap")
            start = time()
            maxCapHeap, dadHeap = maxCapacityHeap(G, s, t)
            runningTimeHeap = time()- start
            print ("Max capacity = " + str(maxCapHeap[t]))
            print ("Calculated in time " + str(runningTimeHeap))
            data[k][i][1] = runningTimeHeap
            sum2 = sum2 + runningTimeHeap
            
            '''print("Finding max capacity with Kruskals")
            start = time()
            maxCapKruskals, dad = maxCapacityKruskals(MST, s, t)
            runningTimeKruskals = MSTTime + time()- start
            print ("Max capacity = " + str(maxCapKruskals))
            print ("Calculated in time " + str(runningTimeKruskals))
            data[k][i][2] = runningTimeKruskals
            sum3 = sum3 + runningTimeKruskals'''

        avg1 = sum1/5.0
        data[k][5][0] = avg1
        avg2 = sum2/5.0
        data[k][5][1] = avg2
        avg3 = sum3/5.0
        data[k][5][2] = avg3

        print ("\n Average running times:")
        print ("Modified Dijkstra's without heap: " + str(avg1))
        print ("Modified Dijkstra's with heap: " + str(avg2))
        print ("Kruskal's with heapsort: " + str(avg3))

#main()


