from dijkstras import *
from time import time
import threading
from time import sleep
import matplotlib.pyplot as plt
import numpy as numpy

#nodeCount = 100
#degree = 30
#maxWeight = 100
noOfGenerations = 200
antsPerGeneration = 10
source = 0
target = 0
onlineStepUpdate = 1
onlineDelayedUpdate = 1
antList = []
antCount = 0
#alpha is the exploitation factor
#alpha = 1 implies complete exploitatin
#alpha = 0 implies complete exploration
alpha = 1
pathsGenerated = {}


class Ant():
	pathMem = []
	pathCost = 0

	def __init__(self):
		self.pathMem = []
		self.pathCost = 0

	def appendToPath(self, node, cost):
		self.pathMem.append(node)
		self.pathCost = self.pathCost + cost

def pheromoneEvaporation(G):
	for i in range (0, nodeCount):
		for j in range (0, nodeCount):
			G.pheromone[i][j] = (G.pheromone[i][j]*0.3)


def pickNextNode(G, currentNode):
	sum = 0
	weights = {}
	actualWeights = {}
	neighbours = G[currentNode]
	for node in neighbours:
		x = G.pheromone[currentNode][node.nodeNumber]
		#x = (1-alpha)*1 + (alpha*ph)
		#x = ph
		sum = sum + x
		weights[node.nodeNumber] = x
		actualWeights[node.nodeNumber] = node.weight
	r = random.uniform(0, sum)
	s = 0
	for index, weight in weights.iteritems():
		s = s + weight
		if s >= r:
			#print ("Returning: " + str(node.nodeNumber))
			return index, actualWeights[index]
	#print ("Returning: " + str(neighbours[-1].nodeNumber))
	return neighbours[-1].nodeNumber, neighbours[-1].weight

def localDepositPheromone(G, i, j, newPheromone):
	#G.pheromone[i][j] = G.pheromone[i][j] + newPheromone
	G.localUpdatePheromone(i, j, newPheromone)

def globalDepositPheromone(G, i, j, newPheromone):
	#G.pheromone[i][j] = G.pheromone[i][j] + newPheromone
	G.globalUpdatePheromone(i, j, newPheromone)

def newActiveAnt(G, source, target, count):
	#print ("Ant " + str(count) +" has been born")
	ant = Ant()
	currentNode = source
	ant.appendToPath(source, 0)

	while(currentNode != target):
		neighbours = G[currentNode]
		nextNode, cost = pickNextNode(G, currentNode)
		ant.appendToPath(nextNode, cost)
		stepPheromone = (1.0/(cost))
		if onlineStepUpdate:
			localDepositPheromone(G, currentNode, nextNode, stepPheromone)
		#print (str(G.pheromone[currentNode][nextNode]))
		currentNode = nextNode
	#print ("Ant has reached here " + str(count))
	if onlineDelayedUpdate:
		currentNode = ant.pathMem[0]
		if len(ant.pathMem) > 1:
			nextNode = ant.pathMem[1]
			i = 2
			pheromone = (1.0/ant.pathCost)
			while (i < len(ant.pathMem) - 1):
				globalDepositPheromone(G, currentNode, nextNode, pheromone)
				i = i + 1
				currentNode = nextNode
				nextNode = ant.pathMem[i]
	print ("Ant "+str(count)+" terminated with cost = " + str(ant.pathCost))
	pathsGenerated[count/14] = ant.pathCost

def antsGenerationAndActivity(G, s, t):
	global antCount
	i = 1
	while i < antsPerGeneration:
		t = threading.Thread(target = newActiveAnt, args = (G, s, t, antCount))
		antList.append(t)
		t.daemon = True

		#print("Starting ant " + str(antCount))
		antCount = antCount + 1
		t.start()
		#sleep(0.1)
		i = i + 1


def ACOMetaHeuristic(G, s, t):
	i = 1
	while i < noOfGenerations:
		antsGenerationAndActivity(G, s, t)
		'''if i%50 == 0:
			pheromoneEvaporation(G)'''
		#daemonActions(G)
		sleep(0.5)
		i = i + 1
	print ("All ants generated!")


def main():
	G = graph(nodeCount, degree)
	G.populateGraph()
	source = random.randint(0, nodeCount - 1)
	target = random.randint(0, nodeCount - 1)
	G.connect(source, target)
	#G.initPheromone(1)
	spath, dadHeap = maxCapacityHeap(G, source, target)
	print ("Shortest path = " + str(spath[target]))
	#newActiveAnt(G, source, target)
	ACOMetaHeuristic(G, source, target)
	sleep(10)
	#sleep(30)
	shortest = spath[target]
	error = pathsGenerated.values()
	for value in error:
		value = value - shortest
	'''error = []
	antNumber = []
	ii = 0
	for path in pathsGenerated:
		x = path - shortest
		error.append(x)
		antNumber.append(ii)
		ii = ii + 1
	plt.plot(antNumber, error)
	plt.show()'''

	plt.plot(pathsGenerated.keys(), error, 'bo', linewidth = 0)
	plt.show()




main()
