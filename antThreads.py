from dijkstras import *
from time import time
import threading
from time import sleep
import matplotlib.pyplot as plt
import numpy as numpy
import copy
from dijkstras import graph

#nodeCount = 100
#degree = 30
#maxWeight = 100
noOfGenerations = 500
antsPerGeneration = 2
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
antLock = threading.Lock()


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


def pickNextNode(G, currentNode, localPherTable):
	sum = 0
	weights = {}
	actualWeights = {}
	antLock.acquire()
	neighbours = G[currentNode]
	antLock.release()
	for node in neighbours:
		ph = localPherTable[currentNode][node.nodeNumber]
		x = (1-alpha)*1 + (alpha*ph)
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

def localDepositPheromone(localPherTable, i, j, newPheromone):
	#G.pheromone[i][j] = G.pheromone[i][j] + newPheromone
	#G.localUpdatePheromone(i, j, newPheromone)
	localPherTable[i][j] = localPherTable[i][j] + newPheromone

def globalDepositPheromone(localPherTable, i, j, newPheromone):
	#G.pheromone[i][j] = G.pheromone[i][j] + newPheromone
	#G.globalUpdatePheromone(i, j, newPheromone)
	localPherTable[i][j] = localPherTable[i][j] + newPheromone

def newActiveAnt(G, source, target, count):
	#print ("Ant " + str(count) +" has been born")
	ant = Ant()
	currentNode = source
	ant.appendToPath(source, 0)
	localPherTable = copy.deepcopy(G.pheromone)

	while(currentNode != target):
		antLock.acquire()
		neighbours = G[currentNode]
		antLock.release()
		nextNode, cost = pickNextNode(G, currentNode, localPherTable)
		ant.appendToPath(nextNode, cost)
		stepPheromone = (1.0/(cost))
		if onlineStepUpdate:
			localDepositPheromone(localPherTable, currentNode, nextNode, stepPheromone)
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
				globalDepositPheromone(localPherTable, currentNode, nextNode, pheromone)
				i = i + 1
				currentNode = nextNode
				nextNode = ant.pathMem[i]

	#Update the global graph
	antLock.acquire()
	G.updatePheromone(localPherTable)
	antLock.release()
	#print ("Ant "+str(count)+" terminated with cost = " + str(ant.pathCost))
	antLock.acquire()
	pathsGenerated[count] = ant.pathCost
	antLock.release()
	#print ("Ant "+str(count)+" terminated with cost = ")

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
		if i%5 == 0:
			antLock.acquire()
			pheromoneEvaporation(G)
			antLock.release()
		#daemonActions(G)
		sleep(0.5)
		i = i + 1
	print ("All ants generated!")

def printGraph(H, i):
	minWeight = 11111110
	for i in range (0, H.nodeCount):
		print ("Node number: "+str(i) +" neighbours:")
		for node in H.adjacencyList[i]:
			print ( str(node.nodeNumber) + " - " + str(node.weight))


def main():
	global alpha
	alpha = input("Enter a value for alpha: ")
	'''G = graph(nodeCount, degree)
	G.populateGraph()
	source = random.randint(0, nodeCount - 1)
	target = random.randint(0, nodeCount - 1)
	G.connect(source, target)

	pickle.dump( G, open( "save.p", "wb" ) )'''
	#G = graph(nodeCount, degree)
	#G.populateGraph()
	for ii in range (0,5):
		G = pickle.load( open( "save.p", "r" ) )
		source = pickle.load( open( "source.p", "r" ) )
		target = pickle.load( open( "target.p", "r" ) )

		
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
		for value in pathsGenerated.values():
			error.append(value - shortest)'''

		'''for i, v in enumerate(error):
			print ("ant: "+str(i)+" cost: "+str(v))
		error = []
		antNumber = []
		ii = 0
		for path in pathsGenerated:
			x = path - shortest
			error.append(x)
			antNumber.append(ii)
			ii = ii + 1
		plt.plot(antNumber, error)
		plt.show()'''
		
		fig, ax = plt.subplots()
		ax.plot(pathsGenerated.keys(), error, 'bo', linewidth = 0, label = "alpha = "+ str(alpha))
		legend = ax.legend(shadow=True)
		for label in legend.get_texts():
			label.set_fontsize('large')
		for label in legend.get_lines():
			label.set_linewidth(1.5) 
		#plt.show()
		fname = "C:\Content\AI\ACO\graphs\\alpha" + str(alpha) +"graph"+str(ii)+".png"
		plt.savefig(fname)
		#pickle.dump( H, open( "save.p", "wb" ) )




main()
