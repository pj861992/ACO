from dijkstras import *

def printGraph(H, i):
	minWeight = 11111110
	for i in range (0, H.nodeCount):
		print ("Node number: "+str(i) +" neighbours:")
		for node in H.adjacencyList[i]:
			print ( str(node.nodeNumber) + " - " + str(node.weight))

def main():

	G = graph(nodeCount, degree)
	G.populateGraph()
	source = random.randint(0, nodeCount - 1)
	pickle.dump( source, open( "source.p", "wb" ) )
	target = random.randint(0, nodeCount - 1)
	pickle.dump( target, open( "target.p", "wb" ) )
	G.connect(source, target)
	pickle.dump( G, open( "save.p", "wb" ) )
	

main()