from TrackNodeHeap import *
import numpy as np
import math

class TrackNodeInfo():
    """
    Class to store information regarding the shortest path of various TrackNodes.
    """
    # _dist= shortest known distance from the start node to this one
    # _bckptr= backpointer on path (with shortest known distance)
    #           from start node to this one

    @property
    def dist(self):
        return self._dist

    @property
    def bckptr(self):
        return self._bckptr

    @bckptr.setter
    def bckptr(self,bp):
        assert isinstance(bp,TrackNode)
        self._bckptr = bp

    @dist.setter
    def dist(self,d):
        assert type(d)==float
        self._dist = d

    def __init__(self, dist, bp):
        """
        Initializes an TrackNodeInfo with distance from the start node and a
        backpointer.

        Parameter dist: shortest known distance from the start node to this one
        Precondition: dist is a float

        Parameter bp: backpointer on path (with shortest known distance)
                        from start node to this one
        Precondition: bp is a TrackNode
        """
        self.dist= dist
        self.bkptr= bp

    def __str__(self):
        """
        Overrides python function str(TrackNodeInfo)
        """
        return "distance" + str(self._dist) + ", backpointer " + str(self._bkptr)


class TrackNode:
    """
    Class representing a track feature as a Node of a 3D coordinate
    """
    def __init__(self, x, y, z):
        self.edgesOut = []  # Contains edges from this node
        self.edgesIn = []     # Contains edges to this node
        self.neighborsFrom = []   # Contains nodes from which you can get here
        self.neighborsTo = [] # Contains nodes to which you can go from here
        self.x = x
        self.y = y
        self.z = z

    def addEdgeIn(self, edge):
        """ Add an edge to this Node """
        assert edge.nodeTo == self
        self.edgesIn.append(edge)
        if edge.nodeFrom not in self.neighborsFrom:
            self.neighborsFrom.append(edge.nodeFrom)

    def addEdgeOut(self, edge):
        """ Add an edge to this Node """
        assert edge.nodeFrom == self
        self.edgesOut.append(edge)
        if edge.nodeTo not in self.neighborsTo:
            self.neighborsTo.append(edge.nodeTo)

    # def findEdgesTo(self, other_node):
    #     """ Returns a list of edges connecting this node and [other_node] """
    #     return filter(lambda n: other_node in n.nodes, self.edges)

    # def __str__(self):
    #     return str((self.x, self.y, self.z))


class Edge:
    """
    Class representing a directed weighted edge
    """

    def __init__(self, weight, nodeFrom, nodeTo):
        assert isinstance(nodeFrom, TrackNode) and isinstance(nodeTo, TrackNode)
        assert type(weight) == float or type(weight) == int
        assert nodeFrom != nodeTo
        self.weight = weight
        self.nodeFrom = nodeFrom
        self.nodeTo = nodeTo
        nodeFrom.addEdgeOut(self)
        nodeTo.addEdgeIn(self)

    # def getOther(self,node):
    #     """
    #     Return the other node of the edge.
    #
    #     Parameter node: Node where this edge 'begins'
    #     Precondition: This edge is connected to this Node.
    #     """
    #     assert node in self.nodes
    #     if node==self.nodes[0]:
    #         return self.nodes[1]
    #     return self.nodes[0]

def energy(a, b, v):
    """
    Returns the energy required for a vehicle to go from TrackNode a to b at a
    constant speed.
    """
    m = 96; g = 9.8; CoeffAR = 0.01; CoeffRR = 0.03; # CoeffCR = 0
    # m = 100; g = 10; CoeffAR = 0; CoeffRR = 0;
    dist = math.sqrt((b.x - a.x)**2+(b.y - a.y)**2+(b.z - a.z)**2)
    va = math.asin((b.z - a.z)/dist)
    force = CoeffAR*v**2 + CoeffRR*m*g*math.cos(va) + m*g*math.sin(va) # + CoeffCR*sa
    energy = dist*force
    if energy < 0:
        energy = 0
    return energy

def interpolate(innerData, outerData):
    """
    Returns a list of arrays of row vectors in the form of [Inner Track,
    First Quarter, Middle Track, Second Quarter, Outer Track]

    Parameter innerData: The datapoints for the inner border of the track.
    Precondition: innerData is a list of row vectors with same length as outerData.

    Parameter outerData: The datapoints for the outer border of the track.
    Precondition: outerData is a list of row vectors  with same length as innerData.
    """
    assert len(innerData) == len(outerData)
    assert len(innerData) > 0
    assert len(innerData[0]) == len(outerData[0])
    IN = np.array(innerData)
    OUT = np.array(outerData)
    MID = (IN + OUT)/2
    FQ = (IN + MID)/2
    SQ = (MID + OUT)/2
    return [IN, FQ, MID, SQ, OUT]

def createGraph(arrayList, clockwise = True):
    """
    Returns a list of Track Nodes after creating the relevant edges from the
    array list.

    Parameter arrayList: The datapoints from which to create the graphs
    Precondition: arrayList is a list of matrices with row vectors where the ith
    row vector of a matrix can have edges to the (i+1)th row vectors or (i-1)th
    row vectors of all the matrices if clockwise is true or false respectively.

    Optional Parameter clockwise: Traversal direction is clockwise or not.
    Precondition: clockwise is a bool.
    """
    assert len(arrayList) > 0 and type(clockwise)==bool
    numArrs = len(arrayList)
    arrLen = len(arrayList[0])
    graph = [] # node corresponding arrayList[h][k] = graph[k*numArrs+h]
    for i in range(arrLen):
        for points in arrayList:
            node = TrackNode(points[i][0], points[i][1], points[i][2])
            graph.append(node)
            if i != 0:
                for index in range(numArrs):
                    fromNode = graph[(i-1)*numArrs+index]
                    weight = energy(fromNode, node, 5)
                    Edge(weight, fromNode, node)
    for points in arrayList:
        for index in range(numArrs):
            fromNode = graph[(arrLen-1)*numArrs+index]
            toNode = graph[index]
            weight = energy(fromNode, toNode, 5)
            Edge(weight, fromNode, toNode)
    return graph

def optimumPath(graphKDTree, current_position, goal_point):
    """
    """
    # IF you want to find nodes closest to start and end points
    start = graphKDTree.getClosestNode(current_position)
    end = graphKDTree.getClosestNode(goal_point)

    # The priority of a node will be the length of discovered
    # shortest path from v to the node.
    F= TrackNodeHeap(False);

    # SandF contains the required node information for all nodes in the settled
    # and frontier sets.
    # Keys and TrackNode objects and Values are TrackNodeInfo objects.
    SandF = {}

    # Initialize Settled={}, F={start}, d[start]=0, bckptr[start]=None
    F.add(start, 0.0);
    SandF[start]= TrackNodeInfo(0.0, None);

    # Invariant:
    # (1) For a node s in Settled set S, a shortest v --> s path exists that
    # contains only settled nodes; d[s] is its length (distance) and bk[s] is
    # s's backpointer.
    # (2) For each node f in Frontier set F, a v --> f path exists that contains
    # only settled nodes except for f; d[f] is the length (distance) of the
    # shortest such path and bk[f] is f's backpointer on that path.
    # (3) All edges leaving S go to F.
    while (len(F) != 0):
        # f= node in F with minimum d value. The path to this node is the shortest path.
        f= F.poll()

        # if f is end we have found the route to take.
        if f==end:
            path = []; energy = SandF[end].dist
            while (end != None):
                path.append(end)
                end= SandF[end].bkptr
            # UNCOMMENT TO add path from current_position to start node (closest node)
            # path.append(TrackNode(current_position[0],current_position[1]))
            path.reverse()
            # UNCOMMENT TO add path from end node (node closest to goal) to goal_point
            # path.append(TrackNode(goal_point[0],goal_point[1]))
            return path, energy
        fInfo= SandF[f]
        edges= f.edgesOut
        for edge in edges:
            w= edge.nodeTo #get neighbor for each edge.
            pathLength= fInfo.dist + edge.weight
            if w not in SandF.keys(): # if w is in far off set
                F.add(w, pathLength) # add it to the frontier
                SandF[w]= TrackNodeInfo(pathLength, f)
            elif pathLength < SandF[w].dist: # if w is in F and if newPath<d[w]
                wInfo= SandF[w]              # update priority and info
                wInfo.dist= pathLength
                wInfo.bkptr= f
                F.updatePriority(w, pathLength)
