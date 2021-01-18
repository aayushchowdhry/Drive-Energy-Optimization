import math
from sklearn.neighbors import KDTree

class TrackNodeKDTree(KDTree):
    """
    KDTree of TrackNodes to get the nearest TrackNode to a given point.
    """

    def __init__(self, data):
        """
        Creates a KDTree of TrackNode objects.

        Parameter data: The TrackNode obejcts of the graph to create a KDTree for.
        Precondition: data is a List of TrackNode objects.
        """
        self._data = data
        self._coordinates = self._projectTo2D(self._data)
        super().__init__(self._coordinates)


    def getClosestNode(self, point):
        """
        Return closest TrackNode to the given point.

        Parameter point: the point to which the closest node is to be found.
        Precondition: point is a list of length 2
        """
        assert len(point)==2
        index = self.query([point], return_distance = False)[0][0]
        return self._data[index]


    def getClosestForwardNode(self, point, heading):
        """
        Return closest TrackNode in front of the given point.

        Parameter point: the point to which the closest node is to be found.
        Precondition: point is a list of length 2

        Parameter heading: the current heading of the object in radians
        Precondition: heading is between 0 and 2pi
        """
        assert 0<=heading<2*math.pi
        heading = heading - 2*math.pi if heading>math.pi else heading
        indices = self.query([point], k=4, return_distance = False)[0]
        for i in indices:
            nodeCoordinate = self._coordinates[i]
            vector = [nodeCoordinate[0]-point[0],nodeCoordinate[1]-point[1]]
            if vector[0]==0:
                if vector[1]==0:
                    angleFromPoint= 0
                else:
                    angleFromPoint = math.pi/2 if vector[1]>0 else -math.pi/2
            else:
                angleFromPoint = math.atan2(vector[1],vector[0])
            headingFromPoint = math.pi/2 - heading - angleFromPoint
            if (-math.pi/2 <= headingFromPoint <= math.pi/2):
                return self._data[i]
        return self.getClosestNode(point)


    def _projectTo2D(self, data):
        """
        Helper function to extract a list of coordinates from a list of TrackNodes.

        Parameter data: The TrackNode obejcts of the graph to create a KDTree for.
        Precondition: data is a List of TrackNode objects.
        """
        result = []
        for node in data:
            result.append([node.x,node.y])
        return result
