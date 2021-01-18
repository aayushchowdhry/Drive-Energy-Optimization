"""
"""
import math
from InterpolatedTrackKDTree import InterpolatedTrackKDTree

class CostNode:
    """
    """
    # get only attributes
    @property
    def speed(self):
        """
        """
        return self._speed

    @property
    def time(self):
        """
        """
        return self._time

    # other getters and setters
    @property
    def x(self):
        """
        """
        return self._x

    @x.setter
    def x(self,x):
        """
        """
        assert type(x) == float
        self._x = x

    @property
    def y(self):
        """
        """
        return self._y

    @y.setter
    def y(self,y):
        """
        """
        assert type(y) == float
        self._y = y

    @property
    def distance(self):
        """
        """
        return self._dist

    @distance.setter
    def distance(self,dist):
        """
        """
        assert type(dist) == float
        self._dist = dist

    @property
    def heading(self):
        """
        """
        return self._heading

    @heading.setter
    def heading(self,sa):
        """
        """
        assert type(sa) == float
        self._heading = sa

    @property
    def backPointer(self):
        """
        """
        return self._bckptr

    @backPointer.setter
    def backPointer(self,bckptr):
        """
        """
        assert type(bckptr) == CostNode
        self._bckptr = bckptr

    @property
    def cost(self):
        """
        """
        return self._cost

    @cost.setter
    def cost(self, cst):
        """
        """
        assert type(cst) == float
        self._cost = cst

    def __init__(self, speed, time):
        self._speed = speed
        self._time = time

        self._cost = inf
        self._x = inf
        self._y = inf
        self._heading = inf
        self._dist = inf
        self._bckptr = None

    def __str__(self):
        return "At Time "+str(self._time)+" and Speed "+str(self._speed)+" the distance covered was "+str(self._dist)+" and the (x,y,sa) coordinates were "+str((self._x,self._y,self._steeringAngle))+"."


def cosd(angle):
    """
    Returns the cosine of an angle in degrees.

    Parameter angle: The cosine of which to find.
    Precondition: angle is a number.
    """
    assert type(angle)==int or type(angle)==float
    return math.cos(math.radians(angle))


def sind(angle):
    """
    Returns the sine of an angle in degrees.

    Parameter angle: The sine of which to find.
    Precondition: angle is a number.
    """
    assert type(angle)==int or type(angle)==float
    return math.sin(math.radians(angle))


def energy(distanceCovered,averageSpeed,averageSteeringAngle,acceleration,
            verticalAngle):
    """
    """
    mass = 96; gravity = 9.8; airResistance = 0.01; rollingResistance = 0.03
    corneringResistance = 0

    forceExerted = mass*acceleration + airResistance*averageSpeed**2 + rollingResistance*mass*gravity*math.cos(verticalAngle) + mass*gravity*math.sin(verticalAngle) + corneringResistance*averageSteeringAngle
    energySpent = distanceCovered*forceExerted
    if energySpent < 0:
        energySpent = 0
    return energySpent



inf = float("inf")
# Trial TrackData
inside=[];out=[];
for i in range(0,361,10):
    inside.append([cosd(i),sind(i),(-1)**(i//2)])
for i in range(0,361,5):
    out.append([6*cosd(i),2*sind(i),(-1)**(i//2)])
interpolatedTrackData = InterpolatedTrackKDTree(inside, out)

maxDistance = interpolatedTrackData.distanceToCover
maxTime = 30
averageDistance = maxDistance/maxTime
speedList = list(range(16)) # list of possible speeds of the car
steeringAngleList = []# list of possible steeringAngles of the car
averageSpace = math.pi/36
for i in range(36):
    steeringAngleList.append(-math.pi/2+averageSpace*i)
costArray = [] # array of cost nodes in column major order
for time in range(maxTime):
    costList = []
    for speed in speedList:
        costList.append(CostNode(speed,time))
    costArray.append(costList)
costArray[0][0].cost = 0.0
costArray[0][0].x = interpolatedTrackData.startingPoint[0]
costArray[0][0].y = interpolatedTrackData.startingPoint[1]
costArray[0][0].distance = 0.0
costArray[0][0].heading = 0.0
# Invariant: We know the most cost efficient way to get to each speed in the
#           at each time int time[0..t].
t = 1
while (t<maxTime):
#   for each speed at n
    for currentNode in costArray[t]:
        currentSpeed = currentNode.speed
#       from each previous speed
        superMinCost = inf
        for previousNode in costArray[t-1]:
            previousSpeed = previousNode.speed
            previousDistance = previousNode.distance
            previousCost = previousNode.cost
            averageSpeed = (currentSpeed + previousSpeed)/2
            acceleration = (currentSpeed - previousSpeed) # as time = 1
            distanceCovered = averageSpeed # as time = 1
            currentDistance = previousDistance + distanceCovered
            distanceFactorInCost = (averageDistance-distanceCovered)**2
            # if race is finished and car is moving the cost is infinite
            if currentDistance > maxDistance:
                if currentSpeed == 0:
                    minCost = previousCost
                    minHeading = 0.0
                    minX = previousX
                    minY = previousY
                else:
                    minCost = inf
                    minHeading = inf
                    minX = inf
                    minY = inf
            else:
#               get previous (x,y)
                previousX = previousNode.x
                previousY = previousNode.y
                previousHeading = previousNode.heading
#               for each steering angle at n
                minCost = inf
                for currentSteeringAngle in steeringAngleList:
                    newHeading = previousHeading + currentSteeringAngle
#                   calculate current (x,y) and coressponding details
                    currentX = previousX + distanceCovered*math.cos(newHeading)
                    currentY = previousY + distanceCovered*math.sin(newHeading)
                    if interpolatedTrackData.isWithinBounds(currentX,currentY):
                        averageX = (currentX + previousX)/2
                        averageY = (currentY + previousY)/2
                        verticalAngle = interpolatedTrackData.getVerticalAngle(averageX,averageY)
#                       find the cost of reaching current (x,y) from  previous (x,y) at given speed and steering angle
                        e = energy(distanceCovered,averageSpeed,newHeading,acceleration,verticalAngle)
                        cost = previousCost + e + distanceFactorInCost
#                       find steering angle such that cost is minimum
                        if cost < minCost:
                            minCost = cost
                            minHeading = newHeading
                            minX = currentX
                            minY = currentY
#           find previous speed such that cost is minimum and store it in cost array[n][speedindex]
            if minCost<superMinCost:
                superMinCost = minCost
                superBackPointer = previousNode
                superMinHeading = minHeading
                superMinX = minX
                superMinY = minY
                superMinDistance = currentDistance
        currentNode.cost = superMinCost
        currentNode.distance = superMinDistance
        currentNode.x = superMinX
        currentNode.y = superMinY
        currentNode.heading = superMinHeading
        currentNode.backPointer = superBackPointer
    print(t)
    t += 1

# find minimum cost at maxTime
minCost = inf
for costNode in costArray[maxTime-1]:
    if costNode.cost<minCost:
        minCost = costNode.cost
        resultNode = costNode
# backtrace path
speedProfile=[];path=[];
while resultNode != None:
    path.append([resultNode.x,resultNode.y])
    speedProfile.append([resultNode.distance,resultNode.speed])
    resultNode = resultNode.backPointer

interpolatedTrackData.plotBirdsEye(interpolatedTrackData.interpolatedTrackData)
interpolatedTrackData.plotBirdsEye(speedProfile, True)
interpolatedTrackData.plotBirdsEye(path)
