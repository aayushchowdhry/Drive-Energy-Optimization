"""
"""
import math
import matplotlib.pyplot as plt
from sklearn.neighbors import KDTree

class InterpolatedTrackKDTree(KDTree):
    """
    """
    @property
    def trackData(self):
        """
        """
        return self._trackData


    @property
    def interpolatedTrackData(self):
        """
        """
        return self._interpolatedTrackData


    @property
    def startingPoint(self):
        """
        """
        return self._midTrack[0]


    @property
    def distanceToCover(self):
        """
        """
        distance = 0
        previousPoint = self.startingPoint
        for point in self._midTrack:
            x = (previousPoint[0]-point[0])**2
            y = (previousPoint[1]-point[1])**2
            z = (previousPoint[2]-point[2])**2
            previousPoint = point
            distance += math.sqrt(x+y+z)
        return distance


    def __init__(self, innerData, outerData):
        """
        """
        self._trackData = innerData + outerData
        self._interpolatedTrackData = self._interpolate(innerData, outerData)
        super().__init__(self._projectTo2D(self._interpolatedTrackData))


    def getAltitude(self, x, y):
        """
        """
        indices = self.query([[x,y]], k=4, return_distance = False)[0]
        totalHeight = 0
        for i in indices:
            totalHeight+= self._interpolatedTrackData[i][2]
        return totalHeight/len(indices)


    def getVerticalAngle(self,x,y):
        """
        """
        indices = self.query([[x,y]], k=4, return_distance = False)[0]
        totalOfVA = 0
        for i in indices:
            point = self._interpolatedTrackData[i]
            x = point[0]
            y = point[1]
            z = point[2]
            rho = math.sqrt(x**2+y**2+z**2)
            totalOfVA += math.asin(z/rho)
        return totalOfVA/len(indices)


    def isWithinBounds(self,x,y):
        for i in range(len(self._innerBound)):
            innerBoundaryPoint = self._innerBound[i]
            outerBoundaryPoint = self._outerBound[i]
            if not (innerBoundaryPoint[0]**2 > x**2 and innerBoundaryPoint[1]**2 > y**2) or (outerBoundaryPoint[0]**2 < x**2 and outerBoundaryPoint[1]**2 < y**2):
                # if no for even 1, it is in bounds
                return True
        return False


    def plotBirdsEye(self, data , line = False, label = None):
        """
        """
        x=[];y=[];
        for i in range(len(data)):
            x.append(data[i][0]);y.append(data[i][1]);
        plt.figure(1)
        if line:
            plt.plot(x,y,label=label)
        else:
            plt.scatter(x, y, label = label, s=1)
        plt.xlim(min(x)-1,max(x)+1)
        plt.ylim(min(y)-1,max(y)+1)
        plt.xlabel('x - axis')
        plt.ylabel('y - axis')
        plt.title('Bird\'s Eye View')
        plt.show()


    def _interpolate(self,inside, out):
        """
        """
        factor=round(len(out)/len(inside))
        outside=[]
        for i in range(len(inside)):
            outside.append(out[factor*i])

        mid=[];innerquart=[];outerquart=[]
        for i in range(len(inside)):
            mid_point=[(inside[i][0]+outside[i][0])/2,
            (inside[i][1]+outside[i][1])/2,(inside[i][2]+outside[i][2])/2]
            mid.append(mid_point)
            innerquart_point=[(inside[i][0]+mid_point[0])/2,(inside[i][1]+mid_point[1])/2,
            (inside[i][2]+mid_point[2])/2]
            innerquart.append(innerquart_point)
            outerquart_point=[(mid_point[0]+outside[i][0])/2,(mid_point[1]+outside[i]
            [1])/2,(mid_point[2]+outside[i][2])/2]
            outerquart.append(outerquart_point)

        self._innerBound = inside
        self._outerBound = outside
        self._midTrack = mid


        k=len(inside)-1
        for i in range(0,4*k,4):
            mid_secondpoint=[(mid[i][0]+mid[i+1][0])/2,(mid[i][1]+mid[i+1][1])/2,
            (mid[i][2]+mid[i+1][2])/2]
            mid_firstpoint=[(mid[i][0]+mid_secondpoint[0])/2,
            (mid[i][1]+mid_secondpoint[1])/2,(mid[i][2]+mid_secondpoint[2])/2]
            mid_thirdpoint=[(mid[i+1][0]+mid_secondpoint[0])/2,
            (mid[i+1][1]+mid_secondpoint[1])/2,(mid[i+1][2]+mid_secondpoint[2])/2]
            mid.insert(i+1,mid_thirdpoint)
            mid.insert(i+1,mid_secondpoint)
            mid.insert(i+1,mid_firstpoint)

            innerquart_secondpoint=[(innerquart[i][0]+innerquart[i+1][0])/2,
            (innerquart[i][1]+innerquart[i+1][1])/2,(innerquart[i][2]+innerquart[i+1][2])/2]
            innerquart_firstpoint=[(innerquart[i][0]+innerquart_secondpoint[0])/2,
            (innerquart[i][1]+innerquart_secondpoint[1])/2,(innerquart[i][2]+innerquart_secondpoint[2])/2]
            innerquart_thirdpoint=[(innerquart[i+1][0]+innerquart_secondpoint[0])/2,
            (innerquart[i+1][1]+innerquart_secondpoint[1])/2,(innerquart[i+1][2]+innerquart_secondpoint[2])/2]
            innerquart.insert(i+1,innerquart_thirdpoint)
            innerquart.insert(i+1,innerquart_secondpoint)
            innerquart.insert(i+1,innerquart_firstpoint)

            outerquart_secondpoint=[(outerquart[i][0]+outerquart[i+1][0])/2,
            (outerquart[i][1]+outerquart[i+1][1])/2,(outerquart[i][2]+outerquart[i+1][2])/2]
            outerquart_firstpoint=[(outerquart[i][0]+outerquart_secondpoint[0])/2,
            (outerquart[i][1]+outerquart_secondpoint[1])/2,(outerquart[i][2]+outerquart_secondpoint[2])/2]
            outerquart_thirdpoint=[(outerquart[i+1][0]+outerquart_secondpoint[0])/2,
            (outerquart[i+1][1]+outerquart_secondpoint[1])/2,(outerquart[i+1][2]+outerquart_secondpoint[2])/2]
            outerquart.insert(i+1,outerquart_thirdpoint)
            outerquart.insert(i+1,outerquart_secondpoint)
            outerquart.insert(i+1,outerquart_firstpoint)

            inside_secondpoint=[(inside[i][0]+inside[i+1][0])/2,
            (inside[i][1]+inside[i+1][1])/2,(inside[i][2]+inside[i+1][2])/2]
            inside_firstpoint=[(inside[i][0]+inside_secondpoint[0])/2,
            (inside[i][1]+inside_secondpoint[1])/2,(inside[i][2]+inside_secondpoint[2])/2]
            inside_thirdpoint=[(inside[i+1][0]+inside_secondpoint[0])/2,
            (inside[i+1][1]+inside_secondpoint[1])/2,(inside[i+1][2]+inside_secondpoint[2])/2]
            inside.insert(i+1,inside_thirdpoint)
            inside.insert(i+1,inside_secondpoint)
            inside.insert(i+1,inside_firstpoint)

            outside_secondpoint=[(outside[i][0]+outside[i+1][0])/2,
            (outside[i][1]+outside[i+1][1])/2,(outside[i][2]+outside[i+1][2])/2]
            outside_firstpoint=[(outside[i][0]+outside_secondpoint[0])/2,
            (outside[i][1]+outside_secondpoint[1])/2,(outside[i][2]+outside_secondpoint[2])/2]
            outside_thirdpoint=[(outside[i+1][0]+outside_secondpoint[0])/2,
            (outside[i+1][1]+outside_secondpoint[1])/2,(outside[i+1][2]+outside_secondpoint[2])/2]
            outside.insert(i+1,outside_thirdpoint)
            outside.insert(i+1,outside_secondpoint)
            outside.insert(i+1,outside_firstpoint)
        return inside + innerquart + mid + outerquart + outside

    def _projectTo2D(self, data):
        """
        """
        result = []
        for point in data:
            result.append(point[:2])
        return result
