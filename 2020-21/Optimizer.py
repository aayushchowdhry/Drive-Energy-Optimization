import math
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from TrackNodeKDTree import *
from TrackGraph import *
from SpeedProfileFinder import *
import random

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

inside=[];out=[];
for i in range(0,361,10):
    inside.append([cosd(i),sind(i),sind(i)])
for i in range(0,361,10):
    out.append([5*cosd(i),5*sind(i),cosd(i)])
inside.reverse(); out.reverse()

class PlotGraph:

    def __init__(self, graph, line = False, label = None):
        """
        """
        fig = plt.figure()
        ax = plt.axes(projection='3d')
        x=[node.x for node in graph]
        y=[node.y for node in graph]
        z=[node.z for node in graph]
        if line:
            ax.plot_trisurf(x, y, z, color='white', cmap='viridis', edgecolor='none')
        else:
            ax.scatter(x, y, z, label = label, s=1)
        ax.set_xlim(min(x)-0.1,max(x)+0.1)
        ax.set_ylim(min(y)-0.1,max(y)+0.1)
        ax.set_zlim(min(z)-0.1,max(z)+0.1)
        ax.set_xlabel('x - axis')
        ax.set_ylabel('y - axis')
        ax.set_zlabel('z - axis')
        ax.view_init(60, 35)
        plt.title('Track')
        self.kdTree = TrackNodeKDTree(graph)
        self.firstPoint = None
        self.ax = ax
        self.fig = fig
        self.arrows = []
        cid = fig.canvas.mpl_connect('button_press_event', self.onclick)
        plt.show()


    def onclick(self,event):
        b = self.ax.button_pressed
        self.ax.button_pressed = -1
        s = self.ax.format_coord(event.xdata,event.ydata)
        self.ax.button_pressed = b
        out = ""
        for i in range(s.find('x')+2,s.find('y')-2):
            out = out+s[i]
        xdata = float(out)
        out = ""
        for i in range(s.find('y')+2,s.find('z')-2):
            out = out+s[i]
        ydata = float(out)
        print(xdata,ydata)
        if self.firstPoint is None:
            for arrow in self.arrows:
                arrow[0].remove()
            self.arrows = []
            self.firstPoint = (xdata, ydata)
            self.fig.canvas.draw()
            print("Awaiting second point...")
        else:
            path, en = optimumPath(self.kdTree, self.firstPoint, (xdata, ydata))
            X = np.array([a.x for a in path]); Y = np.array([a.y for a in path]); Z = np.array([a.z for a in path])
            for i in range(len(path)):
                if i == len(path)-1:
                    pass
                    #plt.arrow(path[i].x, path[i].y, path[0].x - path[i].x, path[0].y - path[i].y)
                else:
                    self.arrows.append(self.ax.plot([node.x for node in path], [node.y for node in path], [node.z for node in path]))
            self.firstPoint = None
            self.fig.canvas.draw()
            print("Energy optimum path at average speed displayed.")

PlotGraph(createGraph(interpolate(inside, out)), line=True)

#print(energy(TrackNode(0, 0 , 0), TrackNode(1, 1, 1), 1))
