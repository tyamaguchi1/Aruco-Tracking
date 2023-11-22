# Import libraries
import matplotlib.pyplot as plt
import numpy as np
import math
from multiprocessing import Queue

"""
Usage Instructions

imports:
from multiprocessing import Process, Queue
import ThreadedGraphModule

spawning Animated Graph:
For multiprocessing, your actual main function needs to be in any defined function.
ex. def mainFunction():

Include the following:
if __name__ == '__main__':
    queue = Queue()
    T1 = Process(target=ThreadedGraphModule.CreateAnimatedGraph, args=(queue, xmin,xmax,ymin,ymax))
    T1.start()
    T2 = Process(target=mainFunction, args=(queue,))
    T2.start()

Fill in mainFunction, xmin, xmax, ymin, ymax with your main function and the desired plot dimensions respectively.
mainFunction() needs to accept queue as an argument to communicate with the Animated Graph

When the program starts, the graph and your main function will be spawned in seperate processes. The graph will scan queue for data,
while the main function should add data to queue.

Adding data to queue:
queue.put(data)
where data is of the form: [string signifier, package (optional)]

Legal signifiers and packages:
End: no package, kills the graph. Put when shutting down program programmatically.
PermPlot: adds a new function that will be permanently displayed on the graph (useful for setting limits or graphics that don't change)
    package: [x, y, color/type]
    ex: ["PermPlot", [x, 2*x+5, "r-"]]
    Note: Use numpy.linspace to get a range of values to execute the function upon (in the above example, x would be a linespace)
PermPoint: adds a new point that will be permanently displayed on the graph
    package: [x, y, size, color]
    ex: ["PermPoint", [10, 20, 2, "g"]]
Update: Sets or overrides a specific key-associated plot or point. Used for data that changes over time. Unless the key is updated,
    its graph will be static
    package: ["key", [type, [type-associated data]]]
    type can be either "point" or "plot" and require [x, y, color/type] or [x, y, size, color] as data respectively
    Ex1: ["Update", ["time", ["point", [15, 40, 5, "b"]]]]
    Ex2: ["Update", ["PastFiveSeconds", ["plot", [x, x**2, "y:"]]]]

For a complete example, see ThreadedGraphingExample.py
"""


#Important!: Only one Animated Graph should be on a given queue, or changes will not sync properly
#Also Important!: Each Animated Graph should be run in its own thread or it will block the rest of the program
def CreateAnimatedGraph(queue, lowX, highX, lowY, highY, name = "Graph", xlabel = "x-axis", ylabel = "y-axis"):
    print("started", flush=True)
    fig = plt.figure(figsize = (14, 8))
    xmin = lowX
    xmax = highX
    ymin = lowY
    ymax = highY

    permPlots = []
    permPoints = []
    
    plt.title(name)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)

    #Stores changing values. Key is arbitrary string. value is [type, data], where type is "point" or "plot" and data is of the respective form of .plot()
    ChangingDict = {}
    while True:
        #print(queue.qsize())
        plt.clf()
        #Get any new data from queue - data is of the form [signifier, data] where signifier is a string and data's type is arbitrary
        #Will only parse up to 10 values each iteration - the graph will lag if data is being placed in the queue too quickly
        timeout = 0
        while timeout < 10 and queue.empty() is not True:
            timeout+=1
            term = queue.get()
            signifier = term[0]
            #Termination signifier
            if signifier == "End":
                print("End",flush=True)
                queue.put(["end"])
                return
            #PermPlot
            if signifier == "PermPlot":
                print("PermPlot",flush=True)
                data = term[1]
                permPlots.append(data)
            #PermPoint
            if signifier == "PermPoint":
                print("PermPoint",flush=True)
                data = term[1]
                permPoints.append(data)
            #advanced - limits
            #Changing data - upon receiving a changing value, update only it if it is in ChangingDict, or add it. Every tick, plot everything in ChangingDict
            if signifier == "Update":
                print("Update",flush=True)
                data = term[1]
                ChangingDict[data[0]] = data[1]
        if timeout >= 10:
            plt.text(xmax/2,ymax*0.9,"WARNING: Graph is delayed")

        #updating graph logic - plot anything that is cleared every tick
        for data in ChangingDict.values():
            pack = data[1]
            if data[0] == "point":
                plt.plot(pack[0], pack[1], marker="o", markersize=pack[2], markerfacecolor=pack[3])
            elif data[0] == "plot":
                plt.plot(pack[0], pack[1], pack[2])
        
        #chores
        for plot in permPlots:
            plt.plot(plot[0], plot[1], plot[2])
        for plot in permPoints:
            plt.plot(plot[0], plot[1], marker="o", markersize=plot[2], markerfacecolor=plot[3])
        plt.axis([xmin, xmax, ymin, ymax])
        plt.pause(.001)