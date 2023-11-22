import numpy as np
import math
import ThreadedGraphModule as TGM
from multiprocessing import Process, Queue
import time as timer

def Controller(queue):
    time = 0
    x = np.linspace(0, 100, 50)
    queue.put(["PermPlot", [x, x**2 / 10, "y:"]])
    while time**2 / 10 <100:
        time+=0.1
        print(time)
        queue.put(["Update", ["time", ["point", [time, time**2 / 10, 5, "r"]]]])
        timer.sleep(0.1)
    queue.put(["End"])



if __name__ == '__main__':
    queue = Queue()
    T1 = Process(target=TGM.CreateAnimatedGraph, args=(queue, 0,100,0,100))
    T1.start()
    T2 = Process(target=Controller, args=(queue,))
    T2.start()
    