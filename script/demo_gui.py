#!/usr/bin/python
# coding: UTF-8

import numpy as np
from math import *
import matplotlib.pyplot as plt       
from matplotlib.widgets import Button
import re
import subprocess
import yaml
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection


win_size = 10
def fprintf(stream, format_spec, *args):
    stream.write(format_spec % args)

class GUI(object):
    def __init__(self):
        self.idx = 0
        self.fig, self.ax = plt.subplots()
        self.ax.axis('equal')
        self.ax.set_xlim([-win_size,win_size])
        self.ax.set_ylim([-win_size,win_size])
        self.cur_pose = [0,0]
        self.play_cnt = -1
        self.paths = []
        self.obsts = []

        self.bclear = Button(plt.axes([0.00, 0.002, 0.11, 0.045]), 'clear')
        self.bclear.on_clicked(self.on_clear)

        self.bpath = Button(plt.axes([0.33, 0.002, 0.11, 0.045]), 'draw path')
        self.bpath.on_clicked(self.on_path)
        self.bpath_is_checked = True
        self.bpath.color = 'green'
        self.bpath.hovercolor = self.bpath.color
        self.bpath.ax.set_facecolor(self.bpath.color)

        self.current_points = []

        self.bsave = Button(plt.axes([0.45, 0.002, 0.11, 0.045]), 'save')
        self.bsave.on_clicked(self.on_save)
        
        self.bload = Button(plt.axes([0.57, 0.002, 0.11, 0.045]), 'load')
        self.bload.on_clicked(self.on_load)

        self.brun = Button(plt.axes([0.69, 0.002, 0.11, 0.045]), 'run')
        self.brun.on_clicked(self.on_run)

        self.bplay = Button(plt.axes([0.81, 0.002, 0.11, 0.045]), 'play')
        self.bplay.on_clicked(self.on_play)


        self.fig.canvas.mpl_connect('key_press_event', self.press)
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect('button_release_event', self.on_release)
        self.timer = self.fig.canvas.new_timer(interval=100)
        self.timer.add_callback(self.play_update)
        self.timer.start()

        self.load_config()
        plt.show()

    def play_update(self):
        if self.play_cnt != -1:
            self.update()
            self.play_cnt += 1
            if(self.play_cnt > len(self.paths)):
                self.play_cnt = -1


    def load_config(self):
        config_file = open("../config/sample.yaml", "r")
        config = yaml.load(config_file,Loader=yaml.Loader)
        self.min_dist = config["min_obstacle_dist"]  
        self.model_name = config["footprint_model"]["type"]
        self.polygon = np.matrix(config["footprint_model"]["vertices"])

    def on_clear(self, event):
        self.paths = []
        self.obsts = []
        self.update()

    def on_play(self, event):
        self.play_cnt = 0

    def on_run(self, event):
        print("here1")
        subprocess.call(["../build/teb_demo", "path.txt", "../config/sample.yaml"])
        self.load("new_path.txt")
        self.update()

    def on_click(self, event):
        if event.button == 1 and  event.inaxes == self.ax:
            if self.bpath_is_checked:
                self.cur_pose = [event.xdata, event.ydata]
            else:
                if event.key == 'shift':
                    self.current_points.append([event.xdata, event.ydata])   
                else:
                    self.obsts.append([[event.xdata, event.ydata]])
                    self.update()
        elif event.button == 3 and  event.inaxes == self.ax: #right click to close polygon
            self.obsts.append(self.current_points)
            self.current_points = []
        self.update()

                
    def on_release(self, event):
        if event.button == 1 and  event.inaxes == self.ax:
            if self.bpath_is_checked:
                vector = [event.xdata-self.cur_pose[0], event.ydata-self.cur_pose[1]]
                theta = atan2(vector[1],vector[0])
                self.paths.append([self.cur_pose[0], self.cur_pose[1], theta])
                self.update()

    def on_save(self, event):
        self.save("path.txt")
        print("save OK!")

    def save(self, file_name):
        file = open(file_name,"w")
        for pose in self.paths:
            fprintf(file, "PATH: %f %f %f\n", pose[0],pose[1],pose[2]) 
        for obst in self.obsts:
            fprintf(file, "OBST:") 
            for point in obst:
                fprintf(file, " %f %f", point[0], point[1]) 
            fprintf(file, "\n") 
            
        file.close()

    def on_load(self, event):
        self.paths = []
        self.obsts = []
        self.load("path.txt")
        self.update()
        

    def load(self, file_name):
        self.load_config()
        self.paths = []
        self.obsts = []
        file = open(file_name)
        lines = file.readlines()
        for line in lines:
            txt = re.split(r'\s+', line)
            if(txt[0] == 'PATH:'):
                self.paths.append([float(txt[1]), float(txt[2]), float(txt[3])])
            if(txt[0] == 'OBST:'):
                obst = []
                for i in range(1, len(txt)):
                    try:
                        obst.append(float(txt[i]))
                    except:
                        pass
                obst = np.array(obst).reshape(-1,2)
                self.obsts.append(obst)
        file.close()

    def on_path(self, event):
        if(self.bpath_is_checked == True):
            self.bpath_is_checked = False
            self.bpath.color = 'blue'
            self.bpath.label.set_text("draw obst") 
            #self.bpath.name = 'draw path'
        else:
            self.bpath_is_checked == False
            self.bpath_is_checked = True
            self.bpath.color = 'green'
            self.bpath.label.set_text("draw path")
            #self.bpath.name = 'draw obst'

        self.bpath.hovercolor = self.bpath.color
        self.bpath.ax.set_facecolor(self.bpath.color)
        self.fig.canvas.draw()
        self.update()


    def press(self, event):
        if event.key == 'r':
            self.obsts = []
            self.paths = []
            self.update()
            
    def update(self):
        self.ax.cla()
        self.ax.set_xlim([-win_size,win_size])
        self.ax.set_ylim([-win_size,win_size])

        if len(self.obsts) != 0:
            for obst in self.obsts:
                obst = np.array(obst)
                if(obst.shape[0] == 1):
                    self.ax.scatter(obst[0,0], obst[0,1], c='b')
                elif(obst.shape[0] == 2):
                    self.ax.plot(obst[:,0], obst[:,1], 'b-')
                elif(obst.shape[0] >= 3):
                    polygon = Polygon(obst, closed=True)
                    self.ax.add_patch(polygon)

        if len(self.current_points) != 0:
            current_points = np.array(self.current_points)
            self.ax.plot(current_points[:,0], current_points[:,1], 'b-')
            self.ax.scatter(current_points[:,0], current_points[:,1], c='b')

        self.draw_paths()

        plt.draw()

    def draw_paths(self):
        if(len(self.paths) == 0):
            return
        
        n = self.play_cnt
        if n > len(self.paths) or n == -1:
            n = len(self.paths)

        path = np.array(self.paths)
        self.ax.plot(path[0:n,0], path[0:n,1], c='g',linewidth=0.5)
        vec = []
        for p in path[0:n,:]:
            c, s = np.cos(p[2]), np.sin(p[2])
            R = np.matrix(((c,-s), (s, c)))
            v = R*np.matrix([[1],[0]])
            vec.append([v[0,0], v[1,0]])
            R = np.matrix(((c,-s), (s, c)))
            polygon = R * np.matrix(self.polygon.transpose())
            polygon = polygon.transpose()
            polygon = polygon + np.tile( np.array([p[0], p[1]]), (self.polygon.shape[0], 1))
            self.ax.add_patch(plt.Polygon(polygon, color='g', alpha=0.1))
        vec = np.array(vec)

        #self.ax.quiver(path[0:n,0], path[0:n,1], vec[0:n,0], vec[0:n,1], color='g',width=0.001, scale= 30)


if __name__ == "__main__":
    print("Hold 'Shift' while clicking the left mouse button to add a point for polygon or line.")
    print("Clicking the right mouse button to close a polygon or line.")
    gui = GUI()