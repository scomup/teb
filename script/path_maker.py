#!/usr/bin/python
# coding: UTF-8

import random
import numpy as np
from math import *
import matplotlib.pyplot as plt       
from matplotlib.widgets import Button
import itertools
import re
import subprocess
import yaml
import sys
win_size = 0.5
def fprintf(stream, format_spec, *args):
    stream.write(format_spec % args)

class GUI(object):
    def __init__(self):
        self.idx = 0
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim([-18*win_size,18*win_size])
        self.ax.set_ylim([-10*win_size,10*win_size])
        self.cur_pose = [0,0]

        self.path = []
        self.path_new = []
        self.obst = []

        self.bclear = Button(plt.axes([0.00, 0.002, 0.12, 0.045]), 'clear')
        self.bclear.on_clicked(self.on_clear)

        self.bpath = Button(plt.axes([0.35, 0.002, 0.12, 0.045]), 'draw path')
        self.bpath.on_clicked(self.on_path)
        self.bpath_is_checked = True
        self.bpath.color = 'green'
        self.bpath.hovercolor = self.bpath.color
        self.bpath.ax.set_facecolor(self.bpath.color)

        self.bsave = Button(plt.axes([0.50, 0.002, 0.12, 0.045]), 'save')
        self.bsave.on_clicked(self.on_save)
        
        self.bload = Button(plt.axes([0.65, 0.002, 0.12, 0.045]), 'load')
        self.bload.on_clicked(self.on_load)

        self.brun = Button(plt.axes([0.80, 0.002, 0.12, 0.045]), 'run')
        self.brun.on_clicked(self.on_run)

        self.fig.canvas.mpl_connect('key_press_event', self.press)
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect('button_release_event', self.on_release)

        self.load_config()
        plt.show()

    def load_config(self):
        config_file = open("/home/liu/workspace/teb/config/sample.yaml", "r")
        config = yaml.load(config_file)
        self.min_dist = config["min_obstacle_dist"]  
        self.model_name = config["footprint_model"]["type"]
        self.polygon = np.matrix(config["footprint_model"]["vertices"])



    def on_clear(self, event):
        self.path = []
        self.obst = []
        self.update()

    def on_run(self, event):
        #self.save("path.txt")
        subprocess.call(["/home/liu/workspace/teb/build/teb_demo"])
        self.load("new_path.txt")
        self.update()

    def on_click(self, event):
        if event.button == 1 and  event.inaxes == self.ax:
            if self.bpath_is_checked:
                self.cur_pose = [event.xdata, event.ydata]
            else:
                self.obst.append([event.xdata, event.ydata])
                self.update()

    def on_release(self, event):
        if event.button == 1 and  event.inaxes == self.ax:
            if self.bpath_is_checked:
                vector = [event.xdata-self.cur_pose[0], event.ydata-self.cur_pose[1]]
                theta = atan2(vector[1],vector[0])
                self.path.append([self.cur_pose[0], self.cur_pose[1], theta])
                self.update()

    def on_save(self, event):
        self.save("path.txt")
        print("save OK!")

    def save(self, file_name):
        file = open(file_name,"w")
        for p in self.path:
            fprintf(file, "PATH: %f %f %f\n", p[0],p[1],p[2]) 
        for o in self.obst:
            fprintf(file, "OBST: %f %f\n", o[0],o[1]) 
            
        file.close()

    def on_load(self, event):
        self.path = []
        self.obst = []
        self.load("path.txt")
        self.update()
        

    def load(self, file_name):
        self.load_config()
        self.path = []
        self.obst = []
        file = open(file_name)
        lines = file.readlines()
        for line in lines:
            txt = re.split(r'\s+', line)
            if(txt[0] == 'PATH:'):
                self.path.append([float(txt[1]), float(txt[2]), float(txt[3])])
            if(txt[0] == 'OBST:'):
                self.obst.append([float(txt[1]), float(txt[2])])
            if(txt[0] == 'PATH_NEW:'):
                self.path_new.append([float(txt[1]), float(txt[2])])
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
            self.obst = []
            self.path = []
            self.update()
            
    def update(self):
        self.ax.cla()
        self.ax.set_xlim([-18*win_size,18*win_size])
        self.ax.set_ylim([-10*win_size,10*win_size])

        if len(self.path_new) != 0:
            path = np.array(self.path_new)
            self.ax.plot(path[:,0], path[:,1], c='r')
            self.ax.scatter(path[:,0], path[:,1], c='r')

        if len(self.obst) != 0:
            obst = np.array(self.obst)
            self.ax.scatter(obst[:,0], obst[:,1], c='b')
            for o in obst:
                self.ax.add_patch(plt.Circle((o[0], o[1]), self.min_dist, color='b', alpha=0.1))


        if len(self.path) != 0:
            path = np.array(self.path)
            self.ax.plot(path[:,0], path[:,1], c='g',linewidth=0.5)
            vec = []
            for p in self.path:
                c, s = np.cos(p[2]), np.sin(p[2])
                R = np.matrix(((c,-s), (s, c)))
                v = R*np.matrix([[1],[0]])
                vec.append([v[0,0], v[1,0]])
                R = np.matrix(((c,-s), (s, c)))
                polygon = R * np.matrix(self.polygon.transpose())
                polygon = polygon.transpose()
                polygon = polygon + np.tile( np.array([p[0], p[1]]), (self.polygon.shape[0], 1))
                self.ax.add_patch(plt.Polygon(polygon, color='g', alpha=0.1))
                #plt.pause(0.1)
                #self.ax.cla()
                #self.ax.set_xlim([-18*win_size,18*win_size])
                #self.ax.set_ylim([-10*win_size,10*win_size])
            vec = np.array(vec)
            self.ax.quiver(path[:,0], path[:,1], vec[:,0], vec[:,1], color='g',width=0.001, scale= 30)


        plt.draw()

if __name__ == "__main__":
    gui = GUI()