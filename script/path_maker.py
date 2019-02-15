#!/usr/bin/python
# coding: UTF-8

import random
import numpy as np
from math import *
import matplotlib.pyplot as plt       
from matplotlib.widgets import Button
import itertools
import re

import sys
def fprintf(stream, format_spec, *args):
    stream.write(format_spec % args)

class GUI(object):
    def __init__(self):
        print "!You can use keyboard(left/right) or botton to play bag file."
        self.idx = 0
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim([-10,10])
        self.ax.set_ylim([-10,10])
        #self.ax.get_xaxis().set_visible(False)
        #self.ax.get_yaxis().set_visible(False)

        self.path = []
        self.path_new = []
        self.obst = []
        self.bpath = Button(plt.axes([0.35, 0.002, 0.15, 0.045]), 'path/obst')
        self.bpath.on_clicked(self.on_path)
        self.bpath_is_checked = False

        self.bsave = Button(plt.axes([0.55, 0.002, 0.15, 0.045]), 'save')
        self.bsave.on_clicked(self.on_save)
        
        self.bload = Button(plt.axes([0.75, 0.002, 0.15, 0.045]), 'load')
        self.bload.on_clicked(self.on_load)

        self.fig.canvas.mpl_connect('key_press_event', self.press)
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        
        plt.show()

    def on_click(self, event):
        if event.button == 1 and  event.inaxes == self.ax:
            if self.bpath_is_checked:
                self.path.append([event.xdata, event.ydata])
            else:
                self.obst.append([event.xdata, event.ydata])
        self.update()

    def on_save(self, event):
        file = open("path.txt","w")
        for p in self.path:
            fprintf(file, "PATH: %f %f\n", p[0],p[1]) 
        for o in self.obst:
            fprintf(file, "OBST: %f %f\n", o[0],o[1]) 
        print("save OK!")
        file.close()

    def on_load(self, event):
        self.path = []
        self.obst = []
        file = open("path.txt")
        lines = file.readlines()
        for line in lines:
            txt = re.split(" ", line)
            if(txt[0] == 'PATH:'):
                self.path.append([float(txt[1]), float(txt[2])])
            if(txt[0] == 'OBST:'):
                self.obst.append([float(txt[1]), float(txt[2])])
            if(txt[0] == 'PATH_NEW:'):
                self.path_new.append([float(txt[1]), float(txt[2])])
        print("load OK!")
        self.update()
        file.close()

    def on_path(self, event):
        if(self.bpath_is_checked == True):
            self.bpath_is_checked = False
            self.bpath.color = 'white'
            self.bpath.label.set_text("draw obst") # works
            #self.bpath.name = 'draw path'
        else:
            self.bpath_is_checked == False
            self.bpath_is_checked = True
            self.bpath.color = 'green'
            self.bpath.label.set_text("draw path") # works
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
        self.ax.set_xlim([-10,10])
        self.ax.set_ylim([-10,10])

        if len(self.path) != 0:
            path = np.array(self.path)
            self.ax.plot(path[:,0], path[:,1], c='b')
            self.ax.scatter(path[:,0], path[:,1], c='b')

        if len(self.path_new) != 0:
            path = np.array(self.path_new)
            self.ax.plot(path[:,0], path[:,1], c='r')
            self.ax.scatter(path[:,0], path[:,1], c='r')

        if len(self.obst) != 0:
            obst = np.array(self.obst)
            self.ax.scatter(obst[:,0], obst[:,1], c='k', s = 1000)


        plt.draw()

if __name__ == "__main__":
    gui = GUI()