from enum import Enum
from tkinter import *
import Tkinter, Tkconstants, tkFileDialog, tkSimpleDialog

import matplotlib.colors as mc
import colorsys


class CellAttributes(Enum):
    BLOCKED=1
    EMPTY=2
    GOAL=3
    FAILSTATE=4
    DOOR=5
    AVOID=6
    INITIALLOC=7
    

class ShapePoints():
    
    @staticmethod
    def getStarPoints(sxc,syc,p,t):
        points = []
        for i in (1,-1):
            points.extend((sxc,syc+i*p))
            points.extend((sxc+i*t,syc+i*t))
            points.extend((sxc+i*p,syc))
            points.extend((sxc+i*t,syc-i*t))
        return points

    @staticmethod
    def getCrossPoints(sx,sy,ex,ey):
        xlen =abs (ex - sx)
        ylen = abs(ey - sy )
        #cross points
        crosspoints = [(0,0),(0.2,0),(0.5,0.3),(0.8,0),(1,0),(1,0.2),(0.7,0.5),(1,0.8),(1,1),(0.8,1),(0.5,0.7),(0.2,1),(0,1),(0,0.8),(0.3,0.5),(0,0.2)]
        #first we've got to scale all the points
        minscale = min(xlen,ylen)
        mcrosspoints = []
        for t in crosspoints:
            mt = (t[0]*minscale+sx,t[1]*minscale+sy)
            mcrosspoints.append(mt)
        return mcrosspoints

    
class ColourHelper():
    
    @staticmethod
    def mixColor(color1,color2):
        if color1 is None:
            return color2
        if color2 is None:
            return color1
        
        rgbcolor = mc.to_rgb(color1)
        rgbcolor2 = mc.to_rgb(color2)
        mixedcolor = ((rgbcolor2[0]+rgbcolor[0])/2,(rgbcolor2[1]+rgbcolor[1])/2,(rgbcolor2[2]+rgbcolor[2])/2)
        mixedcolorstr = tuple([255*x for x in mixedcolor])
        mixedcolorstr = '#%02x%02x%02x' % mixedcolorstr
        return mixedcolorstr
    
        
    #from https://stackoverflow.com/questions/37765197/darken-or-lighten-a-color-in-matplotlib
    @staticmethod
    def lighten_color(color, amount=0.5):
        """
        Lightens the given color by multiplying (1-luminosity) by the given amount.1
        Input can be matplotlib color string, hex string, or RGB tuple.

        Examples:
        >> lighten_color('g', 0.3)
        >> lighten_color('#F034A3', 0.6)
        >> lighten_color((.3,.55,.1), 0.5)
        """


        #print color
        try:
            c = mc.cnames[color]
        except:
            c = color
        c = colorsys.rgb_to_hls(*mc.to_rgb(c))
        newc=colorsys.hls_to_rgb(c[0], 1 - amount * (1 - c[1]), c[2])
        #print newc
        newc=tuple([255*x for x in newc])
        #print newc
        newcstr= '#%02x%02x%02x' % newc
        #print newcstr
        return newcstr



        
class GridFileReader(object):
    def readGridFile(self,fn):
        with open(fn) as gfile:
            lines = gfile.readlines()

            rowcolumnsize = lines[0].replace('\n','')
            rowcolumnsize = rowcolumnsize.split(',')
            
            row = int(rowcolumnsize[0])
            column = int(rowcolumnsize[1])
            size = int(rowcolumnsize[2])
            rowNumber = row
            columnNumber = column
            cellSize = size
            xydict = {}
            for i in range(1,len(lines)):
                line = lines[i].replace('\n','')
                xys = line.split('*')
                flags = xys[1].replace(']','')
                xys = xys[0]
                xys = xys.replace('[','')
                xys = xys.replace('(','')
                xys = xys.replace(')','')
                xys = xys.split(',')
                
                xydict[(int(xys[0]),int(xys[1]))] = eval(flags)
                
        return (row,column,size,xydict)



class GridGuiStaTraDialog(tkSimpleDialog.Dialog):
    defaultFolder="/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/smallerwhdoors/results/logs/debugRes/extras"
    def body(self,master):
        self.stapustaname = None
        self.staputraname = None
        self.ssistaname = None
        self.ssitraname = None
        self.addressToOpen = GridGuiStaTraDialog.defaultFolder
        #print(defaultFolder)
        #self.addressToOpen = defaultAddress 
        defaultAddressVar = StringVar(master,value=self.addressToOpen)
        
        Label(master,text="STAPU File:").grid(row=0,sticky=W)
        self.stapufilebox = Entry(master,textvariable=defaultAddressVar)
        self.stapufilebox.grid(row=0,column=1)
        self.stapufilebutton = Button(master,text='Open Folder',command=self.openStapuFile)
        self.stapufilebutton.grid(row=0,column=2)
        
        Label(master,text="Auctioning File:").grid(row=1,sticky=W)
        self.ssifilebox = Entry(master,textvariable=defaultAddressVar)
        self.ssifilebox.grid(row=1,column=1)
        self.ssifilebutton = Button(master,text='Open Folder',command=self.openSsiFile)
        self.ssifilebutton.grid(row=1,column=2)
        
        

    def openFile(self):
        fn = tkFileDialog.askopenfilename(initialdir=self.addressToOpen,title="Open sta/tra File", filetypes=(("sta",".sta"),("tra",".tra"),("all files","*.*")))
        staname = None
        traname = None
        
        if '.sta' in fn:
            staname = fn
            traname = fn.replace('.sta','.tra')
        elif '.tra' in fn:
            traname = fn
            staname = fn.replace('.tra','.sta')
        else:
            print("Invalid file extension")
        return (staname,traname)

    def openStapuFile(self):
        (staname,traname) = self.openFile()
        if staname is not None:
            address = staname[0:staname.rfind('/')]
            self.addressToOpen = address
            self.stapustaname = staname
            self.staputraname = traname
            self.stapufilebox.delete(0,END)
            self.stapufilebox.insert(0,staname)


    def openSsiFile(self):
        (staname,traname) = self.openFile()
        if staname is not None:
            address = staname[0:staname.rfind('/')]
            self.addressToOpen = address
            self.ssistaname = staname
            self.ssitraname = traname
            self.ssifilebox.delete(0,END)
            self.ssifilebox.insert(0,staname)
            

    def apply(self):
        print ("STAPU:\n"+self.stapustaname+"\n"+self.staputraname)
        print ("Auctioning:\n"+self.ssistaname+"\n"+self.ssitraname)
        print ("All Done")
            
