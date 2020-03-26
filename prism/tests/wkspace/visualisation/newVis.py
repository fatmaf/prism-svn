#a completely new visualisation
#that uses the old stuff but is just better written
#steps
#read the grid file
#draw the basics: goals,doors,failstates,obstacles,robots
#update the goals from file
#update the robots from sta tra file
#end

#classes
#a class that reads the grid file
#a class that reads an sta tra file
#a class that handles the app stuff
#so we could have two of the same running side by side
#a bigger app which loads the grids etc


from tkinter import *
import Tkinter, Tkconstants, tkFileDialog, tkSimpleDialog

import math
from enum import Enum

import matplotlib.colors as mc
import colorsys


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


class CellAttributes(Enum):
    BLOCKED=1
    EMPTY=2
    GOAL=3
    FAILSTATE=4
    DOOR=5
    AVOID=6
    INITIALLOC=7

class Path(object):
    def __init__(self,pathNumber,firstState,cell,canvas,color):
        self.pathnum = pathNumber
        self.states = [firstState]
        self.cells = {firstState:cell}
        self.statesDrawn = []
        self.canvas = canvas
        self.color = color
        self.doLines = False

    def fadePath(self,agnum=0,agsize=None):
        self.color = ColourHelper.lighten_color(self.color)
        self.clearPath()
        self.drawPath(agnum,agsize)
    
    def addStateCell(self,state,cell):
        self.states.append(state)
        if state not in self.cells:
            self.cells[state] = cell

    def getAgentCenter(self,cell,agsize,agnum):
        if agsize is None:
            return cell.getCellCoords()
        else:
            (xmin,xmax,ymin,ymax,xc,yc) = cell.getCellCoords()
            agxy = Agent.agnumtoxy[agnum]
            xpos = xmin+agxy[0]*agsize
            ypos = ymin+agxy[1]*agsize
            xposl = xpos+agsize
            yposl = ypos+agsize
            xposc = (xpos+xposl)/2
            yposc = (ypos+yposl)/2

        return (xpos,ypos,xposl,yposl,xposc,yposc)


    def clearPath(self):
        for obj in self.statesDrawn:
            self.canvas.delete(obj)
        self.statesDrawn = [] 

    def shiftPath(self,shiftby):
        #we get the path number
        for obj in self.statesDrawn:
            #shift by x
            if obj is not None:
                #coords = self.canvas.coords(obj)
                #newcoords = (coords[0]+shiftby,coords[1])
                #self.canvas.coords(obj,newcoords)
                self.canvas.move(obj,0,shiftby)

    def drawPath(self,agnum=0,agsize=None):
        if self.doLines:
            self.drawPathLines(agnum,agsize)
        else:
            self.drawPathCircles(agnum,agsize)

    def drawPathCircles(self,agnum=0,agsize=None):
        dotsize = 2
        if agsize is not None:
            dotsize = agsize/10 
        for i in range(len(self.states)):
            if len(self.statesDrawn) < (i+1):
                cs = self.states[i]
                cscell = self.cells[cs]
                csxy=cscell.getXY()
                (xmin,xmax,ymin,ymax,xc,yc) = self.getAgentCenter(cscell,agsize,agnum)
                
                obj = self.canvas.create_oval(xc-dotsize,yc-dotsize,xc+dotsize,yc+dotsize,fill=self.color,outline='white')
                self.statesDrawn.append(obj)
                
    def drawPathLines(self,agnum=0,agsize=None):
        w = 2
        #print ("Moving from states ")
        #print (self.states)
        for i in range(len(self.states)-1):
            #we want to draw a path for each state
            #first we must determine whether we have drawn this state or not
            #print ("Anything we've drawn")
            #print (self.statesDrawn)
            if len(self.statesDrawn) < (i+1):
                #we have not drawn this state
                cs = self.states[i]
                ns = self.states[i+1]
                cscell = self.cells[cs]
                nscell = self.cells[ns]
                csxy = cscell.getXY()
                nsxy = nscell.getXY()
                dir = 'none'
                if (csxy[1] == nsxy[1]):
                    if (csxy[0] < nsxy[0]):
                        dir = 'right'
                    elif (csxy[0]>nsxy[0]):
                        dir = 'left'
                elif (csxy[0] == nsxy[0]):
                    if(csxy[1] < nsxy[1]):
                        dir = 'up'
                    elif (csxy[1] > nsxy[1]):
                        dir = 'down'
                (xmin,xmax,ymin,ymax,xc,yc) = self.getAgentCenter(cscell,agsize,agnum)
                (xminn,xmaxn,yminn,ymaxn,xcn,ycn) = self.getAgentCenter(nscell,agsize,agnum)
                obj = None
                #print ("Direction : "+dir)
                if dir == 'up' or dir == 'down':
                    #draw a line in the middle xc from one yc to the other
                    obj = self.canvas.create_line(xc,yc,xc,ycn,width=w,fill=self.color)
                elif dir == 'right' or dir == 'left':
                    obj = self.canvas.create_line(xc,yc,xcn,yc,width=w,fill=self.color)

                #print ("We've drawn this now ")
                #print (obj)
                self.statesDrawn.append(obj)
                
                
                        
                
        
class Goal(object):
    def __init__(self,canvas,cell,loc):
        self.canvas = canvas
        self.cell = cell
        fill = Cell.FILLCOLOURS[CellAttributes.GOAL]
        self.visitedFill = ColourHelper.mixColor(fill,'red')
        self.visitFill = ColourHelper.mixColor(fill,'yellow')
        self.outline = Cell.BORDERCOLOURS[CellAttributes.GOAL]
        self.drawGoal = True
        self.visited = False
        self.numVisits = 0
        self.maxLen = cell.size/4
        self.maxWid = cell.size/8
        self.canvasobj = None
        
        

    def draw(self):
        if self.drawGoal:
            fill = self.visitFill
            if self.visited:
                fill=self.visitedFill
            (xmin,xmax,ymin,ymax,xc,yc) = self.cell.getCellCoords()
            points = ShapePoints.getStarPoints(xc,yc,self.maxLen,self.maxWid)
            if self.canvasobj is not None:
                self.canvas.delete(self.canvasobj)
            self.canvas.create_polygon(points,fill=fill,outline=self.outline)
            self.canvas.lift(self.cell.labelObj)

    def setVisited(self):
        if not self.visited:
            self.visited = True
            self.draw()

    def unsetVisited(self):
        if self.visited:
            self.visited = False
            self.draw()
        
    
class Agent(object):
    agnumtoxy={0:(0,0),1:(0,1),2:(0,2),3:(1,0),4:(1,2),5:(2,0),6:(2,1),7:(2,2),8:(0,0),9:(0,1)}
    def __init__(self,canvas,cell,size,currloc,agcolor,num):
        self.canvas = canvas
        self.currloc = currloc
        self.cell = cell 
        self.size = size
        self.deadCounter = 0
        self.dead = False
        self.drawAgent = False
        self.colour = agcolor
        self.canvasobj = {'shape':None,'number':None}
        self.num = num
        self.paths = {}

    def getAgentCenter(self):
        return self.getAgentCenterCell(self.cell)
        
    def getAgentCenterCell(self,cell):
        agsize = self.size
        (xmin,xmax,ymin,ymax,xc,yc) = cell.getCellCoords()
        agnum = self.num 
        agxy = Agent.agnumtoxy[agnum]
        xpos = xmin+agxy[0]*agsize
        ypos = ymin+agxy[1]*agsize
        xposl = xpos+agsize
        yposl = ypos+agsize
        xposc = (xpos+xposl)/2
        yposc = (ypos+yposl)/2

        return (xpos,ypos,xposl,yposl,xposc,yposc)
  
    def draw(self):
        if self.drawAgent:
            (xpos,ypos,xposl,yposl,xposc,yposc) = self.getAgentCenter()
            #agsize = self.size
            #(xmin,xmax,ymin,ymax,xc,yc) = self.cell.getCellCoords()
            #agnum = self.num 
            #agxy = Agent.agnumtoxy[agnum]
            #xpos = xmin+agxy[0]*agsize
            #ypos = ymin+agxy[1]*agsize
            #xposl = xpos+agsize
            #yposl = ypos+agsize
            fill = self.colour #Cell.FILLCOLOURS[CellAttributes.INITIALLOC]
            outline='white'
            if self.dead:
                crosspoints = ShapePoints.getCrossPoints(xpos,ypos,xposl,yposl)
                self.canvasobj['shape']=self.canvas.create_polygon(cp,fill=fill,outline=outline)
            else:
                self.canvasobj['shape']=self.canvas.create_oval(xpos,ypos,xposl,yposl,fill=fill,outline=outline)
            #xposc = (xpos+xposl)/2
            #yposc = (ypos+yposl)/2
            self.canvasobj['number']=self.canvas.create_text((xposc,yposc),text=str(self.deadCounter))

    def getMoveLimits(self,nextcell):
        (xmin,xmax,ymin,ymax,xc,yc) = self.cell.getCellCoords()
        (xminn,xmaxn,yminn,ymaxn,xcn,ycn) = nextcell.getCellCoords()
        xmove = xminn-xmin
        ymove = yminn-ymin
        return (xmove,ymove)
    
    def updateCell(self,nextloc,nextcell,currentPath=0):
        if currentPath not in self.paths:
            self.paths[currentPath] = Path(currentPath,self.currloc,self.cell,self.canvas,self.colour)
        self.paths[currentPath].addStateCell(nextloc,nextcell)
        
        #self.paths[currentPath].append({'loc':self.currloc,'cell':self.currcell,'ncell':nextcell}]
        (xmove,ymove) = self.getMoveLimits(nextcell)
        self.currloc = nextloc
        self.cell = nextcell
        #move the agent
        self.canvas.move(self.canvasobj['shape'],xmove,ymove)
        self.canvas.move(self.canvasobj['number'],xmove,ymove)
        self.drawAllPaths()

    def drawAllPaths(self):
        for p in self.paths:
            self.paths[p].drawPath(self.num,self.size)

    def shiftAllPaths(self,shiftby):
        for p in self.paths:
            self.paths[p].fadePath(self.num,self.size)
            self.paths[p].shiftPath(shiftby)
            
        
    
        
class Cell(object):

    FILLCOLOURS = {CellAttributes.BLOCKED:'black',CellAttributes.EMPTY:'white',CellAttributes.GOAL:'green',CellAttributes.AVOID:'red',CellAttributes.FAILSTATE:'lightgrey',CellAttributes.DOOR:'purple',CellAttributes.INITIALLOC:'blue'}

    BORDERCOLOURS={CellAttributes.BLOCKED:'black',CellAttributes.EMPTY:'black',CellAttributes.GOAL:'green',CellAttributes.AVOID:'red',CellAttributes.FAILSTATE:'grey',CellAttributes.DOOR:'purple',CellAttributes.INITIALLOC:'blue'}
        
    def __init__(self,canvas,x,y,size):
        self.canvas = canvas
        self.abs = x
        self.ord = y
        self.size = size

        self.cellAttributes=[]
        
        self.otherDoor = None
        self.label = str(y)+","+str(x)
        self.defaultLabel = str(y)+","+str(x)
        self.rectangle = None
        self.labelObj = None
        

    def getXY(self):
        return (self.abs,self.ord)
    
    def clearAllFlags(self):
        self.cellAttributes=[]
        self.otherDoor = None

    def clearGoal(self):
        self.cellAttributes.remove(CellAttributes.GOAL)

    def clearInitialLocation(self):
        self.cellAttributes.remove(CellAttributes.INITIALLOC)

    def drawAvoid(self,xmin,ymin,xmax,ymax,defaultfill,defaultoutline):
        circlefill = Cell.FILLCOLOURS[CellAttributes.AVOID]
        ovaldist = (xmax-xmin)/10
        
        self.canvas.create_oval(xmin,ymin,xmax,ymax,fill=circlefill,outline=defaultoutline)
        self.canvas.create_oval(xmin+ovaldist,ymin+ovaldist,xmax-ovaldist,ymax-ovaldist,fill=defaultfill,outline=defaultoutline)

        self.canvas.create_line(xmin+2*ovaldist,ymin+2*ovaldist,xmax-2*ovaldist,ymax-2*ovaldist,width=ovaldist,fill=circlefill)

    def drawGoal(self,xc,yc,visited=False):
        starMaxLen = self.size/4
        starTinyLen = self.size/8
        starfill = Cell.FILLCOLOURS[CellAttributes.GOAL]
        staroutline = Cell.BORDERCOLOURS[CellAttributes.GOAL]
        if visited:
            starfill = ColourHelper.mixColor(starfill,'red')
        else:
            starfill = ColourHelper.mixColor(starfill,'yellow')
        starpoints = ShapePoints.getStarPoints(xc,yc,starMaxLen,starTinyLen)
        self.canvas.create_polygon(starpoints,fill=starfill,outline=staroutline)

    def drawAgent(self,xmin,ymin,agnum,deadCounter=0,dead=False):
        agsize = self.size/3
        agnumtoxy={0:(0,0),1:(0,1),2:(0,2),3:(1,0),4:(1,2),5:(2,0),6:(2,1),7:(2,2)}
        agxy = agnumtoxy[agnum]
        xpos = xmin+agxy[0]*agsize
        ypos = ymin+agxy[1]*agsize
        xposl = xpos+agsize
        yposl = ypos+agsize
        fill = Cell.FILLCOLOURS[CellAttributes.INITIALLOC]
        outline=Cell.BORDERCOLOURS[CellAttributes.INITIALLOC]
        if dead:
            crosspoints = ShapePoints.getCrossPoints(xpos,ypos,xposl,yposl)
            self.canvas.create_polygon(cp,fill=fill,outline=outline)
        else:
            self.canvas.create_oval(xpos,ypos,xposl,yposl,fill=fill,outline=outline)
        xposc = (xpos+xposl)/2
        yposc = (ypos+yposl)/2
        self.canvas.create_text((xposc,yposc),text=str(deadCounter))
            
        

    def getCellCoords(self):
        xmin = self.abs*self.size
        xmax = xmin + self.size
        ymin = self.ord * self.size
        ymax = ymin + self.size
        xc = (xmax-xmin)/2 + xmin
        yc = (ymax-ymin)/2 + ymin
        return (xmin,xmax,ymin,ymax,xc,yc)
        
    def draw(self):
        (xmin,xmax,ymin,ymax,xc,yc)=self.getCellCoords()


        fillhere = Cell.FILLCOLOURS[CellAttributes.EMPTY]
        outlinehere = Cell.BORDERCOLOURS[CellAttributes.EMPTY]
        stipplehere = ''

        if CellAttributes.BLOCKED in self.cellAttributes:
            fillhere = Cell.FILLCOLOURS[CellAttributes.BLOCKED]
            outlinehere=Cell.BORDERCOLOURS[CellAttributes.BLOCKED]
        else:
            if CellAttributes.FAILSTATE in self.cellAttributes:
                fillhere = Cell.FILLCOLOURS[CellAttributes.FAILSTATE]
                outlinehere=Cell.BORDERCOLOURS[CellAttributes.FAILSTATE]
            
            
        
        self.rectangle=self.canvas.create_rectangle(xmin,ymin,xmax,ymax,fill=fillhere,outline=outlinehere,stipple=stipplehere)
        if CellAttributes.AVOID in self.cellAttributes:
            self.drawAvoid(xmin,ymin,xmax,ymax,fillhere,outlinehere)
        #if CellAttributes.GOAL in self.cellAttributes:
        #    self.drawGoal(xc,yc)
        #if CellAttributes.INITIALLOC in self.cellAttributes:
        #    self.drawAgent(xmin,ymin,0)
        
        self.labelObj=self.canvas.create_text((xc,yc),text=self.label)
        
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

            

class CanvasWorld(object):
    MAXSIZE=(18.0*18.0*50.0*50.0)
    ROBOT_COLORS_LIST=[mc.XKCD_COLORS['xkcd:aqua'],mc.XKCD_COLORS['xkcd:beige'],mc.XKCD_COLORS['xkcd:coral'],mc.XKCD_COLORS['xkcd:fuchsia'],mc.XKCD_COLORS['xkcd:indigo'],mc.XKCD_COLORS['xkcd:goldenrod'],mc.XKCD_COLORS['xkcd:khaki'],mc.XKCD_COLORS['xkcd:lavender'],mc.XKCD_COLORS['xkcd:lightblue'],mc.XKCD_COLORS['xkcd:tomato'],mc.XKCD_COLORS['xkcd:pink'],mc.XKCD_COLORS['xkcd:teal']]
    
    def calculateDimensions(self,size=50,ncols=10,nrows=10):
        maxCellsSide = max(ncols,nrows)
        numCells = maxCellsSide*maxCellsSide
        #print ("Num Cells: "+str(numCells))
        newsize = CanvasWorld.MAXSIZE/float(numCells)
        #print ("New cell area "+str(newsize))
        newsize = math.sqrt(newsize)
        #print ("One side "+str(newsize))
        newsize = int(newsize)
        #print ("Max Area = "+str(CanvasWorld.MAXSIZE))
        #print ("Original Size = "+str(size)+" New Size = "+str(newsize))
        return newsize

    def __init__(self,name):
        self.canvas = None
        self.cellsize = None
        self.rows = None
        self.cols = None
        self.name = name 

    def initialiseOrResetCanvas(self,app,size=50,ncols=10,nrows=10):
        size = self.calculateDimensions(size,ncols,nrows)
        w = ncols*size
        h = nrows*size
        self.rows = nrows
        self.cols = ncols
        
        if self.canvas is not None:
            self.canvas.delete('all')
        else:
            self.canvas = Canvas(app,width=w, height=h)
        self.cellsize = size 

    def fillCanvasWithCells(self,xydict=None):
        self.agents = {}
        self.grid = []
        self.goals = {}
        
        agsize = self.cellsize/3
        labelsmap = {}
        for row in range(self.rows):
            line = []
            for col in range(self.cols):
                cell = Cell(self.canvas,col,row,self.cellsize)
                if xydict is not None:
                    if (row,col) in xydict:
                        flagsHere = xydict[(row,col)]
                        if flagsHere['fill']:
                            cell.cellAttributes.append(CellAttributes.BLOCKED)


                        if flagsHere['isFailState']:
                            cell.cellAttributes.append(CellAttributes.FAILSTATE)
                        if flagsHere['isAvoidState']:
                            cell.cellAttributes.append(CellAttributes.AVOID)
                        if flagsHere['isDoor']:
                            cell.cellAttributes.append(CellAttributes.DOOR)
                            cell.otherDoor = flagsHere['otherDoor']
                            
                        cell.label = flagsHere['label']
                        if not "," in cell.label:
                            labelint = int(cell.label)
                            if labelint not in labelsmap:
                                labelsmap[labelint]=cell

                        if flagsHere['isGoalPos']:
                            cell.cellAttributes.append(CellAttributes.GOAL)
                            goal = Goal(self.canvas,cell,labelint)
                            self.goals[labelint] = goal
                            
                        if flagsHere['isInitPos']:
                            cell.cellAttributes.append(CellAttributes.INITIALLOC)
                            #theres an agent here
                            agnum = len(self.agents)
                            agcolor = CanvasWorld.ROBOT_COLORS_LIST[agnum]
                            agent = Agent(self.canvas,cell,agsize,labelint,agcolor,agnum)
                            agent.drawAgent = True 
                            self.agents[agnum]=agent
                            
                        
                                
                line.append(cell)
            self.grid.append(line)
        #self.drawCells()
        self.labelsmap=labelsmap

    def moveAgents(self):
        for ag in self.agents:
            agent = self.agents[ag]
            currloc = agent.currloc
            if (currloc+1) in self.labelsmap:
                agent.updateCell(currloc+1,self.labelsmap[currloc+1])
                if (currloc+1) in self.goals:
                    self.goals[currloc+1].setVisited()

        
    def drawAgents(self):
        for ag in self.agents:
            self.agents[ag].draw()


    def drawGoals(self):
        for gl in self.goals:
            self.goals[gl].draw()
            
    def drawCells(self):
        for row in self.grid:
            for cell in row:
                cell.draw()

        #last cell
        centerx = self.rows*self.cellsize/2

        edgey = self.cols*self.cellsize+self.cellsize/2

        self.canvas.create_text((centerx,edgey),text=self.name)

    def packCanvas(self,side):
        self.canvas.pack(side=side,expand=True,fill='both')
        self.drawCells()
        self.drawGoals()
        self.drawAgents()


    def shiftAgentPaths(self,shiftby):
        for ag in self.agents:
            self.agents[ag].shiftAllPaths(shiftby)

    
                
    


        
class MainApp(object):

    StapuCanvasSide = 'left'
    SsiCanvasSide = 'right'
    def loadGridFile(self):
        fn = tkFileDialog.askopenfilename(initialdir="/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/smallerwhdoors/",title="Open Grid File", filetypes=(("grid",".grid"),("all files","*.*")))
        print ("Loading "+ fn)
        gfr = GridFileReader()
        (row,column,size,xydict)=gfr.readGridFile(fn)
        self.initialiseStapuSsiCanvases(size,column,row)
        #so now we can fill a canvas with xy dict
        self.stapucanvas.fillCanvasWithCells(xydict)
        self.stapucanvas.packCanvas(MainApp.StapuCanvasSide)
        self.ssicanvas.fillCanvasWithCells(xydict)
        self.ssicanvas.packCanvas(MainApp.SsiCanvasSide)


    def initialiseStapuSsiCanvases(self,updatedSize=50,column=10,row=10):
        self.stapucanvas.initialiseOrResetCanvas(self.app,updatedSize,column,row)
        self.ssicanvas.initialiseOrResetCanvas(self.app,updatedSize,column,row)
        self.stapucanvas.fillCanvasWithCells()
        self.ssicanvas.fillCanvasWithCells()
        self.stapucanvas.packCanvas(MainApp.StapuCanvasSide)
        self.ssicanvas.packCanvas(MainApp.SsiCanvasSide)

    
    def exitApp(self):
        self.app.destroy()
        
    def generateMenu(self):
        menu = Menu(self.app)
        menu.add_command(label='Grid File',command=self.loadGridFile)
        menu.add_command(label='Move Agents',command=self.moveAgentsOnBothCanvases)
        menu.add_command(label='Start Animation',command=self.animateAgentsOnBothCanvases)
        menu.add_command(label='Stop Animation',command=self.stopAnimationOnBothCanvases)
        menu.add_command(label='Shift Path', command=self.shiftPaths)
        menu.add_command(label='Exit',command=self.exitApp)
        self.app.config(menu=menu)

        

    def shiftPaths(self):
        self.stapucanvas.shiftAgentPaths(10)
    
        
    def moveAgentsOnBothCanvases(self):
        if self.animateBothCanvases:
            self.stapucanvas.moveAgents()
            self.ssicanvas.moveAgents()
            self.app.after(100,self.moveAgentsOnBothCanvases)
        else:
            self.stapucanvas.moveAgents()
            self.ssicanvas.moveAgents()


    def animateAgentsOnBothCanvases(self):
        self.animateBothCanvases = True


    def stopAnimationOnBothCanvases(self):
        self.animateBothCanvases = False
            

        
    def __init__(self):
        self.animateBothCanvases = False
        self.app = Tk()
        self.stapucanvas = CanvasWorld('STAPU')
        self.ssicanvas = CanvasWorld('Auctioning')
        #lets load the main window
        self.initialiseStapuSsiCanvases()
        self.generateMenu()
        self.app.title('STAPU/Auctioning Visualiser')

        
        self.app.mainloop()

    
        
        


if __name__=="__main__":
    ma = MainApp()
    
