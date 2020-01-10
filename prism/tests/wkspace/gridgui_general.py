from tkinter import *
import Tkinter, Tkconstants, tkFileDialog, tkSimpleDialog

from GeneratePrismFiles import GeneratePrismFile

from enum import Enum

import random
import math
from operator import truediv 

import matplotlib.colors as mc
import colorsys

import os

def ensure_dir(file_path):
    directory = os.path.dirname(file_path)
    if not os.path.exists(directory):
        os.makedirs(directory)


#from https://stackoverflow.com/questions/37765197/darken-or-lighten-a-color-in-matplotlib
def lighten_color(color, amount=0.5):
    """
    Lightens the given color by multiplying (1-luminosity) by the given amount.
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

#constants
class GuiState(Enum):
    OBSTACLES=1
    INITLOCS=2
    GOALS=3
    FAILSTATES=4
    AVOIDSTATES=5
    DOORS=6
    EMPTY=7

class Cell():
    BORDER_COLORS = {GuiState.OBSTACLES:'#000000',GuiState.EMPTY:'#000000',GuiState.INITLOCS:'#0000FF',GuiState.GOALS:'#00FF00',GuiState.AVOIDSTATES:'#FF0000',GuiState.FAILSTATES:'#888888',GuiState.DOORS:'#923A00'}
    FILL_COLORS= {GuiState.OBSTACLES:'#000000',GuiState.EMPTY:'#FFFFFF',GuiState.INITLOCS:'#0000FF',GuiState.GOALS:'#00FF00',GuiState.AVOIDSTATES:'#FF0000',GuiState.FAILSTATES:'#888888',GuiState.DOORS:'#923AFF'}

    def __init__(self,master,x,y,size):
        """ Constructor of the object called by Cell(...) """
        self.master = master
        self.abs = x
        self.ord = y
        self.size= size
        self.fill= False
        self.isInitPos = False
        self.isGoalPos = False
        self.isFailState = False
        self.isAvoidState = False
        self.isDoor = False
        self.otherDoor = None
        self.label = str(y)+","+str(x)
        self.defaultLabel = str(y)+","+str(x)
        

    def clear_all_flags(self):
        self.fill= False
        self.isInitPos = False
        self.isGoalPos = False
        self.isFailState = False
        self.isAvoidState = False
        self.isDoor = False
        self.otherDoor = None

        
    def __str__(self):
        strrep = '[(%d,%d,%d)*{"fill":%s,"isInitPos":%s,"isGoalPos":%s,"isFailState":%s,"isAvoidState":%s,"isDoor":%s,"label":"%s"' % (self.ord,self.abs,self.size,str(self.fill),str(self.isInitPos),str(self.isGoalPos),str(self.isFailState),str(self.isAvoidState),str(self.isDoor),str(self.label))
        if self.isDoor:
            strrep = strrep + ',"otherDoor":'+str(self.otherDoor)
        else:
            strrep = strrep + ',"otherDoor":None'
        strrep = strrep + "}]"
        return strrep
            

    def _switch(self,appState):
        """ Switch if the cell is filled or not. """
        if appState is GuiState.OBSTACLES:
            self.fill= not self.fill
        elif appState is GuiState.INITLOCS:
            self.isInitPos = not self.isInitPos
        elif appState is GuiState.AVOIDSTATES:
            self.isAvoidState = not self.isAvoidState
        elif appState is GuiState.FAILSTATES:
            self.isFailState = not self.isFailState
        elif appState is GuiState.GOALS:
            self.isGoalPos = not self.isGoalPos
        elif appState is GuiState.DOORS:
            self.isDoor = not self.isDoor 
        else:
            print ("invalid app state")

    def draw(self):
        """ order to the cell to draw its representation on the canvas """
        isNothing = True
        if self.master != None :
            
            fill = Cell.FILL_COLORS[GuiState.EMPTY]
            outline = Cell.BORDER_COLORS[GuiState.EMPTY]
            
            if self.fill:
                isNothing = False 
                fill = Cell.FILL_COLORS[GuiState.OBSTACLES]
                outline = Cell.BORDER_COLORS[GuiState.OBSTACLES]
                    
            if self.isInitPos:
                isNothing = False
                fill = Cell.FILL_COLORS[GuiState.INITLOCS]
                outline = Cell.BORDER_COLORS[GuiState.INITLOCS]
                
            if self.isAvoidState:
                isNothing = False 
                fill = Cell.FILL_COLORS[GuiState.AVOIDSTATES]
                outline = Cell.BORDER_COLORS[GuiState.AVOIDSTATES]
                
            if self.isGoalPos:
                isNothing = False 
                fill = Cell.FILL_COLORS[GuiState.GOALS]
                outline = Cell.BORDER_COLORS[GuiState.GOALS]
                
            if self.isDoor:
                isNothing = False
                fill = Cell.FILL_COLORS[GuiState.DOORS]
                outline = Cell.BORDER_COLORS[GuiState.DOORS]

            if self.isFailState:
                if not isNothing:
                    fill = lighten_color(fill,0.5)
                else:
                    fill = Cell.FILL_COLORS[GuiState.FAILSTATES]
                    outline = Cell.BORDER_COLORS[GuiState.FAILSTATES]
                    
                
                    

            xmin = self.abs * self.size
            xmax = xmin + self.size
            ymin = self.ord * self.size
            ymax = ymin + self.size
            xc = (xmax-xmin)/2 + xmin
            yc = (ymax-ymin)/2 + ymin

            self.master.create_rectangle(xmin, ymin, xmax, ymax, fill = fill, outline = outline)
            #mylabel = canvas.create_text((400, 190), text="Label text")
            self.master.create_text((xc,yc),text=self.label)
            #if self.label is not self.defaultLabel:
            #    print self.label 
            

class CellGrid(Canvas):
    def __init__(self,fromFile,fn,master, rowNumber, columnNumber, cellSize, *args, **kwargs):
        self.appState = GuiState.OBSTACLES
        
        xydict = None
        
        if fromFile:
            with open(fn) as gfile:
                lines = gfile.readlines()
            #first line is the row column and size
            #print lines[0]
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
                
        
        Canvas.__init__(self, master, width = cellSize * columnNumber , height = cellSize * rowNumber, *args, **kwargs)

        self.cellSize = cellSize

        self.grid = []
        for row in range(rowNumber):

            line = []
            for column in range(columnNumber):
                cell = Cell(self, column, row, cellSize)
                
                if fromFile:
                    flagsHere = xydict[(row,column)]
                    cell.fill = flagsHere["fill"]
                    cell.isInitPos = flagsHere["isInitPos"]
                    cell.isGoalPos = flagsHere["isGoalPos"]
                    cell.isFailState = flagsHere["isFailState"]
                    cell.isAvoidState = flagsHere["isAvoidState"]
                    cell.isDoor = flagsHere["isDoor"]
                    cell.otherDoor = flagsHere["otherDoor"]
                    cell.label = flagsHere["label"]
                    #print cell
                    
                line.append(cell)
            self.grid.append(line)

        #memorize the cells that have been modified to avoid many switching of state during mouse motion.
        self.switched = []
        
        #bind click action
        self.bind("<Button-1>", self.handleMouseClick)  
        #bind moving while clicking
        self.bind("<B1-Motion>", self.handleMouseMotion)
        #bind release button action - clear the memory of midified cells.
        self.bind("<ButtonRelease-1>", lambda event: self.clearSwitched())

        self.draw()

    


    def clearSwitched(self):
        self.switched*=0
        
    def draw(self):
        for row in self.grid:
            for cell in row:
                cell.draw()

    def _eventCoords(self, event):
        row = int(event.y / self.cellSize)
        column = int(event.x / self.cellSize)
        return row, column

    def handleMouseClick(self, event):
        row, column = self._eventCoords(event)
        cell = self.grid[row][column]
        cell._switch(self.appState)
        cell.draw()
        #add the cell to the list of cell switched during the click
        self.switched.append(cell)

    def handleMouseMotion(self, event):
        row, column = self._eventCoords(event)
        cell = self.grid[row][column]

        if cell not in self.switched:
            cell._switch(self.appState)
            cell.draw()
            self.switched.append(cell)

class GridGuiRandomiseDialog(tkSimpleDialog.Dialog):

    def body(self,master):
        self.numRandomFiles = 1
        self.numRobots = 2
        self.numGoals = 3
        self.numFS = 1
        self.numAS = 1
        self.gridSize = 5
        
        defaultNumRandomFiles = StringVar(master,value=str(self.numRandomFiles))
        defaultNumRobots = StringVar(master,value=str(self.numRobots))
        defaultNumGoals = StringVar(master,value=str(self.numGoals))
        defaultNumFS = StringVar(master,value=str(self.numFS))
        defaultNumAS = StringVar(master,value=str(self.numAS))
        defaultGridSize = StringVar(master,value=str(self.gridSize))
        
        Label(master,text="Number of Random Files:").grid(row=0,sticky=W)
        self.en = Entry(master,textvariable=defaultNumRandomFiles)
        self.en.grid(row=0,column=1)

        Label(master,text="Number of robots:").grid(row=1,sticky=W)
        self.rn = Entry(master,textvariable=defaultNumRobots)
        self.rn.grid(row=1,column=1)

        Label(master,text="Number of goals:").grid(row=2,sticky=W)
        self.gn = Entry(master,textvariable=defaultNumGoals)
        self.gn.grid(row=2,column=1)
        
        Label(master,text="Number of fail states:").grid(row=3,sticky=W)
        self.fsn = Entry(master,textvariable=defaultNumFS)
        self.fsn.grid(row=3,column=1)

        Label(master,text="Number of avoid states:").grid(row=4,sticky=W)
        self.an = Entry(master,textvariable=defaultNumAS)
        self.an.grid(row=4,column=1)

        Label(master,text="Max Grid Size in x or y:").grid(row=5,sticky=W)
        self.grn = Entry(master,textvariable=defaultGridSize)
        self.grn.grid(row=5,column=1)

        
    def apply(self):
        numRandomFiles = int(self.en.get())
        numRobots = int(self.rn.get())
        numGoals = int(self.gn.get())
        numFS = int(self.fsn.get())
        numAS = int(self.an.get())
        gridSize = int(self.grn.get())
        
        #print numRandomFiles
        self.result = (numRandomFiles,numRobots,numGoals,numFS,numAS,gridSize)
        self.numRandomFiles = numRandomFiles
        self.numRobots = numRobots
        self.numGoals = numGoals
        self.numFS = numFS
        self.numAS = numAS
        self.gridSize = gridSize
        


        
    
class GridGui(object):
    
        
    def __init__(self):
        self.app = None
        self.gridArray = None
        self.blockedCells = None
        self.fn = ""
        self.xside = 5
        self.yside = 5
        self.grid = None
        self.loadGridFromFile = False
        self.size = 50
        self.gridArray = None
        self.numFilesToGenerate = 1
        self.numRobots = 2
        self.numGoals = 3
        self.numFS = 0
        self.numAS = 0

    def setShelves(self):
        self.grid.appState = GuiState.OBSTACLES

    def setDepotArea(self):
        self.grid.appState = GuiState.INITLOCS

    def possibleGoals(self):
        self.grid.appState = GuiState.GOALS

    def possibleFailStates(self):
        self.grid.appState = GuiState.FAILSTATES

    def addAvoidStates(self):
        self.grid.appState = GuiState.AVOIDSTATES

    def screenshot(self):
        self.screenshotfn(self.fn)
        
    def screenshotfn(self,fn):
        
        comm = "import -window root "+fn+".png"
        os.system(comm)
        #print type(self.grid)
        #print dir(self.grid)
        if type(fn) is tuple:
            fn = "x"
        self.grid.postscript(file=fn+"x.ps",colormode='color')

    def writeGridFileOnly(self,fn):

        gridArray = self.gridArray 

        gridWriteFile = open(fn+'.grid','w')
        lenx = len(gridArray)
        leny = len(gridArray[0])
        gsize = gridArray[0][0].size
        gridWriteFile.write(str(lenx)+','+str(leny)+','+str(gsize)+'\n')
        
        for i in range(len(gridArray)):
            for j in range(len(gridArray[i])):
                c = gridArray[i][j]
                gridWriteFile.write(str(c)+'\n')
                
        gridWriteFile.close()

    def getStateLists(self):
        gridArray = self.gridArray

        depotChoices = []
        failStateChoices = []
        goalChoices = []
        avoidChoices = []
        blockedStates = []
        openStates = []
        
        for i in range(len(gridArray)):
            for j in range(len(gridArray[i])):
                c = gridArray[i][j]
                d = (i,j)
                if c.fill:
                    blockedStates.append(d)
                else:
                    openStates.append(d)
                    if c.isAvoidState:
                        avoidChoices.append(d)
                    if c.isInitPos:
                        depotChoices.append(d)
                    if c.isGoalPos:
                        goalChoices.append(d)
                    if c.isFailState:
                        failStateChoices.append(d)
        #now lets clean up
        #we can actually clean up later
        #and enforce an order
        #yeah that would be nice
        #if we dont have any failStateChoices or goalChoices or avoidChoices we set them to
        #those from the depot Choices and openStates
        stateChoices = None
        if len(depotChoices) == 0:
            depotChoices = openStates
            if(len(avoidChoices) > 0):
                depotChoices = self.listOneMinuslistTwo(openStates,avoidChoices)
                
            print 'You did not choose initial state locations so setting it to everything!!!!!'
            
        
        if len(failStateChoices) == 0:
            failStateChoices = openStates
        if len(goalChoices) == 0:
            if stateChoices is None:
                if(len(avoidChoices) > 0):
                    stateChoices = self.listOneMinuslistTwo(openStates,avoidChoices) 
                #stateChoices = self.listOneMinuslistTwo(openStates,depotChoices)
                #list(set(openStates).symmetric_difference(set(depotChoices)))
            goalChoices = stateChoices
        if len(avoidChoices) == 0:
            if stateChoices is None:
                stateChoices =self.listOneMinuslistTwo(openStates,depotChoices)
                #stateChoices = self.listOneMinuslistTwo(stateChoices,goalChoices) #list(set(openStates).symmetric_difference(set(depotChoices)))
            avoidChoices = stateChoices
            
        return (depotChoices,failStateChoices,goalChoices,avoidChoices,blockedStates,openStates)



    def intersection(self,lst1, lst2): 
        lst3 = [value for value in lst1 if value in lst2] 
        return lst3

    #sub lst2 from lst1 so basically just get all the values in lst1 that are not in lst2 
    def listOneMinuslistTwo(self,lst1,lst2):
        intersectingList = self.intersection(lst1,lst2)
        lst4 = [value for value in lst1 if value not in intersectingList]
        return lst4
    
    def listsIntersect(self,lst1,lst2):
        lst3 = self.intersection(lst1,lst2)
        if len(lst3) != 0:
            print ("Intersecting values")
            print (lst3)
            return True
        else:
            return False



    def doAutoGenGrid(self):
        self.fn = tkFileDialog.asksaveasfilename(initialdir="/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/guiFiles",title="Save file as", filetypes=(("grid",".grid"),("prism",".prism"),("prop",".prop"),("props",".props"),("all files","*.*")))
        self.app.update()
        self.fn = self.fn.split('.')[0]

        doorPairs = []
        #get max grid size
        #then do increments
        #we only need to do this once
        #the min grid size = int(sqrt(max (numgoals+numinitlocs+avoidstates,failstates)))

        #import math 
        maxStatesNeeded = min(self.numRobots+self.numGoals+self.numAS,self.numFS)
        minGridSize = int(math.sqrt(maxStatesNeeded))+1
        print "Min Grid Size"
        print minGridSize
        
        numGridSizeVars = 10
        gridIncs =int(truediv((self.maxGridSize - minGridSize),numGridSizeVars))+1
        print "Grid incs"
        print gridIncs
        #raw_input("continue")
        

        for gridSx in range(minGridSize,self.maxGridSize,gridIncs):
            gridSy = gridSx

            size = self.getGridCellSize(gridSx,gridSy)
            self.size = size

            self.grid = CellGrid(False,self.fn,self.app,gridSx,gridSy,size)
            self.gridArray = self.grid.grid
            #self.grid.pack()
            #self.app.mainloop()

            #we dont care about anything really
            gridArray = self.gridArray

            depotChoices = []
            failStatesChoices = []
            goalChoices = []
            avoidChoices = []
            blockedStates = []
            openStates = []

            for i in range(len(gridArray)):
                for j in range(len(gridArray[i])):
                    c = gridArray[i][j]
                    d = (i,j)
                    if c.fill:
                        blockedStates.append(d)
                    else:
                        openStates.append(d)

            #resetting the numfs
            self.numFS = gridSx*gridSy
 
            #do a percentage of the state space
            fsNums = []
            pcInc = 10 
            for i in range(0,100,pcInc):
                if i == 0:
                    fsNums.append(0)
                else:
                    
                    numFSHere = int(truediv(i,100)*self.numFS)+1
                    fsNums.append(numFSHere)

            fsNums.append(self.numFS)
            print "fail states"
            print fsNums
            #raw_input("num fs for grid size "+str(gridSx))
            
            for i in range(self.numFilesToGenerate):
                if self.numFS>0:
                    for j in range(len(fsNums)):
                        (chosenInitialLocations,chosenGoalLocations,chosenAvoidStates,chosenFailStates)= self.chooseAllStates(depotChoices,openStates,goalChoices,avoidChoices,failStatesChoices,fsNums[j])


                        gfr = GeneratePrismFile()
                        doFour = True #four grid actions

                        fn = self.fn +"grid_"+str(gridSx)+"_fsp_"+str(j*pcInc)+ "_"+str(i)+"_"
                        print fsNums[j]
                        print j*pcInc

                        smap=gfr.generateFromGUIGrid(chosenInitialLocations,blockedStates,chosenGoalLocations,chosenAvoidStates,chosenFailStates,openStates,doorPairs,gridSx,gridSy,fn,doFour)
                        self.updateGridArray(chosenInitialLocations,blockedStates,chosenGoalLocations,chosenAvoidStates,chosenFailStates,openStates,smap)
                        self.screenshotfn(fn)
                        self.writeGridFileOnly(fn)

                else:
                    (chosenInitialLocations,chosenGoalLocations,chosenAvoidStates,chosenFailStates)= self.chooseAllStates(depotChoices,openStates,goalChoices,avoidChoices,failStatesChoices,self.numFS)

                    gfr = GeneratePrismFile()
                    doFour = True #four grid actions

                    fn = self.fn +"_fs"+str(self.numFS)+ "_"+str(i)+"_"

                    smap=gfr.generateFromGUIGrid(chosenInitialLocations,blockedStates,chosenGoalLocations,chosenAvoidStates,chosenFailStates,openStates,doorPairs,gridSx,gridSy,fn,doFour)
                    self.updateGridArray(chosenInitialLocations,blockedStates,chosenGoalLocations,chosenAvoidStates,chosenFailStates,openStates,smap)
                    self.screenshotfn(fn)
                    self.writeGridFileOnly(fn)                    


        print "Done"

        
                
        
    def allDone(self):
        
        self.fn = tkFileDialog.asksaveasfilename(initialdir="/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/guiFiles",title="Save file as", filetypes=(("grid",".grid"),("prism",".prism"),("prop",".prop"),("props",".props"),("all files","*.*")))
        self.app.update()
        self.fn = self.fn.split('.')[0]
        
        #get all the goal states and stuff


        if self.gridArray is not None:
            (depotChoices,failStatesChoices,goalChoices,avoidChoices,blockedStates,openStates) = self.getStateLists()
            
            print "Depot Choices"
            print depotChoices
            print "Fail State Choices"
            print failStatesChoices
            print "Goal Choices"
            print goalChoices
            print "Avoid Choices"
            print avoidChoices
            print "Blocked States"
            print blockedStates
            print "Open States"
            print openStates

            print "Num Robots"
            print self.numRobots
            print "Num Goals"
            print self.numGoals
            print "Num Fail States"
            print self.numFS
            print "Num Avoid States"
            print self.numAS

            doorPairs = []
            
            #now lets choose some
            #we choose in an order really
            #first we choose the initial locations
            #then we choose the goals
            #then we choose the avoid states
            #then we choose the failstate
            doPercentageFS = True
            maxStates = len(openStates)
            
            if doPercentageFS:
                #resetting the numfs
                self.numFS = maxStates

                #do a percentage of the state space
                fsNums = []
                pcInc = 10 
                for i in range(0,100,pcInc):
                    if i == 0:
                        fsNums.append(0)
                    else:

                        numFSHere = int(truediv(i,100)*self.numFS)+1
                        fsNums.append(numFSHere)

                fsNums.append(self.numFS)
                print "fail states"
                print fsNums                
            else:
                if self.numFS > len(failStatesChoices):
                    self.numFS = len(failStatesChoices)
                numFSInc = 1
                numFSSteps = 10 
                if self.numFS > 10:
                    numFSInc = (self.numFS)/numFSSteps
                print "Max fs"
                print self.numFS
                print "fs inc"
                print numFSInc
                fsNums = range(0,self.numFS,numFSInc)
                print fsNums
                #raw_input("continue")
            
            for i in range(self.numFilesToGenerate):
                if self.numFS>0:
                    for j in range(len(fsNums)):
                        (chosenInitialLocations,chosenGoalLocations,chosenAvoidStates,chosenFailStates)= self.chooseAllStates(depotChoices,openStates,goalChoices,avoidChoices,failStatesChoices,fsNums[j])

                            
                        gfr = GeneratePrismFile()
                        doFour = True #four grid actions
                        if doPercentageFS:
                            fn = self.fn +"_fs"+str(fsNums[j])+"_fsp_"+str(j*10)+ "_"+str(i)+"_"
                        else:
                            fn = self.fn +"_fs"+str(fsNums[j])+ "_"+str(i)+"_"

                        smap=gfr.generateFromGUIGrid(chosenInitialLocations,blockedStates,chosenGoalLocations,chosenAvoidStates,chosenFailStates,openStates,doorPairs,self.xside,self.yside,fn,doFour)
                        self.updateGridArray(chosenInitialLocations,blockedStates,chosenGoalLocations,chosenAvoidStates,chosenFailStates,openStates,smap)
                        self.screenshotfn(fn)
                        self.writeGridFileOnly(fn)

                else:
                    (chosenInitialLocations,chosenGoalLocations,chosenAvoidStates,chosenFailStates)= self.chooseAllStates(depotChoices,openStates,goalChoices,avoidChoices,failStatesChoices,self.numFS)

                    gfr = GeneratePrismFile()
                    doFour = True #four grid actions

                    fn = self.fn +"_fs"+str(self.numFS)+ "_"+str(i)+"_"

                    smap=gfr.generateFromGUIGrid(chosenInitialLocations,blockedStates,chosenGoalLocations,chosenAvoidStates,chosenFailStates,openStates,doorPairs,self.xside,self.yside,fn,doFour)
                    self.updateGridArray(chosenInitialLocations,blockedStates,chosenGoalLocations,chosenAvoidStates,chosenFailStates,openStates,smap)
                    self.screenshotfn(fn)
                    self.writeGridFileOnly(fn)                    

            
        print "Done"


    
    def chooseAllStates(self,depotChoices,openStates,goalChoices,avoidChoices,failStatesChoices,numFS):


        if (len(depotChoices))==0:
            depotChoices = openStates
        chosenInitialLocations = random.sample(depotChoices,self.numRobots)
        #print("initial locations")
        #raw_input(chosenInitialLocations)

        if goalChoices is None or (len(goalChoices))== 0:
            goalChoices =self.listOneMinuslistTwo(openStates,chosenInitialLocations) #list(set(openStates).symmetric_difference(set(chosenInitialLocations)))
        else:
            if (self.listsIntersect(chosenInitialLocations,goalChoices)):
                print "Initial states and goal states intersect!!"
                print "Changing this!!"
                #raw_input()
                goalChoices = self.listOneMinuslistTwo(goalChoices,chosenInitialLocations)#list(set(goalChoices).symmetric_difference(set(chosenInitialLocations)))
                
                
        if (len(goalChoices)) < self.numGoals:
            goalChoices =self.listOneMinuslistTwo(openStates,chosenInitialLocations)# list(set(openStates).symmetric_difference(set(chosenInitialLocations)))
        #raw_input(goalChoices)
        chosenGoalLocations = random.sample(goalChoices,self.numGoals)
        #print("goal locations")
        #raw_input(chosenGoalLocations)
        #print("initial locations")
        #raw_input(chosenInitialLocations)
        
        if(len(avoidChoices)) == 0:
            avoidChoices = self.listOneMinuslistTwo(openStates,chosenInitialLocations)#list(set(openStates).symmetric_difference(set(chosenInitialLocations)))
        statesToPickFrom =self.listOneMinuslistTwo(avoidChoices,chosenGoalLocations)# list(set(avoidChoices).symmetric_difference(set(chosenGoalLocations)))
        
        chosenAvoidStates = random.sample(statesToPickFrom,self.numAS)
        statesToPickFrom = failStatesChoices
        if (len(failStatesChoices)) == 0:
            failStatesChoices = openStates
        if numFS > len(failStatesChoices):
            numFS = len(failStatesChoices)
        chosenFailStates = random.sample(failStatesChoices,numFS)

        print "Chosen Things"
        print "Initial Locations"
        print chosenInitialLocations
        print "Goals"
        print chosenGoalLocations
        print "Avoid States"
        print chosenAvoidStates
        print "Fail States"
        print chosenFailStates

        #just need to check if the initial and goal locations dont overlap
        if self.listsIntersect(chosenInitialLocations,chosenGoalLocations):
            print "Initial states and goal states intersect!!"
            raw_input("press any key to continue but your code is wrong")

        if self.listsIntersect(chosenGoalLocations,chosenAvoidStates):
            print "Avoid states and goal states intersect!!"
            raw_input("press any key to continue but your code is wrong")

        if self.listsIntersect(chosenInitialLocations,chosenAvoidStates):
            print "Avoid states and initial states intersect!!"
            raw_input("press any key to continue but your code is wrong")

        return (chosenInitialLocations,chosenGoalLocations,chosenAvoidStates,chosenFailStates)

    def updateGridArray(self,chosenInitialLocations,blockedStates,chosenGoalLocations,chosenAvoidStates,chosenFailStates,openStates,smap):
        for i in range(len(self.gridArray)):
            for j in range(len(self.gridArray[i])):
                d = (i,j)
                self.gridArray[i][j].clear_all_flags()
                
                if d in chosenInitialLocations:
                    self.gridArray[i][j].isInitPos = True 
                if d in chosenGoalLocations:
                    self.gridArray[i][j].isGoalPos = True
                if d in chosenAvoidStates:
                    self.gridArray[i][j].isAvoidState = True
                if d in chosenFailStates:
                    self.gridArray[i][j].isFailState = True
                if d in blockedStates:
                    self.gridArray[i][j].fill = True 
                if d in smap:
                    lab = smap[d]
                    
                    self.gridArray[i][j].label = lab
                self.gridArray[i][j].draw()
                
                    

    def exitHere(self):
        self.app.destroy()

    def generateMultiples(self):
        d = GridGuiRandomiseDialog(self.app)
        self.numFilesToGenerate  = d.numRandomFiles
        self.numRobots = d.numRobots
        self.numGoals = d.numGoals
        self.numFS = d.numFS
        self.numAS = d.numAS
        self.maxGridSize = d.gridSize
        

    def createGuiMenu(self):
        self.menu = Menu(self.app)
        self.menu.add_command(label='Shelves',command=self.setShelves)
        self.menu.add_command(label='Depot',command = self.setDepotArea)
        self.menu.add_command(label='Possible Pickups',command=self.possibleGoals)
        self.menu.add_command(label='Failstates',command=self.possibleFailStates)
        self.menu.add_command(label='Areas To Avoid', command=self.addAvoidStates)
        #self.menu.add_command(label='Add Doors', command=self.addDoorStates)
        #self.menu.add_command(label='Add Door Pair' , command= self.processDoors)
        #self.menu.add_command(label='Center Goals', command=self.centerGoalsFunc)
        #self.menu.add_command(label='Side Goals',command=self.sideClusterGoals)
        self.menu.add_command(label='Generate Multiple Files', command=self.generateMultiples)
        self.menu.add_command(label='Screenshot', command=self.screenshot)

        self.menu.add_command(label='done',command=self.allDone)
        self.menu.add_command(label='autoGenGrid',command=self.doAutoGenGrid)
        
        self.menu.add_command(label='load file',command=self.openFile)

        self.menu.add_command(label='exit',command=self.exitHere)

        self.app.config(menu=self.menu)
    
    def createGui(self):
        self.app = Tk()
        self.openFile(True)

    def openFile(self,newFile=False):
        self.fn = tkFileDialog.askopenfilename(initialdir="/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/guiFiles/",title="Save file as", filetypes=(("grid",".grid"),("all files","*.*")))
        if self.fn:
            self.loadGridFromFile = True
            self.app.destroy()
            self.app = Tk()
            self.app.title(self.fn)
            
        if newFile:
            if not self.loadGridFromFile:
                self.xside = tkSimpleDialog.askinteger('Num cells x','Num cells x',initialvalue=5,minvalue=1,maxvalue=100)
                if self.xside is None:
                    self.xside = 5

                self.yside = tkSimpleDialog.askinteger('Num cells y','Num cells y',initialvalue=5,minvalue=1,maxvalue=100)

                if self.yside is None:
                    self.yside = 5
        if newFile or self.loadGridFromFile:
            self.createGuiMenu()
            self.size = self.getGridCellSize(self.xside,self.yside)
            
        if newFile or self.loadGridFromFile:
            self.grid = CellGrid(self.loadGridFromFile,self.fn,self.app,self.xside,self.yside,self.size)
            self.gridArray = self.grid.grid

            self.grid.pack()
            self.app.mainloop()

    def getGridCellSize(self,xside,yside):
        biggerside = max(xside,yside)
        area = biggerside*biggerside
        fullsize = area*self.size*self.size
        maxsize = 18*18*50*50
        newsize = maxsize/area
        #import math
        newsize = math.sqrt(newsize)
        size = int(newsize)
        return size
    

if __name__=="__main__":
    gridGui = GridGui()
    gridGui.createGui()
    
