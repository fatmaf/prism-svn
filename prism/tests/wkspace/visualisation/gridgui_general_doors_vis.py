from tkinter import *
import Tkinter, Tkconstants, tkFileDialog, tkSimpleDialog

from BaseCell import GuiState
from BaseCell import Cell
from BaseCell import Agent

from enum import Enum

import random
import math
from operator import truediv 

import matplotlib.colors as mc
import colorsys

import os

from readStaTra import ReadMDPStaTra

    
def ensure_dir(file_path):
    directory = os.path.dirname(file_path)
    if not os.path.exists(directory):
        os.makedirs(directory)




                    
    


class CellGrid(Canvas):
    ROBOT_COLORS_LIST=[mc.XKCD_COLORS['xkcd:aqua'],mc.XKCD_COLORS['xkcd:beige'],mc.XKCD_COLORS['xkcd:coral'],mc.XKCD_COLORS['xkcd:fuchsia'],mc.XKCD_COLORS['xkcd:indigo'],mc.XKCD_COLORS['xkcd:goldenrod'],mc.XKCD_COLORS['xkcd:khaki'],mc.XKCD_COLORS['xkcd:lavender'],mc.XKCD_COLORS['xkcd:lightblue'],mc.XKCD_COLORS['xkcd:tomato'],mc.XKCD_COLORS['xkcd:pink'],mc.XKCD_COLORS['xkcd:teal']]
    
    def __init__(self,fromFile,fn,master, rowNumber, columnNumber, cellSize, *args, **kwargs):
        self.appState = GuiState.OBSTACLES
        self.agentsList = []
        self.labelsmap = {}
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
                    if not "," in cell.label:
                        labelint = int(cell.label)
                        if labelint not in self.labelsmap:
                            self.labelsmap[labelint]=cell.getXY()
                    if cell.isInitPos:
                        agent = Agent()
                        agent.location = cell.getXY()
                        agent.defaultColor = CellGrid.ROBOT_COLORS_LIST[len(self.agentsList)]
                        cell.defaultColor = agent.defaultColor
                        self.agentsList.append(agent)
                        
                        
                        
                    
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

        self.columnNumber = columnNumber
        self.rowNumber =rowNumber
        self.agentlocx = 1
        self.agentlocy = 0

    

    def clearAgentsList(self):
        self.agentsList = []

    def createAgentsListFromStatesDict(self,agentsDict):
        self.clearAgentsList()
        for agnum in agentsDict:
            cellagentdict = {'color':'','dead':False}
            agent = Agent()
            agentloclab = agentsDict[agnum]
            if agentloclab == -1:
                agent.dead = True
                cellagentdict['dead']=True
            else:
                agent.dead = False 
                agent.location = self.labelsmap[agentsDict[agnum]]
            agent.defaultColor = CellGrid.ROBOT_COLORS_LIST[len(self.agentsList)]
            cellagentdict['color']=agent.defaultColor
            
            agent.name = agnum
            self.agentsList.append(agent)
    
    def updateAgentsListFromStatesDict(self,agentsDict):
        for agnum in agentsDict:
            
            agent = self.agentsList[agnum]
            if agent.name != agnum:
                print ("Error with agent name!!!")
            else:
                agentloclabel = agentsDict[agnum]
                if agentloclabel == -1:
                    agent.dead = True
                    
                else:
                    agent.dead = False 
                    agent.location = self.labelsmap[agentloclabel]
                    
            
            
        
    def getCellFromAgentLoc(self,location):
        return self.grid[location[self.agentlocx]][location[self.agentlocy]]
    
    def validatecell(self,col,row):
        if col < 0:
            col = 0
        if row < 0:
            row = 0
        if row >= self.columnNumber:
            row = self.columnNumber -1
            #print (self.columnNumber)
        if col >= self.rowNumber:
            col = self.rowNumber -1
            #print (self.rowNumber)
        return (col,row)
    
    def validatelocation(self,loc):
        newloc = self.validatecell(loc[self.agentlocx],loc[self.agentlocy])
        newloc = (newloc[self.agentlocx],newloc[self.agentlocy])
        return newloc
    
        
            
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

    def removeInitPos(self,cell):
        cell.isInitPos = False
        cell.defaultColor = None
        if cell.agents is not None:
            cell.agents = None
        self.updateCell(cell)

    def addInitPos(self,cell):
        doswitch = False
        if not cell.isInitPos:
            cell.isInitPos = True
            doswitch = True
        self.updateCell(cell,doswitch)
        
    def updateCell(self,cell,switch=False):
        cell.draw()
        if switch:
            self.switched.append(cell)

class GridGuiRandomiseDialog(tkSimpleDialog.Dialog):

    def body(self,master):
        self.numRandomFiles = 1
        self.numRobots = 2
        self.numGoals = 3
        self.numFS = 1
        self.numAS = 1
        self.gridSize = 5
        self.incrementDoors = Tkinter.IntVar()
        
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

        self.cbd = Checkbutton(master,text="Increment Doors",variable=self.incrementDoors)
        self.cbd.grid(row=6,columnspan=2)
        
        

        
    def apply(self):
        numRandomFiles = int(self.en.get())
        numRobots = int(self.rn.get())
        numGoals = int(self.gn.get())
        numFS = int(self.fsn.get())
        numAS = int(self.an.get())
        gridSize = int(self.grn.get())
        incrementDoors = bool(self.incrementDoors.get())
        
        print numRandomFiles
        self.result = (numRandomFiles,numRobots,numGoals,numFS,numAS,gridSize)
        self.numRandomFiles = numRandomFiles
        self.numRobots = numRobots
        self.numGoals = numGoals
        self.numFS = numFS
        self.numAS = numAS
        self.gridSize = gridSize
        self.increaseDoors = incrementDoors
        


        
    
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
        self.hasDoors = False
        self.incrementDoors = False
        self.doorPairs = None
        self.policyObj = None

    def addDoorStates(self):
        self.grid.appState = GuiState.DOORS
        
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

    def processDoors(self):
        print 'Process Doors not implemented'
        if self.grid.appState is not GuiState.DOORS:
            print 'Error - Adding doors before doors chosen'
        else:
            if self.doorPairs is None:
                self.doorPairs = []
            doorPairs = self.doorPairs    
            if self.gridArray is not None:
                currentDoorPair = []
                for i in range(len(self.gridArray)):
                    for j in range(len(self.gridArray[i])):
                        c = self.gridArray[i][j]
                        d = (i,j)
                        if c.isDoor:
                            if len(currentDoorPair) < 2:
                                if d not in doorPairs:
                                    currentDoorPair.append(d)
                            else:
                                doorPairs = doorPairs + currentDoorPair
                                currentDoorPair = []
                                if d not in doorPairs:
                                    currentDoorPair.append(d)
                        #print "Current door pair"
                        #print currentDoorPair
                        
                if len(currentDoorPair) == 2:
                    if currentDoorPair[0] not in doorPairs and currentDoorPair[1] not in doorPairs:
                        doorPairs = doorPairs + currentDoorPair
                elif len(currentDoorPair) < 2 and len(currentDoorPair) > 0:
                    print "Door pairs not equal"

                for i in range(0,len(doorPairs),2):
                    d1 = doorPairs[i]
                    d2 = doorPairs[i+1]
                    d1g = self.gridArray[d1[0]][d1[1]]
                    d2g = self.gridArray[d2[0]][d2[1]]
                    if d1g.otherDoor is None and d2g.otherDoor is None:
                        d1g.otherDoor = d2
                        d2g.otherDoor = d1
                        self.gridArray[d1[0]][d1[1]] = d1g
                        self.gridArray[d2[0]][d2[1]] = d2g
                    else:
                        if d1g.otherDoor is None or d2g.otherDoor is None:
                            print "Mismatched door pair"
                            print doorPairs
                            print d1
                            print d2
                print "Door Pairs"+str(doorPairs)
                self.doorPairs = doorPairs
                if self.doorPairs is not None:
                    self.doDoors = True 
        

            
        
    def screenshot(self):
        self.screenshotfn(self.fn)
        
    
        
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
        doorStates = []
        
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
                    if c.isDoor:
                        doorStates.append(d)
                        
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
            #if len(doorStates) > 0:
            #    failStateChoices = self.listOneMinuslistTwo(openStates,doorStates)
            
        if len(goalChoices) == 0:
            if stateChoices is None:
                if(len(avoidChoices) > 0):
                    stateChoices = self.listOneMinuslistTwo(openStates,avoidChoices)
            goalChoices = stateChoices
        if len(avoidChoices) == 0:
            if stateChoices is None:
                stateChoices =self.listOneMinuslistTwo(openStates,depotChoices)
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
                       


                        fn = self.fn +"grid_"+str(gridSx)+"_fsp_"+str(j*pcInc)+ "_"+str(i)+"_"
                        print fsNums[j]
                        print j*pcInc

                        self.writeFileSet(depotChoices,openStates,goalChoices,avoidChoices,failStatesChoices,fsNums[j],doorPairs,fn,gridSx,gridSy,blockedStates)
                        
                else:
                    
                    fn = self.fn +"_fs"+str(self.numFS)+ "_"+str(i)+"_"

                    self.writeFileSet(depotChoices,openStates,goalChoices,avoidChoices,failStatesChoices,self.numFS,doorPairs,fn,gridSx,gridSy,blockedStates)

        print "Done"

        

    def writeFileSet(self,depotChoices,openStates,goalChoices,avoidChoices,failStatesChoices,numFS,doorPairs,fn,xside,yside,blockedStates):
        (chosenInitialLocations,chosenGoalLocations,chosenAvoidStates,chosenFailStates)= self.chooseAllStates(depotChoices,openStates,goalChoices,avoidChoices,failStatesChoices,numFS)

        #gfr = GeneratePrismFile()
        #doFour = True #four grid actions
        #smap=gfr.generateFromGUIGrid(chosenInitialLocations,blockedStates,chosenGoalLocations,chosenAvoidStates,chosenFailStates,openStates,doorPairs,xside,yside,fn,doFour)
        self.updateGridArray(chosenInitialLocations,blockedStates,chosenGoalLocations,chosenAvoidStates,chosenFailStates,openStates,smap,doorPairs)
        self.screenshotfn(fn)
        self.writeGridFileOnly(fn)


    def loadstatra(self):
        fn = tkFileDialog.askopenfilename(initialdir="/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/smallerwhdoors/results/logs/debugRes/extras",title="Open sta/tra File", filetypes=(("sta",".sta"),("tra",".tra"),("all files","*.*")))
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

        if staname is not None:
            readMDPStaTra = ReadMDPStaTra(staname,traname)
            readMDPStaTra.readsta()
            readMDPStaTra.readtra()
            #readMDPStaTra.doMostProbablePath()
            self.policyObj = readMDPStaTra
            #now we want to initialise all the robots with this
            #so technically we're resetting all agent locations
            #self.grid.clearAgentsList()
            #now we create a new list from the new state stuff
            startState = 0
            agentsLocDict=readMDPStaTra.getAgentStatesFromState(0)
            self.clearCellsForAllRobots()
            self.grid.createAgentsListFromStatesDict(agentsLocDict)
            self.updateAgentsLocationColors()
            self.drawCellsForAgents()
            ignoredstates = [startState]
            parentstates = {startState:None}
            while(len(ignoredstates)!=0):
                startState = ignoredstates.pop(0)
                raw_input("Executing plan for state "+str(startState))
                if startState in parentstates:
                    if parentstates[startState] is not None:
                        #set stuff up
                        self.clearCellsForAllRobots()
                        agentsLocDict = readMDPStaTra.getAgentStatesFromState(parentstates[startState])
                        self.grid.updateAgentsListFromStatesDict(agentsLocDict)
                        self.updateAgentsLocationColors()
                        self.drawCellsForAgents()
                        #nsotherstates = readMDPStaTra.getMostProbableStateReactive(parentStates[startState])
                        
                nsotherstates = readMDPStaTra.getMostProbableStateReactive(startState)
                if nsotherstates is not None:
                    ns=nsotherstates[0]
                    q = [ns]
                    for ignorestate in nsotherstates[1]:
                        if ignorestate not in ignoredstates:
                            ignoredstates.append(ignorestate)
                            parentstates[ignorestate] = startState
                    #ignoredstates = ignoredstates+nsotherstates[1]
                    while(len(q)!=0):
                        raw_input("next move")
                        cs = q.pop(0)
                        agentsLocDict = readMDPStaTra.getAgentStatesFromState(cs)
                        self.clearCellsForAllRobots()
                        self.grid.updateAgentsListFromStatesDict(agentsLocDict)
                        self.updateAgentsLocationColors()
                        self.drawCellsForAgents()
                        nsotherstates = readMDPStaTra.getMostProbableStateReactive(cs)
                        if nsotherstates is not None:
                            ns = nsotherstates[0]
                            for ignoredstate in nsotherstates[1]:
                                if ignoredstate not in ignoredstates:
                                    parentstates[ignoredstate] = cs
                                    ignoredstates.append(ignoredstate)
                            #ignoredstates = ignoredstates+nsotherstates[1]
                            q.append(ns)

            

        
        
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

            print "Door Choices"
            doorPairs = self.doorPairs
            print doorPairs
            
            print "Increment Doors"
            print self.incrementDoors
            
            
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

                        if doPercentageFS:
                            fn = self.fn +"_fs"+str(fsNums[j])+"_fsp_"+str(j*10)+ "_"+str(i)+"_"
                        else:
                            fn = self.fn +"_fs"+str(fsNums[j])+ "_"+str(i)+"_"
                        if self.incrementDoors:
                            numDoors = len(doorPairs)/2
                            for dp in range(numDoors):
                                if (dp+1) == numDoors:
                                    doorNumbers = range(numDoors)
                                else:
                                    doorNumbers = random.sample(range(numDoors),dp+1)
                                chosenDoorPairs = []
                                for doorNumber in doorNumbers:
                                    chosenDoorPairs.append(doorPairs[doorNumber*2])
                                    chosenDoorPairs.append(doorPairs[(doorNumber*2)+1])
                                    

                                nfn = fn+"d_"+str(dp+1)+"_"
                                self.writeFileSet(depotChoices,openStates,goalChoices,avoidChoices,failStatesChoices,fsNums[j],chosenDoorPairs,nfn,self.xside,self.yside,blockedStates)
                        else:

                            self.writeFileSet(depotChoices,openStates,goalChoices,avoidChoices,failStatesChoices,fsNums[j],doorPairs,fn,self.xside,self.yside,blockedStates)
                        

                else:

                    fn = self.fn +"_fs"+str(self.numFS)+ "_"+str(i)+"_"
                    if self.incrementDoors:
                        numDoors = len(doorPairs)/2
                        for dp in range(numDoors):
                            if (dp+1) == numDoors:
                                doorNumbers = range(numDoors)
                            else:
                                doorNumbers = random.sample(range(numDoors),dp+1)
                            #now our chosenDoorPairs are
                            chosenDoorPairs = []
                            for doorNumber in doorNumbers:
                                chosenDoorPairs.append(doorPairs[doorNumber*2])
                                chosenDoorPairs.append(doorPairs[(doorNumber*2)+1])
                            nfn = fn+"d_"+str(dp+1)+"_"
                            self.writeFileSet(depotChoices,openStates,goalChoices,avoidChoices,failStatesChoices,self.numFS,chosenDoorPairs,nfn,self.xside,self.yside,blockedStates)

                            
                    else:
                        self.writeFileSet(depotChoices,openStates,goalChoices,avoidChoices,failStatesChoices,self.numFS,doorPairs,fn,self.xside,self.yside,blockedStates)
            
        print "Done"


    
    def chooseAllStates(self,depotChoices,openStates,goalChoices,avoidChoices,failStatesChoices,numFS):


        print openStates
        if (len(depotChoices))==0:
            depotChoices = openStates
        chosenInitialLocations = random.sample(depotChoices,self.numRobots)
        #print("initial locations")
        #raw_input(chosenInitialLocations)

        if goalChoices is None or (len(goalChoices))== 0:
            goalChoices =self.listOneMinuslistTwo(openStates,chosenInitialLocations) 
        else:
            if (self.listsIntersect(chosenInitialLocations,goalChoices)):
                print "Initial states and goal states intersect!!"
                print "Changing this!!"
                #raw_input()
                goalChoices = self.listOneMinuslistTwo(goalChoices,chosenInitialLocations)          
                
        if (len(goalChoices)) < self.numGoals:
            goalChoices =self.listOneMinuslistTwo(openStates,chosenInitialLocations)# list(set(openStates).symmetric_difference(set(chosenInitialLocations)))
        #raw_input(goalChoices)
        print goalChoices
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

    def updateGridArray(self,chosenInitialLocations,blockedStates,chosenGoalLocations,chosenAvoidStates,chosenFailStates,openStates,smap,doorPairs):
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
                if d in doorPairs:
                    self.gridArray[i][j].isDoor = True 
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
        self.incrementDoors = d.increaseDoors


    def clearCellsForAllRobots(self):
        for agent in self.grid.agentsList:
            currloc = agent.location
            currcell = self.grid.getCellFromAgentLoc(currloc)
            self.grid.removeInitPos(currcell)

    def updateAgentsLocation(self,actions):
        if type(actions) is list:
            for i in range(len(self.grid.agentsList)):
                if len(actions) < i:
                    action = actions[i]
                    agent = self.grid.agentsList[i]
                    agent.move(action)
                    agent.location = self.grid.validatelocation(agent.location)
        else:
            for agent in self.grid.agentsList:
                agent.move(actions)
                agent.location = self.grid.validatelocation(agent.location)
                    

    def updateAgentsLocationColors(self):
        for agent in self.grid.agentsList:
            currloc = agent.location
            currcell = self.grid.getCellFromAgentLoc(currloc)
            currcell.updateInitPosColor(agent.defaultColor)
            if currcell.agents is None:
                currcell.agents = []
            currcell.agents.append({'color':agent.defaultColor,'dead':agent.dead})

    def drawCellsForAgents(self):
        for agent in self.grid.agentsList:
            currloc = agent.location
            currcell = self.grid.getCellFromAgentLoc(currloc)
            self.grid.addInitPos(currcell)

    def printAgentsInfo(self):
        for agent in self.grid.agentsList:
            currloc = agent.location
            #print ("Agent location "+str(agent.location))
            cell = self.grid.getCellFromAgentLoc(currloc)
            print ("Agent location "+str(agent.location)+ " init pos: "+str(cell.isInitPos) + " color "+str(cell.defaultColor) + " agent color "+str(agent.defaultColor))
        
            
            
    def moveRobots(self,actlab):      
        self.clearCellsForAllRobots()
        print ("Cleared all cells")
        self.printAgentsInfo()
        
        self.updateAgentsLocation(actlab)
        print ("Updated Agent locations")
        self.printAgentsInfo()
        
        self.updateAgentsLocationColors()
        print ("Updated Agent Colors")
        self.printAgentsInfo()
        
        self.drawCellsForAgents()
        print ("Drew Agents")
        self.printAgentsInfo()
        

    def moveDown(self):
        actlab = Agent.AgentActions.DOWN
        self.moveRobots(actlab)

    def moveUp(self):
        actlab=Agent.AgentActions.UP
        self.moveRobots(actlab)

    def moveLeft(self):
        actlab = Agent.AgentActions.LEFT
        self.moveRobots(actlab)

    def moveRight(self):
        actlab = Agent.AgentActions.RIGHT
        self.moveRobots(actlab)
        

    def createGuiMenu(self):
        self.menu = Menu(self.app)
        self.menu.add_command(label='Shelves',command=self.setShelves)
        self.menu.add_command(label='Depot',command = self.setDepotArea)
        self.menu.add_command(label='Possible Pickups',command=self.possibleGoals)
        self.menu.add_command(label='Failstates',command=self.possibleFailStates)
        self.menu.add_command(label='Areas To Avoid', command=self.addAvoidStates)
        self.menu.add_command(label='Add Doors', command=self.addDoorStates)
        self.menu.add_command(label='Add Door Pair' , command= self.processDoors)
        self.menu.add_command(label='Set Generation Parameters', command=self.generateMultiples)

        self.menu.add_command(label='Load sta/tra file',command=self.loadstatra)

        self.menu.add_command(label='autoGenFiles',command=self.allDone)
        self.menu.add_command(label='autoGenGrid',command=self.doAutoGenGrid)
        
        self.menu.add_command(label='load file',command=self.openFile)

        self.menu.add_command(label='exit',command=self.exitHere)

        self.app.config(menu=self.menu)
    
    def createGui(self):
        self.app = Tk()
        self.openFile(True)

    def openFile(self,newFile=False):
        self.fn = tkFileDialog.askopenfilename(initialdir="/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/smallerwhdoors/",title="Open Grid File", filetypes=(("grid",".grid"),("all files","*.*")))
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
    
