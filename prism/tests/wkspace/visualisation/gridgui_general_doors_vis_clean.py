from tkinter import *
import Tkinter, Tkconstants, tkFileDialog, tkSimpleDialog

from BaseCell import GuiState
from BaseCell import Cell
from BaseCell import Agent
from BaseCell import Goal 

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

    

    def clearGoals(self,goalsList):
        for goallabel in goalsList:
            #goallabel = goalsList[i]
            if goallabel in self.labelsmap:
                goallocation = self.labelsmap[goallabel]
                goalcell = self.getCellFromAgentLoc(goallocation)
                self.removeGoalPos(goalcell)
                if goalcell.goal is not None:
                    goalcell.goal=None


    def setGoals(self,goalsList):
        for glabel in goalsList:
            if glabel in self.labelsmap:
                goallocation = self.labelsmap[glabel]
                goalcell =self.getCellFromAgentLoc(goallocation)
                
                if goalcell.goal is None:
                    goal = Goal()
                    goal.location = goallocation
                    goal.visited = False
                    goalcell.goal = goal
                else:
                    goalcell.goal.visited = False
                self.setGoalPos(goalcell)
                self.updateCell(goalcell)

    def setGoalsFromState(self,goalsList,goalDAVals,daStates):
        for glabel in goalsList:
            if glabel in self.labelsmap:
                goallocation = self.labelsmap[glabel]
                goalcell = self.getCellFromAgentLoc(goallocation)
                if goalcell.goal is not None:
                    goaldanum = goalDAVals[glabel]
                    if goaldanum in daStates:
                        goalcell.goal.visited = daStates[goaldanum]
                    else:
                        goalcell.goal.visited = False
                self.setGoalPos(goalcell)
                self.updateCell(goalcell)
        
                

    
    def clearAgentsList(self):
        self.agentsList = []

    def createAgentsListFromStatesDict(self,agentsDict):
        self.clearAgentsList()
        for agnum in agentsDict:
            cellagentdict = {'color':'','dead':False,'deadCounter':0}
            agent = Agent()
            agentloclab = agentsDict[agnum]
            if agentloclab == -1:
                agent.died()
                #agent.dead = True
                cellagentdict['dead']=True
                #cellagentdict['deadCounter']=agent.deadCounter
            else:
                agent.alive()
                #cellagentdict['deadCounter']=agent.deadCounter
                #agent.dead = False 
                agent.location = self.labelsmap[agentsDict[agnum]]
            agent.defaultColor = CellGrid.ROBOT_COLORS_LIST[len(self.agentsList)]
            cellagentdict['color']=agent.defaultColor
            cellagentdict['deadCounter']=agent.deadCounter
            
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
                    agent.died()                    
                else:
                    agent.alive()
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

    def removeGoalPos(self,cell):
        cell.isGoalPos = False
        self.updateCell(cell)

    def setGoalPos(self,cell):
        cell.isGoalPos = True
        self.updateCell(cell)
        
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

class GridGuiGoalsDialog(tkSimpleDialog.Dialog):

    def body(self,master):
        self.goalsList = '[0,1,2,3,4,5,6,7,8,9]'
        defaultGoalsList = StringVar(master,value=self.goalsList)
        Label(master,text="Goals List:").grid(row=0,sticky=W)
        self.gl=Entry(master,textvariable=defaultGoalsList)
        self.gl.grid(row=0,column=1)

    def apply(self):
        goalsList = self.gl.get()
        
        #validating
        import re
        pattern = re.compile(r"\[[\d\s?,]*\]")
        if not pattern.match(goalsList):
            print ("Invalid goals list pattern. Expected [1,2,3,4,5...]. Got "+goalsList+". Reverting to default "+self.goalsList)
            goalsList = self.goalsList
        goalsArray = eval(goalsList)
        self.goalsArray = goalsArray
        
    
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
        self.goals = None
        self.goalsDAVals = None
    
        
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


    def getLines(self,fn):
        lines = None
        with open(fn) as f:
            lines = f.readlines()
        return lines

    def propRegex(self,propBit):
        import re
        res=[]
        regex = ur"F \(\"s(\d*)\"\)"


        matches = re.finditer(regex, propBit)

        for matchNum, match in enumerate(matches, start=1):

            #print ("Match {matchNum} was found at {start}-{end}: {match}".format(matchNum = matchNum, start = match.start(), end = match.end(), match = match.group()))

            for groupNum in range(0, len(match.groups())):
                groupNum = groupNum + 1

                #print ("Group {groupNum} found at {start}-{end}: {group}".format(groupNum = groupNum, start = match.start(groupNum), end = match.end(groupNum), group = match.group(groupNum)))
                res.append(int(match.group(groupNum)))
        return res
                

    def readPropsFile(self,propsFile):
        lines = self.getLines(propsFile)
        goals = {}
        for line in lines:
            pline = line.strip()
            pline = pline.split(',')
            for i in range(len(pline)):
                p = pline[i]
                print(p)
                res=self.propRegex(p)
                if(len(res)>0):
                    #print(res)
                    goals[i] = res[0]
        print(goals)
        return goals
            
    def loadgoalsfile(self):
        d = GridGuiGoalsDialog(self.app)
        goalsToAssign = d.goalsArray
        
        fn = tkFileDialog.askdirectory(initialdir="/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/smallerwhdoors/",title="Open Properties File Directory")
        #askopenfilename(initialdir="/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/smallerwhdoors/",title="Open Properties File", filetypes=(("prop",".prop"),("props",".props"),("all files","*.*")))
        tempfn = self.fn.replace('.grid','')
        #folderend = fn.rfind('/')
        #print(fn)
        folder = fn+'/'#fn[0:folderend+1]
        folderend  = tempfn.rfind('/')
        tempfn = tempfn[folderend+1:]
        fn = folder+tempfn+'.prop'
        print(fn)
        
        goals = self.readPropsFile(fn)
        goalLabels = [goals[i] for i in goalsToAssign]
        allgoalsinorder = [goals[i] for i in goals]
        #clear all goals
        self.grid.clearGoals(allgoalsinorder)
        self.grid.setGoals(goalLabels)
        self.goals = goalLabels 
        #print (goalLabels)
            
        #print(goalsToAssign)
        
        
    def clearAllGoals(self,goalsList):
        self.grid.clearallgaols(goalsList)
        
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
                        daState = readMDPStaTra.getDAStatesFromState(parentstates[startState])
                        self.grid.setGoalsFromState(self.goals,self.goalsDAVals,daState)
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
                        daState = readMDPStaTra.getDAStatesFromState(cs)
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
                            daIndsChanged = readMDPStaTra.hasDAStateChanged(cs,ns)
                            if self.goalsDAVals is None:
                                self.goalsDAVals = {}
                            if len(self.goals) != len(self.goalsDAVals):
                                #just get all the da values for each goal bit
                                #so how do we do this
                                #we check if a da index has changed
                                #right we have all the goal labels
                                #we check if any of the new states have the same label as the goal
                                nsAgentsLocDict = readMDPStaTra.getAgentStatesFromState(ns)
                                #check if any of the ns are goal labels
                                for agnum in nsAgentsLocDict:
                                    if nsAgentsLocDict[agnum] in self.goals:
                                        if nsAgentsLocDict[agnum] in self.goalsDAVals:
                                            continue 
                                        #hmmm just pick the da that has changed
                                        for danum in daIndsChanged:
                                            if daIndsChanged[danum]:
                                                self.goalsDAVals[nsAgentsLocDict[agnum]]=danum
                                                print(self.goalsDAVals)
                                                
                            #print(daIndsChanged)

            

        
        
                    

    def exitHere(self):
        self.app.destroy()

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
            currcell.agents.append({'color':agent.defaultColor,'dead':agent.dead,'deadCounter':agent.deadCounter})

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
        self.menu.add_command(label='Load Grid File',command=self.openFile)
        self.menu.add_command(label='Set Goals',command=self.loadgoalsfile)
        self.menu.add_command(label='Load sta/tra file',command=self.loadstatra)
        
        

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
    
