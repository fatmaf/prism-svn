import matplotlib.colors as mc
import math
from tkinter import *
import Tkinter, Tkconstants, tkFileDialog, tkSimpleDialog
from Cell import Cell
from HelperClasses import CellAttributes
from HelperClasses import ShapePoints
from HelperClasses import ColourHelper
from GoalsUtilities import Goal
from Agent import Agent

from readStaTra import ReadMDPStaTra
from simulatePolicy import SimulatePolicy


# a subclass of Canvas for dealing with resizing of windows
class ResizingCanvas(Canvas):
    
    def __init__(self,parent,**kwargs):
        Canvas.__init__(self,parent,**kwargs)
        self.bind("<Configure>", self.on_resize)
        self.height = self.winfo_reqheight()
        self.width = self.winfo_reqwidth()
        self.div = 2 

    def on_resize(self,event):
        # determine the ratio of old width/height to new width/height
        windowWidth = event.width/self.div
        windowHeight = event.height
        wscale = float(windowWidth)/self.width
        hscale = float(windowHeight)/self.height
        self.width = windowWidth
        self.height = windowHeight
        # resize the canvas 
        self.config(width=self.width, height=self.height)
        # rescale all the objects tagged with the "all" tag
        self.scale("all",0,0,wscale,hscale)
        

class CanvasWorld(object):
    
    ONESIDE = (16.0*50.0)
    MAXSIZE=(ONESIDE*ONESIDE)
    MESSAGEAREASIZE = 100.0
    ROBOT_COLORS_LIST=[mc.XKCD_COLORS['xkcd:aqua'],mc.XKCD_COLORS['xkcd:beige'],mc.XKCD_COLORS['xkcd:coral'],mc.XKCD_COLORS['xkcd:fuchsia'],mc.XKCD_COLORS['xkcd:indigo'],mc.XKCD_COLORS['xkcd:goldenrod'],mc.XKCD_COLORS['xkcd:khaki'],mc.XKCD_COLORS['xkcd:lavender'],mc.XKCD_COLORS['xkcd:lightblue'],mc.XKCD_COLORS['xkcd:tomato'],mc.XKCD_COLORS['xkcd:pink'],mc.XKCD_COLORS['xkcd:teal']]

    def canvasbbox(self):
        
        print('  def _canvas(self):')
        print('self.cv.winfo_rootx() = ', self.canvas.winfo_rootx())
        print('self.cv.winfo_rooty() = ', self.canvas.winfo_rooty())
        print('self.cv.winfo_x() =', self.canvas.winfo_x())
        print('self.cv.winfo_y() =', self.canvas.winfo_y())
        print('self.cv.winfo_width() =', self.canvas.winfo_width())
        print('self.cv.winfo_height() =', self.canvas.winfo_height())
        x=self.canvas.winfo_rootx()+self.canvas.winfo_x()
        y=self.canvas.winfo_rooty()+self.canvas.winfo_y()
        x1=x+self.canvas.winfo_width()
        y1=y+self.canvas.winfo_height()
        box=(x,y,x1,y1)
        print('box = ', box)
        return box
    
    def calculateDimensions(self,size=50,ncols=10,nrows=10):
        maxCellsSide = max(ncols,nrows)
        numCells = maxCellsSide*maxCellsSide
        newsize = CanvasWorld.MAXSIZE/float(numCells)
        newsize = math.sqrt(newsize)
        #self.printMessage("One side "+str(newsize))
        newsize = int(newsize)
        #self.printMessage("Max Area = "+str(CanvasWorld.MAXSIZE))
        #self.printMessage("Original Size = "+str(size)+" New Size = "+str(newsize))
        return newsize

    def __init__(self,name):
        self.canvas = None
        self.cellsize = None
        self.rows = None
        self.cols = None
        self.name = name
        self.goals = None
        self.agents = None
        self.grid = None
        self.goalNumbersDict = None
        self.currentGoals = None
        self.policyObj = None
        self.currentAgents = None
        self.currentGoalsToDAInds = None
        self.messageArea = None
        self.titleArea = None
        self.paths={}
        self.statesVisited=[]
        self.policySim = None
        self.doPolicySim = False
        self.goalsVisited= 0 

    def listPaths(self):
        for p in self.paths:
            self.printMessage(str(p)+":"+str(self.paths[p]))
        
    def printMessage(self,toprint):
        toprint = str(toprint)
        print (self.name+":"+toprint)
        

    def initialiseOrResetCanvas(self,app,size=50,ncols=10,nrows=10):
        size = self.calculateDimensions(size,ncols,nrows)
        w = ncols*size
        h = nrows*size
        h = h+CanvasWorld.MESSAGEAREASIZE
        self.rows = nrows
        self.cols = ncols
        
        if self.canvas is not None:
            self.canvas.delete('all')
        else:
            self.canvas = ResizingCanvas(app,width=w,height=h)#Canvas(app,width=w, height=h)
        self.cellsize = size 

    def fillCanvasWithCells(self,xydict=None):
        self.agents = {}
        self.grid = []
        self.goals = {}
        doorsList = {}
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
                            doorsList[(row,col)]=cell
                            #see if we have a corresponding door
                            doorOptions = [(row-1,col),(row,col-1),(row+1,col),(row,col+1)]
                            #doorFound = False 
                            for doorOption in doorOptions:
                                if doorOption in doorsList:
                                    cell.otherDoor = doorsList[doorOption].getXY()
                                    doorsList[doorOption].otherDoor = cell.getXY()
                                    #self.printMessage(doorOption)
                                    #self.printMessage((row,col))
                                    #raw_input(doorsList)
                                    #doorFound = True
                                    #del doorsList[doorOption]
                                    break
                            #if not doorFound:
                                
                                    
                                    
                            
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
        allDone = False 
        if self.currentAgents is None:
            for ag in self.agents:
                agent = self.agents[ag]
                currloc = agent.currloc
                if (currloc+1) in self.labelsmap:
                    agent.updateCell(currloc+1,self.labelsmap[currloc+1])
                    if (currloc+1) in self.goals:
                        self.goals[currloc+1].setVisited()
                        
                else:
                    allDone = True 
        else:
            if self.policyObj is not None:
                if self.currentJointState is not None:
                    #if self.pathNumber not in self.paths:
                    #    self.paths[self.pathNumber]=[]
                    prevJS = self.currentJointState
                    if self.currentJointState not in self.statesVisited:
                        self.statesVisited.append(self.currentJointState)
                    self.moveAgentsFromPolicy()
                    if self.currentJointState is not None:
                        agentsLocDict = self.policyObj.getAgentStatesFromState(self.currentJointState)
                        newJs = self.currentJointState
                        if self.pathNumber not in self.paths:
                            self.paths[self.pathNumber]=[]
                        self.paths[self.pathNumber].append((prevJS,newJs))
                        changedAndTrueDAStates = None
                        if self.currentGoalsToDAInds is None:
                            changedDAStates = self.policyObj.hasDAStateChanged(prevJS,newJs)
                            trueDAStates = self.policyObj.getDAStatesFromState(self.currentJointState)
                            #changedandtrue
                            changedAndTrueDAStates = {}
                            for danum in changedDAStates:
                                if danum in trueDAStates:
                                    changedAndTrueDAStates[danum]=True
                        elif len(self.currentGoalsToDAInds) != len(self.currentGoals):
                            changedDAStates = self.policyObj.hasDAStateChanged(prevJS,newJs)
                            trueDAStates = self.policyObj.getDAStatesFromState(self.currentJointState)
                            changedAndTrueDAStates = {}
                            for danum in changedDAStates:
                                if danum in trueDAStates:
                                    changedAndTrueDAStates[danum]=True
                        self.moveAgentsFromAgentLocDict(agentsLocDict,self.pathNumber,changedAndTrueDAStates)
                        self.updateMessageArea(self.pathNumber)
                    else:
                        #if prevJS is not None:
                        #    self.printMessage("Last State "+str(self.policyObj.staDict[prevJS]))
                        if self.statesVisited is not None:
                            unvisitedStates=[]
            
                            for state in self.policyObj.staDict:
                                if state not in self.statesVisited:
                                    unvisitedStates.append(state)
                            self.updateStatusText(unvisitedStates)
                            self.printMessage("Unvisited States"+str(unvisitedStates))
                            allDone = True
                else:
                    allDone= True 
   
        return allDone

                            

    def moveAgentsToStartState(self):
        startState = 0 
        agentsLocDict = self.policyObj.getAgentStatesFromState(startState)
        self.moveAgentsFromAgentLocDict(agentsLocDict,-1)
        daStates = None
        self.resetGoals(daStates)
                        

    def moveAgentsFromAgentLocDict(self,agentsLocDict,pathNumber,changedDAStates=None):
        numAgentsMoved = 0
        for agnum in agentsLocDict:
            coragnum = self.currentAgents[agnum]
            agent = self.agents[coragnum]
            currloc = agent.currloc
            nextloc = agentsLocDict[agnum]
            if(nextloc != currloc) and nextloc!=-1 and currloc !=-1:
                numAgentsMoved = numAgentsMoved+1
            if nextloc in self.labelsmap:
                agent.alive()
                agent.updateCell(nextloc,self.labelsmap[nextloc],pathNumber)
                if nextloc in self.goals:
                    self.goals[nextloc].setVisited()
                    if changedDAStates is not None:
                        if self.currentGoalsToDAInds is None:
                            self.currentGoalsToDAInds = {}
                        if len(self.currentGoalsToDAInds) != len(self.currentGoals):
                            if len(changedDAStates) > 1:
                                self.printMessage("Two da states changed at the same time, can not determine goal")
                            else:
                                self.printMessage("Determining Goal")
                                self.printMessage(changedDAStates)
                                self.printMessage(nextloc)
                                for danum in changedDAStates:
                                    if self.goals[nextloc].justVisited:
                                        if nextloc not in self.currentGoalsToDAInds:
                                            self.currentGoalsToDAInds[nextloc]=danum
                                            #self.printMessage(self.currentGoalsToDAInds)
                                        else:
                                            self.printMessage("Old DA Num"+str(self.currentGoalsToDAInds[nextloc])+" new danum"+str(danum))
                                    
                                
            if nextloc == -1:
                agent.died()
            #if(numAgentsMoved >0):
            self.updateStatusText(str(numAgentsMoved)+" agents moved")
            
            
        
    def drawAgents(self):
        if self.currentAgents is None:
            for ag in self.agents:
                self.agents[ag].draw()
        else:
            for ag in self.agents:
                if ag in self.currentAgentsList:
                    self.agents[ag].show()
                else:
                    self.agents[ag].hide()
                    
                    
                


    def resetGoals(self,daStates):
        if daStates is None:
            #just reset them all
            for gl in self.currentGoals:
                self.goals[gl].unsetVisited()
                self.printMessage("Unsetting goal "+str(gl))
        else:
            if self.currentGoalsToDAInds is not None:
                if len(self.currentGoalsToDAInds) == len(self.currentGoals):
                    for gl in self.currentGoalsToDAInds:
                        danum = self.currentGoalsToDAInds[gl]
                        self.printMessage("Goal: "+str(gl)+" danum: "+str(danum))
                        if danum not in daStates:
                            #unset goal
                            self.goals[gl].unsetVisited()
                            self.printMessage("Unsetting goal "+str(gl))
        
                
    def drawGoals(self):
        if self.currentGoals is None:
            for gl in self.goals:
                self.goals[gl].draw()
        else:
            for gl in self.goals:
                if gl in self.currentGoals:
                    self.goals[gl].showGoal()
                else:
                    self.goals[gl].hideGoal()

    

    def getCanvasEdgeY(self):
        centerx = self.rows*self.cellsize/2
        self.textInc = 10 
        edgey = self.cols*self.cellsize+self.textInc
        return (centerx,edgey)
       
    def drawTitleText(self):
        (centerx,edgey) = self.getCanvasEdgeY()
        if self.titleArea is not None:
            self.canvas.delete(self.titleArea)
        self.titleArea = self.canvas.create_text((centerx,edgey),text=self.name)
        
    def drawMessageAreaText(self):
        (centerx,edgey) = self.getCanvasEdgeY()
        edgey = edgey+self.textInc
        if self.messageArea is not None:
            if type(self.messageArea) is dict:
                for obj in self.messageArea:
                    self.canvas.delete(self.messageArea[obj])
                self.messageArea = None
            else:
                self.canvas.delete(self.messageArea)
                self.messageArea = None
        if self.currentAgents is not None:
            #we need a line for each agent
            self.messageArea={}
            for ag in self.currentAgentsList:
                agcolor = self.agents[ag].colour
                self.messageArea[ag] = self.canvas.create_text((0,edgey),text='[]',fill=agcolor,anchor=Tkinter.NW)
                edgey = edgey + self.textInc+5
            self.messageArea['status']=self.canvas.create_text((0,edgey),text='Status',anchor=Tkinter.NW)
        else:
            self.messageArea = self.canvas.create_text((0,edgey),text='Status Bar',anchor=Tkinter.NW)
                
            
    def drawCells(self):
        for row in self.grid:
            for cell in row:
                cell.draw()

        
    def packCanvas(self,side):
        self.canvas.pack(side=side,expand=True,fill='both')
        self.drawCells()
        self.drawGoals()
        self.drawAgents()
        self.drawTitleText()
        self.drawMessageAreaText()
        

    def updateStatusText(self,txt):
        if self.messageArea is not None:
            if type(self.messageArea) is dict:
                self.canvas.itemconfigure(self.messageArea['status'],text=txt)
            else:
                self.canvas.itemconfigure(self.messageArea,text=txt)

                
    def updateMessageArea(self,currentPath):
        if self.currentAgents is not None:
            for ag in self.currentAgentsList:
                agPath = self.agents[ag].getPathString(currentPath)
                if agPath is not None:
                    self.canvas.itemconfigure(self.messageArea[ag],text=agPath)


    def shiftAgentPaths(self,shiftby):
        if self.currentAgents is None:
            for ag in self.agents:
                self.agents[ag].shiftAllPaths(shiftby)
        else:
            for ag in self.currentAgentsList:
                self.agents[ag].shiftAllPaths(shiftby)
                


    def setCurrentGoalsAndGoalDict(self,goalDict,goalNumbers):
        if goalDict is not None:
            self.goalNumbersDict = goalDict
        if goalNumbers is not None:
            self.currentGoals = [goalDict[i] for i in goalNumbers]
            #redraw goals
            self.drawGoals()

    def setPolicyObj(self,staname,traname):
        self.printMessage("Reading Policy From File: "+staname+"\n"+traname)
        self.policyObj = ReadMDPStaTra(staname,traname)
        self.policyObj.readsta()
        self.policyObj.readtra()
        startState = 0
        self.setAgents(startState)
        self.drawMessageAreaText()
        self.currentJointState = startState
        self.pathNumber = 0
        self.statesToExploreLater=[]
        self.parentStates={}
        self.parentPaths={}


    def setPolicySimObj(self,mdpname):
        self.policySim = SimulatePolicy(None,mdpname)
        if self.policyObj is not None:
            self.policySim.polobj = self.policyObj
            
    def setAgents(self,state):
        agentsLocDict = self.policyObj.getAgentStatesFromState(state)
        if self.currentAgents is None:
            self.currentAgents = {}
            self.currentAgentsList=[]
            #we need to set the current Agents
            #just do all new agents
            for ag in self.agents:
                agloc = self.agents[ag].currloc
                for aag in agentsLocDict:
                    aagloc = agentsLocDict[aag]
                    if aagloc == agloc:
                        self.currentAgents[aag]=ag
                        self.currentAgentsList.append(ag)
            self.drawAgents()
        


    def anyDeadRobots(self,agentsLocDict):
        anyAgentsDead = False
        deadAgentNumbers = []
        for ag in agentsLocDict:
            if agentsLocDict[ag] == -1:
                deadAgentNumbers.append(ag)
        if len(deadAgentNumbers) > 0:
            anyAgentsDead = True
        return (anyAgentsDead,deadAgentNumbers)
            
    def getUndeadPositions(self,state):
        import copy
        agentsLocDict = self.policyObj.getAgentStatesFromState(state)
        firstagentslocdict = copy.deepcopy(agentsLocDict)
        #self.printMessage("First State")
        #self.printMessage(firstagentslocdict)
        (anyAgentsDead,deadAgentNumbers) = self.anyDeadRobots(agentsLocDict)
        previousDeadAgentNumbers = copy.deepcopy(deadAgentNumbers)
        stateToConsider = state 
        while(anyAgentsDead):
            if stateToConsider in self.parentStates:
                self.printMessage("Child State"+str(stateToConsider))
                stateToConsider = self.parentStates[stateToConsider]
                self.printMessage("Parent State"+str(stateToConsider))
            else:
                stateToConsider = None
            if stateToConsider is not None:
                agentsLocDict = self.policyObj.getAgentStatesFromState(stateToConsider)
                (anyAgentsDead,deadAgentNumbers) = self.anyDeadRobots(agentsLocDict)
                if(len(deadAgentNumbers) < len(previousDeadAgentNumbers)):
                    #then we've got a possible state
                    for agnum in previousDeadAgentNumbers:
                        if agnum not in deadAgentNumbers:
                            firstagentslocdict[agnum]=agentsLocDict[agnum]
                previousDeadAgentNumbers = copy.deepcopy(deadAgentNumbers)
            else:
                anyAgentsDead = False 

        #self.printMessage("Undead States")
        #self.printMessage(firstagentslocdict)
        return firstagentslocdict
        
                
    def moveAgentsFromPolicy(self):
        if not self.doPolicySim:
            nextJSes=self.policyObj.getMostProbableStateReactive(self.currentJointState)
            if nextJSes is not None:
                mostProbableNextJS = nextJSes[0]
                for js in nextJSes[1]:
                    if js not in self.statesToExploreLater:
                        self.statesToExploreLater.append(js)
                        self.parentStates[js]=self.currentJointState
                        self.parentPaths[js]=self.pathNumber
                        self.printMessage("Ignored State's Parent "+str(self.parentStates[js]))
                        self.printMessage("Ignored State "+str(js))
                        self.printMessage("Sibling State "+str(mostProbableNextJS))

                agentsLocDict = self.policyObj.getAgentStatesFromState(mostProbableNextJS)
                self.currentJointState = mostProbableNextJS
            else:
                if len(self.statesToExploreLater) > 0:
                    self.printMessage("Previous State: "+str(self.policyObj.getAgentStatesFromState(self.currentJointState)))
                    self.currentJointState = self.statesToExploreLater.pop()
                    self.pathNumber = self.pathNumber + 1
                    undeadPositions=self.getUndeadPositions(self.currentJointState)
                    self.moveAgentsFromAgentLocDict(undeadPositions,-1)
                    #also reset any goals
                    daStates = self.policyObj.getDAStatesFromState(self.currentJointState)
                    self.printMessage("New DA states? "+str(daStates))
                    self.resetGoals(daStates)
                else:
                    self.updateStatusText("All Done!!")
                    self.currentJointState = None
        else:
            nextJS = self.policySim.getNextState(self.currentJointState)
            self.currentJointState = nextJS

                
                


    def resetEverythingForPolSim(self):
        glvisited = 0
        for gl in self.currentGoals:
            if self.goals[gl].visited:
                glvisited = glvisited +1
        self.moveAgentsToStartState()
        self.currentJointState = 0
        self.pathNumber = self.pathNumber+ 1
        return glvisited


    def showRandomAgentsAndGoals(self,numgoals,numagents):
        import random
        agkeys = self.agents.keys()
        ags = random.sample(agkeys,numagents)

        gkeys = self.goals.keys()
        gs = random.sample(gkeys,numgoals)

        for ag in self.agents:
            if ag in ags:
                self.agents[ag].show()
            else:
                self.agents[ag].hide()

        for g in self.goals:
            if g in gs:
                self.goals[g].showGoal()
            else:
                self.goals[g].hideGoal()
                
        
        
            
                    
