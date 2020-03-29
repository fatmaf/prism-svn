from tkinter import *
import Tkinter, Tkconstants, tkFileDialog, tkSimpleDialog
from Cell import Cell
from HelperClasses import CellAttributes
from HelperClasses import ShapePoints
from HelperClasses import ColourHelper
        
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
        self.isHidden = False
        self.justVisited = False 
        
        

    def draw(self):
        if self.drawGoal:
            fill = self.visitFill
            if self.visited:
                fill=self.visitedFill
            (xmin,xmax,ymin,ymax,xc,yc) = self.cell.getCellCoords()
            points = ShapePoints.getStarPoints(xc,yc,self.maxLen,self.maxWid)
            if self.canvasobj is not None:
                self.canvas.delete(self.canvasobj)
            self.canvasobj=self.canvas.create_polygon(points,fill=fill,outline=self.outline)
            self.canvas.lift(self.cell.labelObj)
        else:
            if self.canvasobj is not None:
                if not self.isHidden:
                    #hide object from canvas
                    self.canvas.itemconfigure(self.canvasobj, state='hidden')
                    self.isHidden = True

    def hideGoal(self):
        if not self.isHidden:
            self.drawGoal = False
            self.draw()

    def showGoal(self):
        self.drawGoal = True
        self.isHidden = False
        self.draw()

    def setVisited(self):
        if not self.visited:
            self.justVisited = True 
            self.visited = True
            self.draw()
        else:
            self.justVisited = False 

    def unsetVisited(self):
        if self.visited:
            self.visited = False
            self.draw()
            self.justVisited = False 

            
class GridGuiGoalsDialog(tkSimpleDialog.Dialog):

    def body(self,master):
        self.goalsList = '0,1,2,3,4,5,6,7,8,9'
        defaultGoalsList = StringVar(master,value=self.goalsList)
        Label(master,text="Goals List:").grid(row=0,sticky=W)
        self.gl=Entry(master,textvariable=defaultGoalsList)
        self.gl.grid(row=0,column=1)
        self.goalsArray = None 

    def apply(self):
        goalsList = self.gl.get()
        
        #validating
        import re
        pattern = re.compile(r"[\d\s?,]*")
        if not pattern.match(goalsList):
            print ("Invalid goals list pattern. Expected 1,2,3,4,5... Got "+goalsList+". Reverting to default "+self.goalsList)
            goalsList = self.goalsList
        goalsArray = eval('['+goalsList+']')
        self.goalsArray = goalsArray

        
class GoalsFileReader(object):
    
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
    
