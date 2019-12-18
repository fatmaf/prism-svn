from tkinter import *
import Tkinter, Tkconstants, tkFileDialog, tkSimpleDialog

from GeneratePrismFilesForestFail import GeneratePrismFile

from enum import Enum

import random

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
    import matplotlib.colors as mc
    import colorsys
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
    CLEARPATH=8
    

class Cell():
    BORDER_COLORS = {GuiState.OBSTACLES:'#000000',GuiState.EMPTY:'#000000',GuiState.INITLOCS:'#0000FF',GuiState.GOALS:'#00FF00',GuiState.AVOIDSTATES:'#FF0000',GuiState.FAILSTATES:'#888888',GuiState.DOORS:'#923A00',GuiState.CLEARPATH:'#923A00'}
    FILL_COLORS= {GuiState.OBSTACLES:'#000000',GuiState.EMPTY:'#FFFFFF',GuiState.INITLOCS:'#0000FF',GuiState.GOALS:'#00FF00',GuiState.AVOIDSTATES:'#FF0000',GuiState.FAILSTATES:'#888888',GuiState.DOORS:'#923AFF',GuiState.CLEARPATH:'#923A00'}

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
        self.isClearPath = False; 
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
        self.isClearPath = False

        
    def __str__(self):
        strrep = '[(%d,%d,%d)*{"fill":%s,"isInitPos":%s,"isGoalPos":%s,"isFailState":%s,"isAvoidState":%s,"isDoor":%s,"isClearPath":%s,"label":"%s"' % (self.ord,self.abs,self.size,str(self.fill),str(self.isInitPos),str(self.isGoalPos),str(self.isFailState),str(self.isAvoidState),str(self.isDoor),str(self.isClearPath),str(self.label))
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
        elif appState is GuiState.CLEARPATH:
            self.isClearPath = not self.isClearPath
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

            if self.isClearPath:
                if not isNothing:
                    self.clear_all_flags()
                fill = Cell.FILL_COLORS[GuiState.CLEARPATH]
                outline = Cell.BORDER_COLORS[GuiState.CLEARPATH]
                    
                
                    

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
                    cell.isClearPath = flagsHere["isClearPath"]
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
        defaultNumRandomFiles = StringVar(master,value='0')
        defaultNumRobots = StringVar(master,value='2')
        defaultNumGoals = StringVar(master,value='3')
        defaultNumFS = StringVar(master,value='0')
        defaultNumAS = StringVar(master,value='0')
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

        
    def apply(self):
        numRandomFiles = int(self.en.get())
        numRobots = int(self.rn.get())
        numGoals = int(self.gn.get())
        numFS = int(self.fsn.get())
        numAS = int(self.an.get())
        print numRandomFiles
        self.result = (numRandomFiles,numRobots,numGoals,numFS,numAS)
        self.numRandomFiles = numRandomFiles
        self.numRobots = numRobots
        self.numGoals = numGoals
        self.numFS = numFS
        self.numAS = numAS


        
    
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

    def addClearPath(self):
        self.grid.appState=GuiState.CLEARPATH

    def screenshot(self):
        self.screenshotfn(self.fn)
        
    def screenshotfn(self,fn):
        import os
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
        clearPathStates = []
        
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
                    if c.isClearPath:
                        clearPathStates.append(d)
                        
        #now lets clean up
        #we can actually clean up later
        #and enforce an order
        #yeah that would be nice
        #if we dont have any failStateChoices or goalChoices or avoidChoices we set them to
        #those from the depot Choices and openStates
        stateChoices = None
        if len(depotChoices) == 0:
            depotChoices = openStates
            print 'You did not choose depot areas!!!!!'
            
        openStatesExceptClearPath = list(set(openStates).symmetric_difference(set(clearPathStates)))
        
        if len(failStateChoices) == 0:
            failStateChoices = openStatesExceptClearPath
        if len(goalChoices) == 0:
            if stateChoices is None:
                stateChoices = list(set(openStatesExceptClearPath).symmetric_difference(set(depotChoices)))
            goalChoices = stateChoices
        if len(avoidChoices) == 0:
            if stateChoices is None:
                stateChoices = list(set(openStatesExceptClearPath).symmetric_difference(set(depotChoices)))
            avoidChoices = stateChoices
            
        return (depotChoices,failStateChoices,goalChoices,avoidChoices,blockedStates,openStates,clearPathStates)
            
    def allDone(self):
        
        self.fn = tkFileDialog.asksaveasfilename(initialdir="/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/guiFiles",title="Save file as", filetypes=(("grid",".grid"),("prism",".prism"),("prop",".prop"),("props",".props"),("all files","*.*")))
        self.app.update()
        self.fn = self.fn.split('.')[0]
        
        #get all the goal states and stuff


        if self.gridArray is not None:
            (depotChoices,failStatesChoices,goalChoices,avoidChoices,blockedStates,openStates,clearPathStates) = self.getStateLists()
            
            print "Depot Choices"
            print depotChoices
            print "Fail State Choices"
            print failStatesChoices
            print "Goal Choices"
            print goalChoices
            print "Forest Choices"
            print avoidChoices
            print "Blocked States"
            print blockedStates
            print "Open States"
            print openStates
            print "Clear Path States"
            print clearPathStates

            print "Num Robots"
            print self.numRobots
            print "Num Goals"
            print self.numGoals
            print "Num Fail States"
            print self.numFS
            print "Num Forest States"
            print self.numAS

            doorPairs = []
            
            #now lets choose some
            #we choose in an order really
            #first we choose the initial locations
            #then we choose the goals
            #then we choose the avoid states
            #then we choose the failstate
            if self.numFS > len(failStatesChoices):
                self.numFS = len(failStatesChoices)
            numFSInc = 1
            numFSSteps = 10 
            if self.numFS > 10:
                numFSInc = (self.numFS-1)/numFSSteps
                
            for i in range(self.numFilesToGenerate):
                if self.numFS>0:
                    j=self.numFS
                    openStatesExceptClearPath = list(set(openStates).symmetric_difference(set(clearPathStates)))

                    chosenInitialLocations = random.sample(depotChoices,self.numRobots)
                    if(len(goalChoices))== 0:
                        goalChoices = list(set(openStatesExceptClearPath).symmetric_difference(set(chosenInitialLocations)))
                    chosenGoalLocations = random.sample(goalChoices,self.numGoals)
                    if(len(avoidChoices)) == 0:
                        avoidChoices = list(set(openStatesExceptClearPath).symmetric_difference(set(chosenInitialLocations)))
                    statesToPickFrom = list(set(avoidChoices).symmetric_difference(set(chosenGoalLocations)))
                    if len(statesToPickFrom) == len(avoidChoices)+len(chosenGoalLocations):
                        statesToPickFrom = avoidChoices
                    chosenAvoidStates = random.sample(statesToPickFrom,self.numAS)
                    statesToPickFrom = failStatesChoices
                    chosenFailStates = random.sample(statesToPickFrom,j)

                    print "Chosen Things"
                    print "Initial Locations"
                    print chosenInitialLocations
                    print "Goals"
                    print chosenGoalLocations
                    print "Forest States"
                    print chosenAvoidStates
                    print "Fail States"
                    print chosenFailStates

                    gfr = GeneratePrismFile()
                    doFour = True #four grid actions

                    fn = self.fn +"_fs"+str(j)+ "_"+str(i)+"_"

                    smap=gfr.generateFromGUIGrid(chosenInitialLocations,blockedStates,chosenGoalLocations,chosenAvoidStates,chosenFailStates,openStates,doorPairs,self.xside,self.yside,fn,doFour)
                    self.updateGridArray(chosenInitialLocations,blockedStates,chosenGoalLocations,chosenAvoidStates,chosenFailStates,openStates,smap)
                    self.screenshotfn(fn)
                    self.writeGridFileOnly(fn)

                else:
                    chosenInitialLocations = random.sample(depotChoices,self.numRobots)
                    chosenGoalLocations = random.sample(goalChoices,self.numGoals)
                    statesToPickFrom = list(set(avoidChoices).symmetric_difference(set(chosenGoalLocations)))
                    if len(statesToPickFrom) == len(avoidChoices)+len(chosenGoalLocations):
                        statesToPickFrom = avoidChoices
                    chosenAvoidStates = random.sample(statesToPickFrom,self.numAS)
                    statesToPickFrom = failStatesChoices
                    chosenFailStates = random.sample(statesToPickFrom,self.numFS)

                    print "Chosen Things"
                    print "Initial Locations"
                    print chosenInitialLocations
                    print "Goals"
                    print chosenGoalLocations
                    print "Avoid States"
                    print chosenAvoidStates
                    print "Fail States"
                    print chosenFailStates

                    gfr = GeneratePrismFile()
                    doFour = True #four grid actions

                    fn = self.fn +"_fs"+str(self.numFS)+ "_"+str(i)+"_"

                    smap=gfr.generateFromGUIGrid(chosenInitialLocations,blockedStates,chosenGoalLocations,chosenAvoidStates,chosenFailStates,openStates,doorPairs,self.xside,self.yside,fn,doFour)
                    self.updateGridArray(chosenInitialLocations,blockedStates,chosenGoalLocations,chosenAvoidStates,chosenFailStates,openStates,smap)
                    self.screenshotfn(fn)
                    self.writeGridFileOnly(fn)                    

            
            #for ij in smap:
            #    i = ij[0]
            #    j = ij[1]
            #    lab = smap[ij]
            #    self.gridArray[i][j].label = lab
            #    self.gridArray[i][j].draw()
        print "Done"


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
        
        

    def createGuiMenu(self):
        self.menu = Menu(self.app)
        self.menu.add_command(label='Shelves',command=self.setShelves)
        self.menu.add_command(label='Depot',command = self.setDepotArea)
        self.menu.add_command(label='Possible Pickups',command=self.possibleGoals)
        self.menu.add_command(label='Failstates',command=self.possibleFailStates)
        self.menu.add_command(label='Areas To Avoid', command=self.addAvoidStates)
        self.menu.add_command(label='Clear Path',command=self.addClearPath)
        #self.menu.add_command(label='Add Doors', command=self.addDoorStates)
        #self.menu.add_command(label='Add Door Pair' , command= self.processDoors)
        #self.menu.add_command(label='Center Goals', command=self.centerGoalsFunc)
        #self.menu.add_command(label='Side Goals',command=self.sideClusterGoals)
        self.menu.add_command(label='Generate Multiple Files', command=self.generateMultiples)
        self.menu.add_command(label='Screenshot', command=self.screenshot)

        self.menu.add_command(label='done',command=self.allDone)
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
            biggerside = max(self.xside,self.yside)
            area = biggerside*biggerside
            fullsize = area*self.size*self.size
            maxsize = 18*18*50*50
            newsize = maxsize/area
            import math
            newsize = math.sqrt(newsize)
            self.size = int(newsize)
        if newFile or self.loadGridFromFile:
            self.grid = CellGrid(self.loadGridFromFile,self.fn,self.app,self.xside,self.yside,self.size)
            self.gridArray = self.grid.grid

            self.grid.pack()
            self.app.mainloop()
        

if __name__=="__main__":
    gridGui = GridGui()
    gridGui.createGui()
    
