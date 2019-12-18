from tkinter import *
import Tkinter, Tkconstants, tkFileDialog, tkSimpleDialog

from GeneratePrismFiles import GeneratePrismFile

from enum import Enum


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
        

    def __str__(self):
        strrep = '[(%d,%d,%d)*{"fill":%s,"isInitPos":%s,"isGoalPos":%s,"isFailState":%s,"isAvoidState":%s,"isDoor":%s' % (self.ord,self.abs,self.size,str(self.fill),str(self.isInitPos),str(self.isGoalPos),str(self.isFailState),str(self.isAvoidState),str(self.isDoor))
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
        self.doFSVar = Tkinter.IntVar()
        self.doGSVar = Tkinter.IntVar()
        self.doISVar = Tkinter.IntVar()
        self.doDVar = Tkinter.IntVar()
        
        Label(master,text="Number of Random Files:").grid(row=0,sticky=W)
        self.en = Entry(master)
        self.en.grid(row=0,column=1)
        Label(master,text="Randomise:").grid(row=1)

        self.cbfs = Checkbutton(master,text="Fail states",variable=self.doFSVar)
        self.cbfs.grid(row=2,columnspan=2,sticky=W)

        self.cbgs = Checkbutton(master,text="Goals",variable = self.doGSVar)
        self.cbgs.grid(row=3,columnspan=2,sticky=W)

        self.cbis = Checkbutton(master,text="Initial Locations",variable=self.doISVar)
        self.cbis.grid(row=4,columnspan=2,sticky=W)

        self.cbd = Checkbutton(master,text="Increment Doors",variable = self.doDVar)
        self.cbd.grid(row=5,columnspan=2,sticky=W)
        
    def apply(self):
        numRandomFiles = int(self.en.get())
        doFS = bool(self.doFSVar.get())
        doGS = bool(self.doGSVar.get())
        doIS = bool(self.doISVar.get())
        doD = bool(self.doDVar.get())
        
        print numRandomFiles
        print doFS
        print doGS
        print doIS
        print doD
        self.result = (numRandomFiles,doFS,doGS,doIS,doD)
        self.numRandomFiles = numRandomFiles
        self.doFS = doFS
        self.doGS = doGS
        self.doIS = doIS
        self.doD = doD
        
        
        
        #return (numRandomFiles,doFS,doGS,doIS)

        
    
class GridGui(object):
    
        
    def __init__(self):
        self.app = None
        self.gridArray = None
        self.blockedCells = None
        self.doorPairs = []
        self.fn = ""
        self.xside = 5
        self.yside = 5
        self.grid = None
        self.loadGridFromFile = False
        self.size = 50
        self.gridArray = None
        self.numFilesToGenerate = 0
        self.randomiseFS = False
        self.randomiseGoals = False
        self.randomiseInitLocs = False
        self.centerGoals = False
        self.clusterGoals = False
        self.enumerateDoors = False
        
        
    def addDoorStates(self):
        self.grid.appState = GuiState.DOORS

    def processDoors(self):
        doorPairs = self.doorPairs
        
        if self.grid.appState is not GuiState.DOORS:
            print 'ERROR - SET DOORS BEFORE ADDING DOOR PAIR'
        else:
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
                if len(currentDoorPair) == 2:
                    doorPairs = doorPairs + currentDoorPair
                elif len(currentDoorPair) < 2 and len(currentDoorPair) > 0:
                    print "Door Pairs not equal"

                #mark doors
                for i in range(0,len(doorPairs),2):
                    d1 = doorPairs[i]
                    d2 = doorPairs[i+1]
                    d1g = self.gridArray[d1[0]][d1[1]]
                    d2g = self.gridArray[d2[0]][d2[1]]
                    if d1g.otherDoor is None:
                        if d2g.otherDoor is None:
                            continue
                        else:
                            print "mismatched door pair?? "
                            print doorPairs
                            print d1
                            print d2

                    d1g.otherDoor = d2
                    d2g.otherDoor = d1
                    self.gridArray[d1[0]][d1[1]] = d1g
                    self.gridArray[d2[0]][d2[1]]= d2g


                print doorPairs
                self.doorPairs = doorPairs
                

    def setMap(self):
        self.grid.appState = GuiState.OBSTACLES

    def addInitialLocations(self):
        self.grid.appState = GuiState.INITLOCS

    def addGoals(self):
        self.grid.appState = GuiState.GOALS

    def addFailStates(self):
        self.grid.appState = GuiState.FAILSTATES

    def addAvoidStates(self):
        self.grid.appState = GuiState.AVOIDSTATES

    def centerGoalsFunc(self):
        self.centerGoals = True
        print 'Centering Goals'
        if self.clusterGoals:
            self.clusterGoals = False
            print 'Set cluster goals to False' 

    def sideClusterGoals(self):
        self.clusterGoals = True
        print 'Clustering Goals to Side'
        if self.centerGoals:
            self.centerGoals = False
            print 'Set center goals to False'
            

    def screenshot(self):
        self.screenshotfn(self.fn)
        
    def screenshotfn(self,fn):
        import os
        comm = "import -window root "+fn+".png"
        os.system(comm)
        

    def groundToEdgeX(self,x,dist,blockedStates,avoidStates):
        gridArray = self.gridArray
        lenx = len(gridArray)
        leny = len(gridArray[0])

        discardStates = blockedStates+avoidStates
        #print discardStates
        statesToPickFrom = [] 
        for i in range(x,x+dist):
            for j in range(len(gridArray[i])):
                d=(i,j)
                if not d in discardStates:
                    statesToPickFrom.append(d)
        return statesToPickFrom

    def groundToEdgeY(self,y,dist,blockedStates,avoidStates):
        gridArray = self.gridArray
        discardStates = blockedStates+avoidStates
        statesToPickFrom = []

        for i in range(len(gridArray)):
            for j in range(y,y+dist):
                d = (i,j)
                if not d in discardStates:
                    statesToPickFrom.append(d)

        return statesToPickFrom

    def groundToCenter(self,dist,blockedStates,avoidStates):
        gridArray = self.gridArray
        lenx = len(gridArray)
        leny = len(gridArray[0])
        discardStates = blockedStates + avoidStates
        statesToPickFrom = []
        cx = lenx/2
        cy = leny/2
        cx=cx-1
        cy=cy-1
        print (cx,cy)
        #print (cx-dist)
        #print (cx+dist)
        
        for i in range(cx-dist,cx+dist+1):
            for j in range(cy-dist,cy+dist+1):
                d = (i,j)
                if not d in discardStates:
                    if self.euclideanDist(d,(cx,cy)) <= dist:
                        statesToPickFrom.append(d)
        return statesToPickFrom
    

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

                    
    def writeGridFileAndCollectStates(self,fn):
        gridArray = self.gridArray
        
        goalStates = []
        failStates = []
        initStates = []
        blockedStates =[]
        avoidStates = []
        connectedStates=[]
        doDoors = False
        xedge=[]
        yedge=[]
        center=[]
        
        if self.doorPairs == []:
            doDoors = True 
            doorPair =[]
            potentialDoors = []

        gridWriteFile = open(fn+'.grid','w')
        lenx = len(gridArray)
        leny = len(gridArray[0])
        gsize = gridArray[0][0].size
        gridWriteFile.write(str(lenx)+','+str(leny)+','+str(gsize)+'\n')
        #lets make sure we dont select failstates from doors

        failStateChoices = []
        for i in range(len(gridArray)):
            for j in range(len(gridArray[i])):
                c = gridArray[i][j]
                gridWriteFile.write(str(c)+'\n')
                d = (i,j)
                if c.fill:
                    blockedStates.append(d)
                else:
                    failStateChoices.append(d)
                if c.isInitPos:
                    initStates.append(d)
                if c.isGoalPos:
                    goalStates.append(d)
                if c.isFailState:
                    failStates.append(d)
                if c.isAvoidState:
                    avoidStates.append(d)
                if not c.fill:
                    connectedStates.append(d)
                if doDoors:
                    if c.isDoor:
                        if not d in doorPairs:
                            if c.otherDoor is None:
                                potentialDoors.append(d)
                if doDoors:
                    if d in potentialDoors:
                        failStateChoices.remove(d)
                if not self.doorPairs == []:
                    if d in self.doorPairs:
                        failStateChoices.remove(d)
                        

        if doDoors:
            doorPairs = []
            potDoorPairs = []
            for d in potentialDoors:
                for dd in potentialDoors:
                    if not d == dd:
                        dist = self.euclideanDist(d,dd)
                        if dist == 1.0:
                            potDoorPairs.append([d,dd])
            numDoors = len(potentialDoors)/2
            for i in range(len(potDoorPairs)):
                potDoors = potDoorPairs[i]
                if potDoors[0] not in doorPairs and potDoors[1] not in doorPairs:
                    doorPairs = doorPairs + potDoors
            self.doorPairs = doorPairs
            

        gridWriteFile.close()

        #print avoidStates
        #just testing this stuff
        #dist = 2
        #print self.groundToEdgeX(0,dist,blockedStates,avoidStates)
        #print self.groundToEdgeX(lenx-dist,dist,blockedStates,avoidStates)
        #print self.groundToEdgeY(0,dist,blockedStates,avoidStates)
        #print self.groundToEdgeY(lenx-dist,dist,blockedStates,avoidStates)
        #print self.groundToCenter(dist,blockedStates,avoidStates)
        #raw_input()
        return (initStates,blockedStates,goalStates,avoidStates,failStates,connectedStates,self.doorPairs,failStateChoices)

    def euclideanDist(self,d,dd):
        dist = ((d[0]-dd[0])*(d[0]-dd[0])+(d[1]-dd[1])*(d[1]-dd[1]))
        import math
        dist = math.sqrt(dist)
        return dist

    def groundToEdgeXY(self,xmin,xmax,ymin,ymax,dist,blockedStates,avoidStates):
        toret = self.groundToEdgeX(xmin,dist,blockedStates,avoidStates)
        toret = toret + self.groundToEdgeX(xmax,dist,blockedStates,avoidStates)
        toret = toret + self.groundToEdgeY(ymin,dist,blockedStates,avoidStates)
        toret = toret + self.groundToEdgeY(ymax,dist,blockedStates,avoidStates)

        return toret
    
    def generateRandomInitGoalStates(self,avoidStates,blockedStates,goalStates,initStates,numFilesToGenerate):
        gsLists = []
        isLists = []
        numRobots = len(initStates)
        numGoals = len(goalStates)
        lenx = len(self.gridArray)
        leny = len(self.gridArray[0])
        goalChoices = goalStates
        initChoices = initStates
        
        if self.centerGoals:
            #put goals at the center
            #put initial states at the edges

            print 'Generating lists for centering goals'
            import math 
            distGoals = int(math.sqrt(numGoals))
            goalChoices = self.groundToCenter(distGoals,blockedStates,avoidStates)
            while(len(goalChoices) <= numGoals):
                distGoals = distGoals+1
                goalChoices = self.groundToCenter(distGoals,blockedStates,avoidStates)
            #put all the initial states on the sides
            distInitLocs = numRobots/4
            if(distInitLocs == 0):
                distInitLocs = 1
            initChoices = self.groundToEdgeXY(0,lenx-distInitLocs,0,leny-distInitLocs,distInitLocs,blockedStates+goalChoices,avoidStates)
            while(len(initChoices) <= numRobots):
                distInitLocs = distInitLocs+1
                while(lenx-distInitLocs < 0):
                    distInitLocs = distInitLocs-1
                while(leny-distInitLocs < 0):
                    distInitLocs = distInitLocs-1
                initChoices = self.groundToEdgeXY(0,lenx-distInitLocs,0,leny-distInitLocs,distInitLocs,blockedStates+goalChoices,avoidStates)

                
        elif self.clusterGoals:
            print 'Generating lists for clustering goals'
            distGoals = numGoals/lenx
            distInitLocs = numRobots/lenx
            
            goalChoices = self.groundToEdgeX(lenx-distInitLocs,distGoals,blockedStates,avoidStates)
            while (len(goalChoices) <= numGoals):
                distGoals = distGoals+1
                goalChoices = self.groundToEdgeX(lenx-distInitLocs,distGoals,blockedStates,avoidStates)
                
            initChoices = self.groundToEdgeX(0,distInitLocs,blockedStates+goalChoices,avoidStates)
            while (len(initChoices) <= numRobots):
                distInitLocs = distInitLocs+1
                initChoices = self.groundToEdgeX(0,distInitLocs,blockedStates+goalChoices,avoidStates)
                
            

        import random 
        for numf in range(numFilesToGenerate):
            gsLists.append(random.sample(goalChoices,numGoals))
            isLists.append(random.sample(initChoices,numRobots))
            print 'Goals'
            print gsLists[numf]
            print 'Init States'
            print isLists[numf]

        return (gsLists,isLists)
                               

                
    def generateRandomFailStates(self,failStates,failStateChoices,connectedStates,fn,numFilesToGenerate):
        gridArray = self.gridArray

        originalFailStates = []
        for fs in failStates:
            originalFailStates.append(fs)

        maxNumFailStates = len(failStates)
        numConnectedStates = len(connectedStates)

        fsLists = []
        #fs bins
        #so like I want a nice resolution
        if(maxNumFailStates < 11):
            fsStep = 1
        elif maxNumFailStates < 51:
            fsStep = 5
        elif maxNumFailStates < 101:
            fsStep = 10
        else:
            fsStep = 10

        import random
        print "Generating Random states"
        if(maxNumFailStates > 0):
            for numfs in range(1,maxNumFailStates,fsStep):
                for nfs in range(numFilesToGenerate):
                    print numfs
                    print len(failStateChoices) 
                    fsLists.append(random.sample(failStateChoices,numfs))
                    print fsLists[nfs]
                    print "printing len"
                    print len(fsLists[nfs])
                print fsLists
            print "all fail states"
            print fsLists
        return fsLists
    

    def writeMultipleFiles(self,fsLists,gsLists,isLists,blockedStates,avoidStates,connectedStates,doorPairs,doFour):
        fscount = 0
        gscount = 0
        iscount = 0 
        currentFSLen = len(fsLists[0])
        for i in range(len(fsLists)):
            if(len(fsLists[i]))!= currentFSLen:
                currentFSLen = len(fsLists[i])
                fscount = 0
            fscount = fscount + 1
            gfr = GeneratePrismFile()


            fn = self.fn+'_fs'+str(len(fsLists[i]))+"_f"+str(fscount)+"_r"+str(iscount)+"_t"+str(gscount)
            if self.enumerateDoors:
                fn = fn+'_d'+str(len(doorPairs)/2)+'_';
                
            smap=gfr.generateFromGUIGrid(isLists[iscount],blockedStates,gsLists[gscount],avoidStates,fsLists[i],connectedStates,doorPairs,self.xside,self.yside,fn,doFour)
            print gsLists[gscount]
            print isLists[iscount]

            if gscount < len(gsLists)-1:
                gscount = gscount + 1
            else:
                gscount = 0

            if iscount < len(isLists)-1:
                iscount = iscount + 1
            else:
                iscount = 0

            for ij in smap:
                i = ij[0]
                j = ij[1]
                lab = smap[ij]
                self.gridArray[i][j].label = lab
                self.gridArray[i][j].draw()
            self.writeGridFileOnly(fn)
            self.app.update()

            #just a for loop to wait for the dialog to close
            for i in range(1000000):
                xx=i
            for i in range(1000000):
                xx=i
            for i in range(1000000):
                xx=i
            self.screenshotfn(fn)

            
    def allDone(self):
        
        self.fn = tkFileDialog.asksaveasfilename(initialdir="/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/guiFiles",title="Save file as", filetypes=(("grid",".grid"),("prism",".prism"),("prop",".prop"),("props",".props"),("all files","*.*")))
        self.app.update()
        self.fn = self.fn.split('.')[0]
        
        #get all the goal states and stuff


        if self.gridArray is not None:
            (initStates,blockedStates,goalStates,avoidStates,failStates,connectedStates,doorPairs,failStateChoices) = self.writeGridFileAndCollectStates(self.fn)
            fsLists = [failStates] * self.numFilesToGenerate
            gsLists = [goalStates] * self.numFilesToGenerate
            isLists = [initStates] * self.numFilesToGenerate
            
            if self.randomiseFS:
                fsLists = self.generateRandomFailStates(failStates,failStateChoices,connectedStates,self.fn,self.numFilesToGenerate)
            if self.randomiseGoals or self.randomiseInitLocs:
                (gsLists,isLists) = self.generateRandomInitGoalStates(avoidStates,blockedStates,goalStates,initStates,self.numFilesToGenerate)

        print ("Init")
        print (initStates)
        print ("Goals")
        print (goalStates)
        print ("Fail")
        print (failStates)
        print ("Avoid")
        print (avoidStates)
        print ("blocked")
        print (blockedStates)
        print ("open")
        print (connectedStates)
        print ("doors")
        print (doorPairs)

        gfr = GeneratePrismFile()
        doFour = True #four grid actions

        smap=gfr.generateFromGUIGrid(initStates,blockedStates,goalStates,avoidStates,failStates,connectedStates,doorPairs,self.xside,self.yside,self.fn,doFour)
        for ij in smap:
            i = ij[0]
            j = ij[1]
            lab = smap[ij]
            self.gridArray[i][j].label = lab
            self.gridArray[i][j].draw()
        self.writeGridFileOnly(self.fn)

        #just a for loop to wait for the dialog to close
        for i in range(1000000):
            xx=i
        for i in range(1000000):
            xx=i
        for i in range(1000000):
            xx=i

        self.screenshot()

        if self.enumerateDoors:
            for i in range(1,len(doorPairs),2):
                tempDoorPairs = doorPairs[0:i+1]
                print 'doors'
                print tempDoorPairs
                if self.randomiseFS or self.randomiseGoals or self.randomiseInitLocs:
                    self.writeMultipleFiles(fsLists,gsLists,isLists,blockedStates,avoidStates,connectedStates,tempDoorPairs,doFour)
                else:
                    smap=gfr.generateFromGUIGrid(initStates,blockedStates,goalStates,avoidStates,failStates,connectedStates,tempDoorPairs,self.xside,self.yside,self.fn+"_d"+str(i/2)+"_",doFour)
                    
        else:
            if self.randomiseFS or self.randomiseGoals or self.randomiseInitLocs:
                self.writeMultipleFiles(fsLists,gsLists,isLists,blockedStates,avoidStates,connectedStates,doorPairs,doFour)
            
        print "Done"


    def exitHere(self):
        self.app.destroy()

    def generateMultiples(self):
        d = GridGuiRandomiseDialog(self.app)
        self.numFilesToGenerate  = d.numRandomFiles
        self.randomiseFS = d.doFS
        self.randomiseGoals = d.doGS
        self.randomiseInitLocs = d.doIS
        self.enumerateDoors = d.doD
        
        

    def createGuiMenu(self):
        self.menu = Menu(self.app)
        self.menu.add_command(label='Set Map',command=self.setMap)
        self.menu.add_command(label='Initial Locaitions',command = self.addInitialLocations)
        self.menu.add_command(label='Goals',command=self.addGoals)
        self.menu.add_command(label='Failstates',command=self.addFailStates)
        self.menu.add_command(label='Avoid States', command=self.addAvoidStates)
        self.menu.add_command(label='Add Doors', command=self.addDoorStates)
        self.menu.add_command(label='Add Door Pair' , command= self.processDoors)
        self.menu.add_command(label='Center Goals', command=self.centerGoalsFunc)
        self.menu.add_command(label='Side Goals',command=self.sideClusterGoals)
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
    
