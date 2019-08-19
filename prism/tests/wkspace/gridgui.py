from tkinter import *
import Tkinter, Tkconstants, tkFileDialog, tkSimpleDialog
#import pickle

from GeneratePrismFiles import GeneratePrismFile


app=None
MAP="map"
SETINITLOCS="initlocs"
SETGOALS="goals"
SETAVOID="avoid"
SETFS = "fail"
SETDOOR="door"
xside = 0
yside = 0

appState = MAP
gridArray = None
blockedCells = None
doorPairs = []
fn = ""

class Cell():
    FILLED_COLOR_BG = "black"
    EMPTY_COLOR_BG = "white"
    FILLED_COLOR_BORDER = "black"
    EMPTY_COLOR_BORDER = "black"
    START_COLOR_BG = "blue"
    START_COLOR_BORDER = "blue"
    GOAL_COLOR_BG="green"
    GOAL_COLOR_BORDER = "green"
    AVOID_COLOR_BG="red"
    AVOID_COLOR_BORDER="red"
    FAIL_COLOR_BG="grey"
    FAIL_COLOR_BORDER="grey"
    GOAL_FAIL_COLOR_BG="lightgreen"
    DOOR_COLOR_BG="brown"
    DOOR_COLOR_BORDER="magenta"
    

    def __init__(self, master, x, y, size):
        """ Constructor of the object called by Cell(...) """
        self.master = master#Canvas(master)
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
        self.label = str(x)+","+str(y)
        self.defaultLabel = str(x)+","+str(y)
        

    def __str__(self):
        strrep = '[(%d,%d,%d)*{"fill":%s,"isInitPos":%s,"isGoalPos":%s,"isFailState":%s,"isAvoidState":%s,"isDoor":%s' % (self.ord,self.abs,self.size,str(self.fill),str(self.isInitPos),str(self.isGoalPos),str(self.isFailState),str(self.isAvoidState),str(self.isDoor))
        if self.isDoor:
            strrep = strrep + ',"otherDoor":'+str(self.otherDoor)
        else:
            strrep = strrep + ',"otherDoor":None'
        strrep = strrep + "}]"
        return strrep
            

    def _switch(self):
        """ Switch if the cell is filled or not. """
        global appState 
        #print (appState)
        if appState is MAP:
            self.fill= not self.fill
        elif appState is SETINITLOCS:
            self.isInitPos = not self.isInitPos
        elif appState is SETAVOID:
            self.isAvoidState = not self.isAvoidState
        elif appState is SETFS:
            self.isFailState = not self.isFailState
        elif appState is SETGOALS:
            self.isGoalPos = not self.isGoalPos
        elif appState is SETDOOR:
            self.isDoor = not self.isDoor 
        else:
            print ("invalid app state")

    def draw(self):
        """ order to the cell to draw its representation on the canvas """
        if self.master != None :
            fill = Cell.EMPTY_COLOR_BG
            outline = Cell.EMPTY_COLOR_BORDER
            global appState
            
            #if appState is MAP:
            if self.fill:
                fill = Cell.FILLED_COLOR_BG
                outline = Cell.FILLED_COLOR_BORDER
                    
            #elif appState is SETINITLOCS:
            if self.isInitPos:
                fill = Cell.START_COLOR_BG
                outline = Cell.START_COLOR_BORDER
            #elif appState is SETAVOID:
            if self.isAvoidState:
                fill = Cell.AVOID_COLOR_BG
                outline = Cell.AVOID_COLOR_BORDER

            #elif appState is SETFS:
            if self.isFailState:
                fill = Cell.FAIL_COLOR_BG
                outline = Cell.FAIL_COLOR_BORDER
            #elif appState is SETGOALS:
            if self.isGoalPos:
                fill = Cell.GOAL_COLOR_BG
                if self.isFailState:
                    outline = Cell.FAIL_COLOR_BORDER
                    fill = Cell.GOAL_FAIL_COLOR_BG 
                else:
                    outline = Cell.GOAL_COLOR_BORDER
            #elif appState is SETDOOR:
            if self.isDoor:
                fill = Cell.DOOR_COLOR_BG
                outline = Cell.DOOR_COLOR_BORDER
                    
                
                    

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
                #strrep = '[(%d,%d,%d)*{"fill":%s,"isInitPos":%s,"isGoalPos":%s,"isFailState":%s,"isAvoidState":%s,"isDoor":%s' % (self.abs,self.ord,self.size,str(self.fill),str(self.isInitPos),str(self.isGoalPos),str(self.isFailState),str(self.isAvoidState),str(self.isDoor))
                xys = line.split('*')
                flags = xys[1].replace(']','')
                xys = xys[0]
                xys = xys.replace('[','')
                xys = xys.replace('(','')
                xys = xys.replace(')','')
                xys = xys.split(',')
                
                
                xydict[(int(xys[0]),int(xys[1]))] = eval(flags)
            #print xydict
            #print row
            #print column
            #print size 
                
        
        Canvas.__init__(self, master, width = cellSize * columnNumber , height = cellSize * rowNumber, *args, **kwargs)

        self.cellSize = cellSize

        self.grid = []
        #print rowNumber
        #print columnNumber
        #print cellSize
        for row in range(rowNumber):

            line = []
            for column in range(columnNumber):
                cell = Cell(self, column, row, cellSize)
                
                if fromFile:
                    flagsHere = xydict[(row,column)]
                    #print flagsHere
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
        #self.canvas = Canvas(self)

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
        cell._switch()
        cell.draw()
        #add the cell to the list of cell switched during the click
        self.switched.append(cell)

    def handleMouseMotion(self, event):
        row, column = self._eventCoords(event)
        cell = self.grid[row][column]

        if cell not in self.switched:
            cell._switch()
            cell.draw()
            self.switched.append(cell)

def setMap():
    global appState
    appState = MAP
    
def addInitialLocations():
    global appState
    appState = SETINITLOCS
    #print (appState)
    
def addGoals():
    global appState
    appState = SETGOALS
    #print (appState)

def addFailStates():
    global appState
    appState = SETFS

def addAvoidStates():
    global appState
    appState = SETAVOID

def addDoorStates():
    global appState
    appState = SETDOOR

def processDoors():
    #called every two pair of doors really
    global appState
    if appState is not SETDOOR:
        print "Error!!! you havent set doors. Click on add doors first"
    else:
        global doorPairs
        global gridArray

        if gridArray is not None:
            currentDoorPair = []
            for i in range(len(gridArray)):
                for j in range(len(gridArray[i])):
                    c = gridArray[i][j]
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
                d1g = gridArray[d1[0]][d1[1]]
                d2g = gridArray[d2[0]][d2[1]]
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


            print doorPairs
    
                    
def allDone():

    #global app
    #app.update()

    global fn
    
    fn = tkFileDialog.asksaveasfilename(initialdir="/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/guiFiles",title="Save file as", filetypes=(("grid",".grid"),("prism",".prism"),("prop",".prop"),("props",".props"),("all files","*.*")))
    app.update()
    fn = fn.split('.')[0]
    #get all the goal states and stuff
    goalStates = []
    failStates = []
    initStates = []
    blockedStates =[]
    avoidStates = []
    connectedStates=[]
    doDoors = False 
    global doorPairs
    if doorPairs == []:
        doDoors = True 
        doorPair =[]
        potentialDoors = []
        
    global gridArray
    if gridArray is not None:
        gridWriteFile = open(fn+'.grid','w')
        lenx = len(gridArray)
        leny = len(gridArray[0])
        gsize = gridArray[0][0].size
        gridWriteFile.write(str(lenx)+','+str(leny)+','+str(gsize)+'\n')
        
        for i in range(len(gridArray)):
            for j in range(len(gridArray[i])):
                c = gridArray[i][j]
                gridWriteFile.write(str(c)+'\n')
                d = (i,j)
                if c.fill:
                    blockedStates.append(d)
                elif c.isInitPos:
                    initStates.append(d)
                elif c.isGoalPos:
                    goalStates.append(d)
                    if c.isFailState:
                        failStates.append(d)
                elif c.isFailState:
                    failStates.append(d)
                elif c.isAvoidState:
                    avoidStates.append(d)
                if not c.fill:
                    connectedStates.append(d)
                if doDoors:
                    if c.isDoor:
                        if not d in doorPairs:
                            if c.otherDoor is None:
                                potentialDoors.append(d)
        
        #lets make door pairs
        #doors should be 1 apart
        #doorDistances ={}
        if doDoors:
            potDoorPairs = []
            for d in potentialDoors:
                for dd in potentialDoors:
                    if not d == dd:
                        dist = ((d[0]-dd[0])*(d[0]-dd[0])+(d[1]-dd[1])*(d[1]-dd[1]))
                        import math
                        dist = math.sqrt(dist)
                        if dist == 1.0:
                            potDoorPairs.append([d,dd])
            numDoors = len(potentialDoors)/2
            for i in range(len(potDoorPairs)):
                potDoors = potDoorPairs[i]
                if potDoors[0] not in doorPairs and potDoors[1] not in doorPairs:
                    doorPairs = doorPairs + potDoors
                
                    
        gridWriteFile.close()
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
    global doorPairs
    print ("doors")
    print (doorPairs)
    #raw_input()
    gfr = GeneratePrismFile()
    global xside
    global yside

    smap=gfr.generateFromGUIGrid(initStates,blockedStates,goalStates,avoidStates,failStates,connectedStates,doorPairs,xside,yside,fn)
    for ij in smap:
        i = ij[0]
        j = ij[1]
        lab = smap[ij]
        gridArray[i][j].label = lab
        gridArray[i][j].draw()

        
    global app
    #app.update()
    
    #just a for loop to wait for the dialog to close
    for i in range(1000000):
        xx=i
    for i in range(1000000):
        xx=i
    for i in range(1000000):
        xx=i

    #cv = Canvas(app)
    import os
    comm = "import -window root "+fn+".png"
    os.system(comm)
    #cv.postscript(file=fn+"_img.ps", colormode='color')

def screenshot():
    global fn 
    import os
    comm = "import -window root "+fn+".png"
    os.system(comm)
    #cv.postscript(file=fn+"_img.ps", colormode='color')


    
def exitHere():
    global app
    app.destroy()
    
if __name__ == "__main__" :

    #yside = int(input("Enter y side"))
    #reading the grid
    global app
    app = Tk()
    
    fn = tkFileDialog.askopenfilename(initialdir="/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/guiFiles/",title="Save file as", filetypes=(("grid",".grid"),("all files","*.*")))#"/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/guiFiles/kya.grid"
    print fn
    print type(fn)
    
    if fn:
        loadGridFromFile = True#open(fn)
    else:
        loadGridFromFile = False
        

    
    
    if not loadGridFromFile:
        global xside
        xside= tkSimpleDialog.askinteger('Num cells x','Num cells x',initialvalue=5,minvalue=1,maxvalue=100)
        if xside is None:
            xside = 5
        #xside = int(input("Enter x side"))
        global yside
        yside = tkSimpleDialog.askinteger('Num cells y','Num cells y',initialvalue=5,minvalue=1,maxvalue=100)
        if yside is None:
            yside = 5
    else:
        global xside
        xside = 5 
        global yside
        yside = 5
            
    menu = Menu(app)
    menu.add_command(label='Set Map',command=setMap)
    menu.add_command(label='Initial Locaitions',command = addInitialLocations)
    menu.add_command(label='Goals',command=addGoals)
    menu.add_command(label='Failstates',command=addFailStates)
    menu.add_command(label='Avoid States', command=addAvoidStates)
    menu.add_command(label='Add Doors', command=addDoorStates)
    menu.add_command(label='Add Door Pair' , command= processDoors)
    menu.add_command(label='Screenshot', command=screenshot)
    
    menu.add_command(label='done',command=allDone)
    #menu.add_command(label='load file',command=loadFile)
    
    menu.add_command(label='exit',command=exitHere)
    
    app.config(menu=menu)
    #app2 = Tk()
    
    #btn = Button(app2,text="Test")
    #btn.grid(column=51,row=51)
    size = 50
    #doing some calculations
    #if its 50 then is bad for 20
    #so say for a 15 by 15 it works

    sside = max(xside,yside)
    area = sside*sside
    fullsize = area*size*size
    maxsize = 18*18*50*50
    newsize = maxsize/area
    import math 
    newsize = math.sqrt(newsize)
    size = int(newsize)
    
    

    grid = CellGrid(loadGridFromFile,fn,app, xside,yside, size)
    global gridArray
    gridArray = grid.grid
    
    grid.pack()


    
    app.mainloop()
    #app2.mainloop()
