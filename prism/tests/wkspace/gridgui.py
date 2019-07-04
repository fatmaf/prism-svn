from tkinter import *
import Tkinter, Tkconstants, tkFileDialog, tkSimpleDialog

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
            
            if appState is MAP:
                if self.fill:
                    fill = Cell.FILLED_COLOR_BG
                    outline = Cell.FILLED_COLOR_BORDER
                    
            elif appState is SETINITLOCS:
                if self.isInitPos:
                    fill = Cell.START_COLOR_BG
                    outline = Cell.START_COLOR_BORDER
            elif appState is SETAVOID:
                if self.isAvoidState:
                    fill = Cell.AVOID_COLOR_BG
                    outline = Cell.AVOID_COLOR_BORDER

            elif appState is SETFS:
                if self.isFailState:
                    fill = Cell.FAIL_COLOR_BG
                    outline = Cell.FAIL_COLOR_BORDER
            elif appState is SETGOALS:
                if self.isGoalPos:
                    fill = Cell.GOAL_COLOR_BG
                    outline = Cell.GOAL_COLOR_BORDER
            elif appState is SETDOOR:
                if self.isDoor:
                    fill = Cell.DOOR_COLOR_BG
                    outline = Cell.DOOR_COLOR_BORDER
                    
                
                    

            xmin = self.abs * self.size
            xmax = xmin + self.size
            ymin = self.ord * self.size
            ymax = ymin + self.size

            self.master.create_rectangle(xmin, ymin, xmax, ymax, fill = fill, outline = outline)

class CellGrid(Canvas):
    def __init__(self,master, rowNumber, columnNumber, cellSize, *args, **kwargs):
        Canvas.__init__(self, master, width = cellSize * columnNumber , height = cellSize * rowNumber, *args, **kwargs)

        self.cellSize = cellSize

        self.grid = []
        for row in range(rowNumber):

            line = []
            for column in range(columnNumber):
                line.append(Cell(self, column, row, cellSize))

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


            print doorPairs
    
                    
def allDone():
    
    fn = tkFileDialog.asksaveasfilename(initialdir="/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/",title="Save file as", filetypes=(("prism",".prism"),("prop",".prop"),("props",".props"),("all files","*.*")))
    fn = fn.split('.')[0]
    #get all the goal states and stuff
    goalStates = []
    failStates = []
    initStates = []
    blockedStates =[]
    avoidStates = []
    connectedStates=[]
    global gridArray
    if gridArray is not None:
        for i in range(len(gridArray)):
            for j in range(len(gridArray[i])):
                c = gridArray[i][j]
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
    gfr = GeneratePrismFile()
    global xside
    global yside
    
    gfr.generateFromGUIGrid(initStates,blockedStates,goalStates,avoidStates,failStates,connectedStates,doorPairs,xside,yside,fn)
    global app
    #cv = Canvas(app)
    import os
    comm = "import -window root "+fn+".png"
    os.system(comm)
    #cv.postscript(file=fn+"_img.ps", colormode='color')

def exitHere():
    global app
    app.destroy()
    
if __name__ == "__main__" :

    #yside = int(input("Enter y side"))
    global app
    app = Tk()
    global xside
    xside= tkSimpleDialog.askinteger('Num cells x','Num cells x',initialvalue=5,minvalue=2,maxvalue=100)
    if xside is None:
        xside = 5
    #xside = int(input("Enter x side"))
    global yside
    yside = tkSimpleDialog.askinteger('Num cells y','Num cells y',initialvalue=5,minvalue=2,maxvalue=100)
    if yside is None:
        yside = 5
    menu = Menu(app)
    menu.add_command(label='Set Map',command=setMap)
    menu.add_command(label='Initial Locaitions',command = addInitialLocations)
    menu.add_command(label='Goals',command=addGoals)
    menu.add_command(label='Failstates',command=addFailStates)
    menu.add_command(label='Avoid States', command=addAvoidStates)
    menu.add_command(label='Add Doors', command=addDoorStates)
    menu.add_command(label='Add Door Pair' , command= processDoors)
    
    menu.add_command(label='done',command=allDone)
    menu.add_command(label='exit',command=exitHere)
    
    app.config(menu=menu)
    #app2 = Tk()
    
    #btn = Button(app2,text="Test")
    #btn.grid(column=51,row=51)
    grid = CellGrid(app, xside,yside, 50)
    global gridArray
    gridArray = grid.grid
    
    grid.pack()


    
    app.mainloop()
    #app2.mainloop()
