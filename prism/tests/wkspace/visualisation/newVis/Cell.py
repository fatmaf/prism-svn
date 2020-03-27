from HelperClasses import CellAttributes
from HelperClasses import ShapePoints
from HelperClasses import ColourHelper

class Cell(object):

    FILLCOLOURS = {CellAttributes.BLOCKED:'black',CellAttributes.EMPTY:'white',CellAttributes.GOAL:'green',CellAttributes.AVOID:'red',CellAttributes.FAILSTATE:'lightgrey',CellAttributes.DOOR:'brown',CellAttributes.INITIALLOC:'blue'}

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
            
        

    def drawDoor(self,xmin,ymin,xmax,ymax):
        doorwidth = self.size/20
        linew = doorwidth
        fill = Cell.FILLCOLOURS[CellAttributes.DOOR]
        d = self.getXY()
        od = self.otherDoor
        #print (d)
        #print (od)
        if d[1] == od[1]:
            #create door on x
            if d[0] < od[0]:
                self.canvas.create_line(xmax,ymin,xmax,ymax,w=linew,dash=(3,5),fill=fill)
                #self.canvas.create_rectangle(xmax-doorwidth,ymin,xmax,ymax,stipple='gray75',fill=fill)
                #print ("Drawing at xmax,")
            else:
                self.canvas.create_line(xmin,ymin,xmin,ymax,w=linew,dash=(3,5),fill=fill)
                #self.canvas.create_rectangle(xmin,ymin,xmin+doorwidth,ymax,stipple='gray75',fill=fill)
                #print ("Drawing at xmin")
        else:
            if d[1] < od[1]:
                self.canvas.create_line(xmin,ymax,xmax,ymax,w=linew,dash=(3,5),fill=fill)
            else:
                self.canvas.create_line(xmin,ymin,xmax,ymin,w=linew,dash=(3,5),fill=fill)
        #        self.canvas.create_rectangle(xmin,ymax-doorwidth,xmax,ymax,stipple='gray75',fill=fill)
        #    else:
        #        self.canvas.create_rectangle(xmin,ymin,xmax,ymin+doorwidth,stipple='gray75',fill=fill)
        #if d[0] == 1:
        #self.canvas.create_line(xmax,ymin+self.size/4,xmax-self.size/4,ymax-self.size/3,width=2,fill=fill)
        #self.canvas.create_line(xmax-self.size/4,ymax-self.size/3,xmax,ymax-self.size/4,width=2,fill=fill)
        #self.canvas.create_rectangle(
            
            
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
        if CellAttributes.DOOR in self.cellAttributes:
            self.drawDoor(xmin,ymin,xmax,ymax)
        #if CellAttributes.GOAL in self.cellAttributes:
        #    self.drawGoal(xc,yc)
        #if CellAttributes.INITIALLOC in self.cellAttributes:
        #    self.drawAgent(xmin,ymin,0)
        
        self.labelObj=self.canvas.create_text((xc,yc),text=self.label)
