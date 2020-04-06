from HelperClasses import CellAttributes
from HelperClasses import ShapePoints
from HelperClasses import ColourHelper


class Path(object):
    def __init__(self,pathNumber,firstState,cell,canvas,color):
        self.pathnum = pathNumber
        self.states = [firstState]
        self.cells = {firstState:cell}
        self.statesDrawn = []
        self.canvas = canvas
        self.color = color
        self.doLines = True
        self.fadeByVal = 0,1 

    def __repr__(self):
        #print path number
        retstr = str(self.pathnum)+": [ "
        for s in self.states:
            retstr = retstr + str(s) + " , "
        retstr = retstr + ' ] '
        retstr = retstr + 'doLines:'+str(doLines)
        retstr = retstr + ' statesDrawn:'+str(len(statesDrawn))
        return retstr

    def __str__(self):
        retstr = str(self.pathnum)+": [ "
        for s in self.states:
            retstr = retstr + str(s) + " , "
        retstr = retstr + ' ] '
        return retstr 

    def fadePath(self,agnum=0,agsize=None):
        self.color = ColourHelper.lighten_color(self.color,self.fadeByVal)
        self.clearPath()
        self.drawPath(agnum,agsize)
    
    def addStateCell(self,state,cell):
        #we want to check if the states have changed
        #if not we don't add them
        doaddStateCell = True 
        lastStateInd = len(self.states)-1
        if lastStateInd !=-1:
            lastState = self.states[lastStateInd]
            if lastState == state:
                doaddStateCell = False 
        if doaddStateCell:
            self.states.append(state)
            if state not in self.cells:
                self.cells[state] = cell

    def getAgentCenter(self,cell,agsize,agnum):
        if agsize is None:
            return cell.getCellCoords()
        else:
            (xmin,xmax,ymin,ymax,xc,yc) = cell.getCellCoords()
            agxy = Agent.agnumtoxy[agnum]
            xpos = xmin+agxy[0]*agsize
            ypos = ymin+agxy[1]*agsize
            xposl = xpos+agsize
            yposl = ypos+agsize
            xposc = (xpos+xposl)/2
            yposc = (ypos+yposl)/2

        return (xpos,ypos,xposl,yposl,xposc,yposc)


    def clearPath(self):
        for obj in self.statesDrawn:
            self.canvas.delete(obj)
        self.statesDrawn = [] 

    def shiftPath(self,shiftby):
        #we get the path number
        for obj in self.statesDrawn:
            #shift by x
            if obj is not None:
                #coords = self.canvas.coords(obj)
                #newcoords = (coords[0]+shiftby,coords[1])
                #self.canvas.coords(obj,newcoords)
                self.canvas.move(obj,0,shiftby)

    def drawPath(self,agnum=0,agsize=None):
        if self.doLines:
            self.drawPathLines(agnum,agsize)
        else:
            self.drawPathCircles(agnum,agsize)

    def drawPathCircles(self,agnum=0,agsize=None):
        dotsize = 2
        if agsize is not None:
            dotsize = agsize/10 
        for i in range(len(self.states)):
            if len(self.statesDrawn) < (i+1):
                cs = self.states[i]
                cscell = self.cells[cs]
                csxy=cscell.getXY()
                (xmin,xmax,ymin,ymax,xc,yc) = self.getAgentCenter(cscell,agsize,agnum)
                
                obj = self.canvas.create_oval(xc-dotsize,yc-dotsize,xc+dotsize,yc+dotsize,fill=self.color,outline='white')
                self.statesDrawn.append(obj)
                
    def drawPathLines(self,agnum=0,agsize=None):
        w = 2
        #print ("Moving from states ")
        #print (self.states)
        for i in range(len(self.states)-1):
            #we want to draw a path for each state
            #first we must determine whether we have drawn this state or not
            #print ("Anything we've drawn")
            #print (self.statesDrawn)
            if len(self.statesDrawn) < (i+1):
                #we have not drawn this state
                cs = self.states[i]
                ns = self.states[i+1]
                cscell = self.cells[cs]
                nscell = self.cells[ns]
                csxy = cscell.getXY()
                nsxy = nscell.getXY()
                dir = 'none'
                if (csxy[1] == nsxy[1]):
                    if (csxy[0] < nsxy[0]):
                        dir = 'right'
                    elif (csxy[0]>nsxy[0]):
                        dir = 'left'
                elif (csxy[0] == nsxy[0]):
                    if(csxy[1] < nsxy[1]):
                        dir = 'up'
                    elif (csxy[1] > nsxy[1]):
                        dir = 'down'
                (xmin,xmax,ymin,ymax,xc,yc) = self.getAgentCenter(cscell,agsize,agnum)
                (xminn,xmaxn,yminn,ymaxn,xcn,ycn) = self.getAgentCenter(nscell,agsize,agnum)
                obj = None
                #print ("Direction : "+dir)
                if dir == 'up' or dir == 'down':
                    #draw a line in the middle xc from one yc to the other
                    obj = self.canvas.create_line(xc,yc,xc,ycn,width=w,fill=self.color)
                elif dir == 'right' or dir == 'left':
                    obj = self.canvas.create_line(xc,yc,xcn,yc,width=w,fill=self.color)

                #print ("We've drawn this now ")
                #print (obj)
                self.statesDrawn.append(obj)
                


class Agent(object):
    agnumtoxy={0:(0,0),1:(0,1),2:(0,2),3:(1,0),4:(1,2),5:(2,0),6:(2,1),7:(2,2),8:(0,0),9:(0,1)}
    def __init__(self,canvas,cell,size,currloc,agcolor,num):
        self.canvas = canvas
        self.currloc = currloc
        self.cell = cell 
        self.size = size
        self.deadCounter = 0
        self.dead = False
        self.drawAgent = True
        self.colour = agcolor
        self.canvasobj = {'shape':None,'number':None}
        self.num = num
        self.paths = {}
        self.hidden = False
        self.aliveCounter = 0 

    def died(self):
        if not self.dead:
            self.dead = True
            self.deadCounter = self.deadCounter + 1
            self.draw()
            self.updateDeadCounter()

    def alive(self):
        if self.dead:
            self.aliveCounter = self.aliveCounter + 1
            self.dead=False
            self.draw()

    def getAgentCenter(self):
        return self.getAgentCenterCell(self.cell)
        
    def getAgentCenterCell(self,cell):
        agsize = self.size
        (xmin,xmax,ymin,ymax,xc,yc) = cell.getCellCoords()
        agnum = self.num 
        agxy = Agent.agnumtoxy[agnum]
        xpos = xmin+agxy[0]*agsize
        ypos = ymin+agxy[1]*agsize
        xposl = xpos+agsize
        yposl = ypos+agsize
        xposc = (xpos+xposl)/2
        yposc = (ypos+yposl)/2

        return (xpos,ypos,xposl,yposl,xposc,yposc)

    def hide(self):
        if not self.hidden:
            self.drawAgent = False
            self.draw()

    def show(self):
        if self.hidden or not self.drawAgent:
            self.hidden = False
            #self.draw()
            #if not self.drawAgent:
            self.drawAgent = True
            self.draw()
            
    def draw(self):
        if self.drawAgent:
            if self.hidden:
                self.canvas.itemconfigure(self.canvasobj['shape'],state='normal')
                self.canvas.itemconfigure(self.canvasobj['number'],state='normal')
            else:
                
                (xpos,ypos,xposl,yposl,xposc,yposc) = self.getAgentCenter()
                fill = self.colour #Cell.FILLCOLOURS[CellAttributes.INITIALLOC]
                outline='white'
                if self.dead:
                    if self.canvasobj['shape'] is not None:
                        self.canvas.delete(self.canvasobj['shape'])
                    crosspoints = ShapePoints.getCrossPoints(xpos,ypos,xposl,yposl)
                    self.canvasobj['shape']=self.canvas.create_polygon(crosspoints,fill=fill,outline=outline)
                else:
                    if self.canvasobj['shape'] is not None:
                        self.canvas.delete(self.canvasobj['shape'])

                    self.canvasobj['shape']=self.canvas.create_oval(xpos,ypos,xposl,yposl,fill=fill,outline=outline)
                    #xposc = (xpos+xposl)/2
                    #yposc = (ypos+yposl)/2
                if self.canvasobj['number'] is None:
                    self.canvasobj['number']=self.canvas.create_text((xposc,yposc),text=str(self.deadCounter))
                else:
                    self.canvas.lift(self.canvasobj['number'])
        else:
            if not self.hidden:
                self.hidden = True
                if self.canvasobj['shape'] is not None:
                    self.canvas.itemconfigure(self.canvasobj['shape'],state='hidden')
                if self.canvasobj['number'] is not None:
                    self.canvas.itemconfigure(self.canvasobj['number'],state='hidden')

    def updateDeadCounter(self):
        if self.canvasobj['number'] is None:
            self.canvasobj['number']=self.canvas.create_text((xposc,yposc),text=str(self.deadCounter))
        else:
            self.canvas.itemconfigure(self.canvasobj['number'],text=str(self.deadCounter))
                

    def getMoveLimits(self,nextcell):
        (xmin,xmax,ymin,ymax,xc,yc) = self.cell.getCellCoords()
        (xminn,xmaxn,yminn,ymaxn,xcn,ycn) = nextcell.getCellCoords()
        xmove = xminn-xmin
        ymove = yminn-ymin
        return (xmove,ymove)
    
    def updateCell(self,nextloc,nextcell,currentPath=0):
        if currentPath != -1:
            if currentPath not in self.paths:
                self.paths[currentPath] = Path(currentPath,self.currloc,self.cell,self.canvas,self.colour)
            self.paths[currentPath].addStateCell(nextloc,nextcell)
        
        #self.paths[currentPath].append({'loc':self.currloc,'cell':self.currcell,'ncell':nextcell}]
        (xmove,ymove) = self.getMoveLimits(nextcell)
        self.currloc = nextloc
        self.cell = nextcell
        #move the agent
        self.canvas.move(self.canvasobj['shape'],xmove,ymove)
        self.canvas.move(self.canvasobj['number'],xmove,ymove)
        self.drawAllPaths(currentPath,True)

    def drawAllPaths(self,currentPath,removeAllPreviousPaths=True):
        if self.drawAgent:
            for p in self.paths:
                if p == currentPath:
                    self.paths[p].drawPath(self.num,self.size)
                else:
                    if removeAllPreviousPaths:
                        self.paths[p].clearPath()
                    else:
                        if currentPath-p < 10:
                            shiftby = 2
                            self.paths[p].fadePath(self.num,self.size)
                            self.paths[p].shiftPath(shiftby)
                        else:
                            self.paths[p].clearPath()
                            

    def shiftAllPaths(self,shiftby):
        if self.drawAgent:
            for p in self.paths:
                self.paths[p].fadePath(self.num,self.size)
                self.paths[p].shiftPath(shiftby)


    def getPathString(self,pathNum):
        if pathNum in self.paths:
            return str(self.paths[pathNum])
        else:
            return None

    
            


                
