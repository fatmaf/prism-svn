from enum import Enum
import matplotlib.colors as mc
import colorsys

class Agent(object):
    class AgentActions(Enum):
        LEFT=0
        RIGHT=1
        UP=2
        DOWN=3
        
    ACTIONEFFECTS = {AgentActions.DOWN:(0,1),AgentActions.UP:(0,-1),AgentActions.LEFT:(-1,0),AgentActions.RIGHT:(1,0)}
    
    def move(self,actionlabel):
        actiontuple = Agent.ACTIONEFFECTS[actionlabel]
        updatedLocation = (self.location[0]+actiontuple[0] ,self.location[1]+actiontuple[1])
        self.location = updatedLocation
        
        
    def __init__(self):
        self.location = None
        self.color = None
        self.name = None
        self.dead = False
        self.deadCounter = 0

    def died(self):
        if not self.dead:
            self.dead = True
            self.deadCounter = self.deadCounter + 1

    def alive(self):
        if self.dead:
            self.dead = False 
        
class Goal(object):
    def __init__(self):
        self.visited = False
        self.location = None

    def visited(self):
        self.visited = True

    def clear(self):
        self.visited = False

        
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
    BORDER_COLORS = {GuiState.OBSTACLES:'#000000',GuiState.EMPTY:'#000000',GuiState.INITLOCS:'#FFFFFF',GuiState.GOALS:'#00FF00',GuiState.AVOIDSTATES:'#FF0000',GuiState.FAILSTATES:'#888888',GuiState.DOORS:'#923A00'}
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
        self.defaultColor = None
        self.agents=None
        self.goal=None
        self.lines=None
        

    def clear_all_flags(self):
        self.fill= False
        self.isInitPos = False
        self.isGoalPos = False
        self.isFailState = False
        self.isAvoidState = False
        self.isDoor = False
        self.otherDoor = None
        self.agents = None

        
    def __str__(self):
        strrep = '[(%d,%d,%d)*{"fill":%s,"isInitPos":%s,"isGoalPos":%s,"isFailState":%s,"isAvoidState":%s,"isDoor":%s,"label":"%s"' % (self.ord,self.abs,self.size,str(self.fill),str(self.isInitPos),str(self.isGoalPos),str(self.isFailState),str(self.isAvoidState),str(self.isDoor),str(self.label))
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

    def getStarPoints(self,sxc,syc,p,t):
        points = []
        for i in (1,-1):
            points.extend((sxc,syc+i*p))
            points.extend((sxc+i*t,syc+i*t))
            points.extend((sxc+i*p,syc))
            points.extend((sxc+i*t,syc-i*t))
        return points

    def getCrossPoints(self,sx,sy,ex,ey):
        xlen =abs (ex - sx)
        ylen = abs(ey - sy )
        #cross points
        crosspoints = [(0,0),(0.2,0),(0.5,0.3),(0.8,0),(1,0),(1,0.2),(0.7,0.5),(1,0.8),(1,1),(0.8,1),(0.5,0.7),(0.2,1),(0,1),(0,0.8),(0.3,0.5),(0,0.2)]
        #first we've got to scale all the points
        minscale = min(xlen,ylen)
        mcrosspoints = []
        for t in crosspoints:
            mt = (t[0]*minscale+sx,t[1]*minscale+sy)
            mcrosspoints.append(mt)
        return mcrosspoints
        

    def draw(self):
        """ order to the cell to draw its representation on the canvas """
        isNothing = True
        if self.master != None :
            
            fill = Cell.FILL_COLORS[GuiState.EMPTY]
            outline = Cell.BORDER_COLORS[GuiState.EMPTY]
            
            if self.fill:
                self.clear_all_flags()
                self.fill = True 
                isNothing = False 
                fill = Cell.FILL_COLORS[GuiState.OBSTACLES]
                outline = Cell.BORDER_COLORS[GuiState.OBSTACLES]
                
                    
            if self.isInitPos:
                isNothing = False
                
            if self.isAvoidState:
                isNothing = False 
                fill = Cell.FILL_COLORS[GuiState.AVOIDSTATES]
                outline = Cell.BORDER_COLORS[GuiState.AVOIDSTATES]
                
            if self.isGoalPos:
                isNothing = False
                fillcolor = Cell.FILL_COLORS[GuiState.GOALS]
                if self.goal is not None:
                    if self.goal.visited:
                        fillcolor=self.mixColor(fillcolor,'orange')
                fill = fillcolor #Cell.FILL_COLORS[GuiState.GOALS]
                outline = Cell.BORDER_COLORS[GuiState.GOALS]
                
            if self.isDoor:
                isNothing = False
                fill = Cell.FILL_COLORS[GuiState.DOORS]
                outline = Cell.BORDER_COLORS[GuiState.DOORS]

            if self.isFailState:
                if not isNothing:
                    #if not self.isInitPos:
                    if fill == Cell.FILL_COLORS[GuiState.EMPTY]:
                        fill = Cell.FILL_COLORS[GuiState.FAILSTATES]
                    fill = self.lighten_color(fill,0.5)
                    outline = Cell.BORDER_COLORS[GuiState.FAILSTATES]
                    #else:
                    #    fill = Cell.FILL_COLORS[GuiState.FAILSTATES]
                    #    outline = Cell.BORDER_COLORS[GuiState.FAILSTATES]
                else:
                    fill = Cell.FILL_COLORS[GuiState.FAILSTATES]
                    outline = Cell.BORDER_COLORS[GuiState.FAILSTATES]
                    
                
                    

            xmin = self.abs * self.size
            xmax = xmin + self.size
            ymin = self.ord * self.size
            ymax = ymin + self.size
            xc = (xmax-xmin)/2 + xmin
            yc = (ymax-ymin)/2 + ymin
            crad = self.size/4


            self.master.create_rectangle(xmin, ymin, xmax, ymax, fill = fill, outline = outline)
            if self.isInitPos:
                dotiling = self.agents is not None
                xpos = xmin+crad
                ypos = ymin+crad
                d = self.getXY()
                #process agents
                if self.agents is not None:
                    if dotiling:
                        divsize = self.size/3
                        for i in range(len(self.agents)):
                            ag = self.agents[i]
                            agcolor = ag['color']
                            agx = i/3
                            agy = i%3
                            xpos = xmin+agx*divsize
                            ypos = ymin+agy*divsize
                            xposl = xpos+divsize
                            yposl = ypos+divsize
                            if ag['dead']:
                                cp = self.getCrossPoints(xpos,ypos,xposl,yposl)
                                self.master.create_polygon(cp,fill=agcolor,outline=Cell.BORDER_COLORS[GuiState.INITLOCS])
                            else:
                                self.master.create_oval(xpos,ypos,xposl,yposl,fill=agcolor,outline=Cell.BORDER_COLORS[GuiState.INITLOCS])
                            xposc = (xpos+xposl)/2
                            yposc = (ypos+yposl)/2
                            numdead = ag['deadCounter']
                            self.master.create_text((xposc,yposc),text=str(numdead))
                    else:

                        alivecolor = None
                        deadcolor = None
                        for ag in self.agents:
                            agcolor = ag['color']
                            if ag['dead']:
                                deadcolor = self.mixColor(deadcolor,agcolor)
                            else:
                                alivecolor = self.mixColor(alivecolor,agcolor)
                        if alivecolor is not None:
                            self.master.create_oval(xpos,ypos,xpos+crad,ypos+crad,fill=alivecolor,outline=Cell.BORDER_COLORS[GuiState.INITLOCS])
                        if deadcolor is not None:
                            self.master.create_text((xmax-crad,ymax-crad),text='X',fill=deadcolor)
                        if alivecolor is not None:
                            if self.goal is not None:
                                self.goal.visited = True
                        else:
                            if self.goal is not None:
                                self.goal.visited=False 
                        #hmmm?
                        #for all the not dead agents get a new color 
                    
                else:
                    if self.defaultColor is None:
                        self.master.create_oval(xpos,ypos,xpos+crad,ypos+crad,fill=Cell.FILL_COLORS[GuiState.INITLOCS],outline=Cell.BORDER_COLORS[GuiState.INITLOCS])
                    else:
                        self.master.create_oval(xpos,ypos,xpos+crad,ypos+crad,fill=self.defaultColor,outline=Cell.BORDER_COLORS[GuiState.INITLOCS])
            if self.isDoor:
                d = self.getXY()
                if d[0] == 1:
                    self.master.create_line(xmax,ymin+self.size/4,xmax-self.size/4,ymax-self.size/3)
                    self.master.create_line(xmax-self.size/4,ymax-self.size/3,xmax,ymax-self.size/4)

            self.master.create_text((xc,yc),text=self.label)


    def getXY(self):
        return (self.abs,self.ord)
    
    def updateInitPosColor(self,color):
        #just going to add two colors
        if self.defaultColor is None:
            self.defaultColor = color
        else:
            self.defaultColor = self.mixColor(color,self.defaultColor)
            #print (self.defaultColor)
            #print (color)
            #rgbcolor = mc.to_rgb(color)
            #rgbdefault = (mc.to_rgb(self.defaultColor))
            
            #mixedcolor = ((rgbdefault[0]+rgbcolor[0])/2,(rgbdefault[1]+rgbcolor[1])/2,(rgbdefault[2]+rgbcolor[2])/2)
            #self.defaultColor = tuple([255*x for x in mixedcolor])
            #self.defaultColor = '#%02x%02x%02x' % self.defaultColor
            #print (self.defaultColor)

    def mixColor(self,color1,color2):
        if color1 is None:
            return color2
        if color2 is None:
            return color1
        
        rgbcolor = mc.to_rgb(color1)
        rgbcolor2 = mc.to_rgb(color2)
        mixedcolor = ((rgbcolor2[0]+rgbcolor[0])/2,(rgbcolor2[1]+rgbcolor[1])/2,(rgbcolor2[2]+rgbcolor[2])/2)
        mixedcolorstr = tuple([255*x for x in mixedcolor])
        mixedcolorstr = '#%02x%02x%02x' % mixedcolorstr
        return mixedcolorstr
    
        
    #from https://stackoverflow.com/questions/37765197/darken-or-lighten-a-color-in-matplotlib
    def lighten_color(self,color, amount=0.5):
        """
        Lightens the given color by multiplying (1-luminosity) by the given amount.1
        Input can be matplotlib color string, hex string, or RGB tuple.

        Examples:
        >> lighten_color('g', 0.3)
        >> lighten_color('#F034A3', 0.6)
        >> lighten_color((.3,.55,.1), 0.5)
        """


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
