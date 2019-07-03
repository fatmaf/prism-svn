
# coding: utf-8

# In[13]:


from PrismFileReader import PrismFileReader
from PrismModule import PrismModule
import random
import os

class GeneratePrismFile(object):
    
    failstateVar = 'failstate'
    succProbVar = 'p'
    succProb = 0.8

    def diffLists(self,l1,l2):
        l1s = set(l1)
        l2s = set(l2)
        res = l1s.symmetric_difference(l2s)
        return list(res)

    def sample(self,l,n):
        return random.sample(l,n)

    def pickStates(self,previousStatesList,numToPick,currentStatesList):
        statesPicked = []
        n = numToPick
        if previousStatesList is not None:
            statesPicked = previousStatesList[0:n]
            n = n - len(statesPicked)
        if n> 0:
            statesToPickFrom = diffLists(statesPicked,currentStatesList)
            newStatesPicked = sample(statesToPickFrom,n)
            statesPicked = statesPicked + newStatesPicked
        return statesPicked

    def ensureDir(self,dirLoc):
        if not os.path.exists(dirLoc):
            os.makedirs(dirLoc)

    def getTermWidth(self):
        rows, columns = os.popen('stty size', 'r').read().split()
        width = int(columns)
        return width

    def printC(self,string,width):
        #width = os.get_terminal_size().columns


        print(string.center(width))


    def linInd(self,x,y,xside,yside):
        return y*yside + x

    def stateString(self,varname,varvalue):
        return '('+varname+'='+str(varvalue)+')'

    def stateLabel(self,varname,varvalue):
        return varname+str(varvalue)

    def addFailstateConstant(self,pfr):
        if self.failstateVar not in pfr.constants:
            pfr.addConstant(self.failstateVar,'int','-1')
        return pfr

    def addSuccProbConstant(self,pfr):
        if self.succProbVar not in pfr.constants:
            pfr.addConstant(self.succProbVar,'double',str(self.succProb))
        return pfr

    def checkModInPFR(self,pfr,modName):
        addName = True
        if pfr.moduleNames is not None:
            if len(pfr.moduleNames) > 0:
                if modName in pfr.moduleNames:
                    addName = False
        else:
            pfr.moduleNames=[]
        if addName:
            pfr.moduleNames.append(modName)
        return pfr    



    # In[14]:


    def createWarehouseActions(self,gpfr,gmod,xside,yside,varname):
        scount = 0 
        for x in range(xside):
            for y in range(yside):
                scount = self.linInd(x,y,xside,yside)
                svar = self.stateLabel(varname,scount)
                sstr = self.stateString(varname,scount)
                if gpfr.labels is None:
                    gpfr.addLabel(svar,sstr,gmod.variables)
                if not svar in gpfr.labels:
                    gpfr.addLabel(svar,sstr,gmod.variables)
                xx = min(x+1,xside-1)
                yy = min(y+1,yside-1)
                _x = max(x-1,0)
                _y = max(y-1,0)
                if y == 0 or y == (yside-1) or x==0 or x==(xside-1):
                    combs = [(x,yy),(x,_y),(xx,y),(_x,y)]
                else:
                    combs = [(xx,y),(_x,y)]
                #print (x,y)
                #print combs
                #raw_input()
                for xy in combs:
                    xp = xy[0]
                    yp = xy[1]
                    if(xp == x and yp == y):
                        continue
                    pcount = self.linInd(xp,yp,xside,yside)
                    pstr = self.stateString(varname,pcount)
                    pvar = self.stateLabel(varname,pcount)
                    if not pvar in gpfr.labels:
                        gpfr.addLabel(pvar,pstr,gmod.variables)
                    #pa = mod.createAction('s1_s2',{'"time"':1.0},'(s=0)',['(s=1)','(s=2)'],['p','1-p'])
                    #pa = gmod.createAction(svar+'_'+pvar,{'"time"':1.0},sstr,[pstr,'(s=failstate)'],['p','1-p'])
                    pa = gmod.createAction(svar+'_'+pvar,{'"time"':1.0},sstr,pstr,'1.0')
                    #print pa
                    gmod.actions.append(pa)
        return [gmod,gpfr]


    # In[15]:


    def addWarehouseLinesModule(self,gridPfr,modName,xside,yside,numFailStates):
        #check if we have p and failstate 
        gridPfr = self.addFailstateConstant(gridPfr)
        gridPfr = self.addSuccProbConstant(gridPfr)
        gridPfr = self.checkModInPFR(gridPfr,modName)
        gmod = PrismModule(modName,[],gridPfr.constants)
        varname = 's'
        rewName = '"time"'
        gmod.addVariable(varname,gridPfr.constants[self.failstateVar],xside*yside,0,0)
        if gridPfr.rewardNames is None:
            gridPfr.rewardNames = []
        if rewName not in gridPfr.rewardNames:
            gridPfr.rewardNames.append(rewName)
        if gridPfr.modVars is None:
            gridPfr.modVars = []
        [gmod,gridPfr] = self.createWarehouseActions(gridPfr,gmod,xside,yside,varname)
        gridPfr.modVars.append(gmod)
        return gridPfr


    # In[16]:


    def addToSmap(self,xy,smap):
        if xy not in smap:
            scount = len(smap)
            smap[xy] = scount
            #print smap
        
        return smap
    
    def addLabelToPFR(self,svar,sstr,pfr,mod):
        if pfr.labels is None:
            pfr.addLabel(svar,sstr,mod.variables)
        if not svar in pfr.labels:
            pfr.addLabel(svar,sstr,mod.variables)
        return pfr
    
    def createActionsFromLists(self,pfr,mod,xside,yside,varname,initStates,blockedStates,failstates,connectedStates):
        smap = {}
        scount = len(smap)
        for xy in connectedStates:
            x = xy[0]
            y = xy[1]
            smap = self.addToSmap(xy,smap)
            svar = self.stateLabel(varname,smap[xy])
            sstr = self.stateString(varname,smap[xy])
            pfr = self.addLabelToPFR(svar,sstr,pfr,mod)
            xx = min(x+1,xside-1)
            yy = min(y+1,yside-1)
            _x = max(x-1,0)
            _y = max(y-1,0)
            combs = [(x,yy),(x,_y),(xx,y),(_x,y)]
            for xyp in combs:
                xp = xyp[0]
                yp = xyp[1]
                if (xp == x and yp == y):
                    continue
                if xyp in blockedStates:
                    continue
                smap = self.addToSmap(xyp,smap)
                pvar = self.stateLabel(varname,smap[xyp])
                pstr = self.stateString(varname,smap[xyp])
                pfr = self.addLabelToPFR(pvar,pstr,pfr,mod)
                #if xyp is in failstates we need to make it a failstate action
                isFailState = xyp in failstates
                if isFailState:
                    pa = mod.createAction(svar+'_'+pvar,{'"time"':1.0},sstr,[pstr,'(s=failstate)'],['p','1-p'])
                else:
                    pa = mod.createAction(svar+'_'+pvar,{'"time"':1.0},sstr,pstr,1.0)
                mod.actions.append(pa)
        return [mod,pfr,smap]
    
                    
    def createGridActions(self,gpfr,gmod,xside,yside,varname):
        scount = 0 
        for x in range(xside):
            for y in range(yside):
                scount = self.linInd(x,y,xside,yside)
                svar = self.stateLabel(varname,scount)
                sstr = self.stateString(varname,scount)
                if gpfr.labels is None:
                    gpfr.addLabel(svar,sstr,gmod.variables)
                if not svar in gpfr.labels:
                    gpfr.addLabel(svar,sstr,gmod.variables)
                xx = min(x+1,xside-1)
                yy = min(y+1,yside-1)
                _x = max(x-1,0)
                _y = max(y-1,0)
                combs = [(x,yy),(x,_y),(xx,y),(_x,y)]

                for xy in combs:
                    xp = xy[0]
                    yp = xy[1]
                    if(xp == x and yp == y):
                        continue
                    pcount = self.linInd(xp,yp,xside,yside)
                    pstr = self.stateString(varname,pcount)
                    pvar = self.stateLabel(varname,pcount)
                    if not pvar in gpfr.labels:
                        gpfr.addLabel(pvar,pstr,gmod.variables)
                    #pa = mod.createAction('s1_s2',{'"time"':1.0},'(s=0)',['(s=1)','(s=2)'],['p','1-p'])
                    #pa = gmod.createAction(svar+'_'+pvar,{'"time"':1.0},sstr,[pstr,'(s=failstate)'],['p','1-p'])
                    pa = gmod.createAction(svar+'_'+pvar,{'"time"':1.0},sstr,pstr,'1.0')
                    #print pa
                    gmod.actions.append(pa)
        return [gmod,gpfr]
    
    def generateModuleLinesFromLists(self,initStates,blockedStates,goalStates,avoidStates,failstates,connectedStates,xside,yside,pfr,modName):
        pfr = self.addFailstateConstant(pfr)
        pfr = self.addSuccProbConstant(pfr)
        pfr = self.checkModInPFR(pfr,modName)
        gmod = PrismModule(modName,[], pfr.constants)
        varname = 's'
        rewName = '"time"'
        #add variable later cuz we dont know the limits
        varlim = len(connectedStates)
        gmod.addVariable(varname,pfr.constants[self.failstateVar],varlim,0,0)
        if pfr.rewardNames is None:
            pfr.rewardNames = []
        if rewName not in pfr.rewardNames:
            pfr.rewardNames.append(rewName)
        if pfr.modVars is None:
            pfr.modVars = []
        [gmod,pfr,smap] = self.createActionsFromLists(pfr,gmod,xside,yside,varname,initStates,blockedStates,failstates,connectedStates)
        pfr.modVars.append(gmod)
        return [pfr,smap,varname]
    


    def addGridLinesModule(self,gridPfr,modName,xside,yside,numFailStates):
        #check if we have p and failstate 
        gridPfr = self.addFailstateConstant(gridPfr)
        gridPfr = self.addSuccProbConstant(gridPfr)
        gridPfr = self.checkModInPFR(gridPfr,modName)
        gmod = PrismModule(modName,[],gridPfr.constants)
        varname = 's'
        rewName = '"time"'
        gmod.addVariable(varname,gridPfr.constants[self.failstateVar],xside*yside,0,0)
        if gridPfr.rewardNames is None:
            gridPfr.rewardNames = []
        if rewName not in gridPfr.rewardNames:
            gridPfr.rewardNames.append(rewName)
        if gridPfr.modVars is None:
            gridPfr.modVars = []
        [gmod,gridPfr] = self.createGridActions(gridPfr,gmod,xside,yside,varname)
        gridPfr.modVars.append(gmod)
        return gridPfr


    # In[19]:

    def testFileGen(self):
        gridPfr = PrismFileReader(None)
        modName = 'grid'
        xside = 4  
        yside = 4 
        numFailStates = 0
        gridPfr = self.addWarehouseLinesModule(gridPfr,modName,xside,yside,numFailStates)


        # In[20]:


        lines = gridPfr.createFileLinesToWrite()
        gridPfr.writeLinesToFile(lines,'warehouse.prism')


        # In[ ]:

    def generateFromGUIGrid(self,initStates,blockedStates,goalStates,avoidStates,failStates,connectedStates,xside,yside):
        pfr = PrismFileReader(None)
        modName = 'grid'
        [pfr,smap,varname] = self.generateModuleLinesFromLists(initStates,blockedStates,goalStates,avoidStates,failStates,connectedStates,xside,yside,pfr,modName)
        i = 0 
        for initState in initStates:
            sv = smap[initState]
            pfr.changeVariableInitValue(modName,varname,sv)
            lines = pfr.createFileLinesToWrite()
            pfr.writeLinesToFile(lines,'gui'+str(i)+'.prism')
            i = i + 1
        newGoalStates = []
        for g in goalStates:
            newGoalStates.append(smap[g])
        newAvoidStates = []
        for a in avoidStates:
            newAvoidStates.append(smap[a])
        pfr.writeGoalStatesAvoid(newGoalStates,newAvoidStates,varname,'gui.props')
        pfr.writeGoalStatesSplitAvoid(newGoalStates,newAvoidStates,varname,'gui.prop')
   



if __name__ == "__main__":
    gfr = GeneratePrismFile()
    gfr.testFileGen()
    
            
            

