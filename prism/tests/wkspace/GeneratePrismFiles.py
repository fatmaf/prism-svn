
# coding: utf-8

# In[13]:

import math 
from PrismFileReader import PrismFileReader
from PrismModule import PrismModule
import random
import os
import copy

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
                    rew = math.sqrt(((xp-x)*(xp-x))+((yp-y)*(yp-y)))
                    pa = gmod.createAction(svar+'_'+pvar,{'"time"':rew},sstr,pstr,'1.0')
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
    
    def createActionsFromLists(self,pfr,mod,xside,yside,varname,initStates,blockedStates,failstates,connectedStates,doFour):
        smap = {}
        scount = len(smap)
        #doFour = True
        #default = 8
        for xy in connectedStates:
            x = xy[0]
            y = xy[1]
            smap = self.addToSmap(xy,smap)
            svar = self.stateLabel(varname,smap[xy])
            sstr = self.stateString(varname,smap[xy])
            pfr = self.addLabelToPFR(svar,sstr,pfr,mod)
            #print "connecting " 
            #print xy
            #print smap
            #print pfr.labels
            #print " to "
            #raw_input()
            xx = min(x+1,xside-1)
            yy = min(y+1,yside-1)
            _x = max(x-1,0)
            _y = max(y-1,0)
            if doFour:
                combs = [(x,yy),(x,_y),(xx,y),(_x,y)]
            else:
                combs = [(x,yy),(x,_y),(xx,y),(_x,y),(_x,yy),(xx,yy),(_x,_y),(xx,_y)]
            combs = list(set(combs))
            usedCombs = [] 
            for xyp in combs:
                xp = xyp[0]
                yp = xyp[1]
                if (xp == x and yp == y):
                    continue
                if xyp in blockedStates:
                    continue
                usedCombs.append(xyp)
                smap = self.addToSmap(xyp,smap)
                pvar = self.stateLabel(varname,smap[xyp])
                pstr = self.stateString(varname,smap[xyp])
                pfr = self.addLabelToPFR(pvar,pstr,pfr,mod)
                #print xyp
                #print smap
                #print pfr.labels
                #if xyp is in failstates we need to make it a failstate action
                isFailState = xyp in failstates
                rew = 0 
                rew = math.sqrt(((xp-x)*(xp-x))+((yp-y)*(yp-y)))
                #print rew
                #print xyp
                #print xy
                rewDict = {'"time"':rew}
                #print rewDict 
                if isFailState:
                    pa = mod.createAction(svar+'_'+pvar,copy.deepcopy(rewDict),sstr,[pstr,'('+varname+'=failstate)'],['p','1-p'])
                else:
                    pa = mod.createAction(svar+'_'+pvar,copy.deepcopy(rewDict),sstr,pstr,1.0)
                #if rew > 1.0:
                #    print "more than one"
                #    print pa
                #else:
                #    print "less than or equal to one"
                #    print pa
                    
                #print pa
                #print pa.prismStringReward('"time"')
                
                mod.actions.append(copy.deepcopy(pa))
            doThis = False 
            if doThis and len(usedCombs) != len(combs):
                print "Generated Combs for " + str(x)+","+str(y)
                print combs
                print "Used combs"
                print usedCombs
                #checking if different things have the same values
                for xc in usedCombs:
                    print xc
                    print smap[xc]
                raw_input("continue")
        if len(failstates)>0:
            pa = mod.createAction('failed',{'"time"':1.0},'('+varname+'=failstate)','('+varname+'=failstate)','1.0')
            mod.actions.append(copy.deepcopy(pa))
            
        #for pa in mod.actions:
        #    #print pa.prismStringReward('"time"')
            
        return [mod,pfr,smap]

    def createDoorsFromLists(self,pfr,doorStates,variable,moduleName):
        numDoors = len(doorStates)/2
        if 'unknown' not in pfr.constants:
            pfr.addConstant('unknown','int','-1')
        if 'open' not in pfr.constants:
            pfr.addConstant('open','int','1')
        if 'closed' not in pfr.constants:
            pfr.addConstant('closed','int','0')
            
        for i in range(0,numDoors):
        #add door variables
    
            doorVarName = 'door'+str(i)
            #print doorVarName
            pfr.addVariable(moduleName,doorVarName,pfr.constants['unknown'],pfr.constants['open'],pfr.constants['unknown'],pfr.constants['unknown'])

        numDoorVar = 0
        for i in range(0,len(doorStates),2):
            #print "counted doors"
            #print numDoorVar
            #print "total doors"
            #print numDoors
            if(numDoorVar == numDoors):
                break
            doorVarName = 'door'+str(numDoorVar)
            s1 = doorStates[i]
            s2 = doorStates[i+1]
            #print doorVarName
            #print s1
            #print s2

            #raw_input()
            s1s = self.stateString(variable,s1)
            s2s = self.stateString(variable,s2)
            pfr.addDoor(moduleName,doorVarName,s1s,s2s,'p','1-p')
            numDoorVar = numDoorVar+1

        return pfr
    
                    
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
    
    def generateModuleLinesFromLists(self,initStates,blockedStates,goalStates,avoidStates,failstates,connectedStates,doorStates,xside,yside,pfr,modName,doFour):
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
        [gmod,pfr,smap] = self.createActionsFromLists(pfr,gmod,xside,yside,varname,initStates,blockedStates,failstates,connectedStates,doFour)
        #for pa in gmod.actions:
        #    print pa.prismStringReward(rewName)
        pfr.modVars.append(gmod)
        updatedDoorStates = [] 
        for ds in doorStates:
            updatedDoorStates.append(smap[ds])
            
        pfr = self.createDoorsFromLists(pfr,updatedDoorStates,varname,modName)
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

    def generateFromGUIGrid(self,initStates,blockedStates,goalStates,avoidStates,failStates,connectedStates,doorStates,xside,yside,fn,doFour):
        pfr = PrismFileReader(None)
        modName = 'grid'
        [pfr,smap,varname] = self.generateModuleLinesFromLists(initStates,blockedStates,goalStates,avoidStates,failStates,connectedStates,doorStates,xside,yside,pfr,modName,doFour)
        #print smap
        #print varname
        #print pfr
        #print pfr.modVars[0].actions
        i = 0 
        for initState in initStates:
            sv = smap[initState]
            pfr.changeVariableInitValue(modName,varname,sv)
            lines = pfr.createFileLinesToWrite()
            nf = fn+str(i)+'.prism'
            print nf 
            pfr.writeLinesToFile(lines,fn+str(i)+'.prism')
            i = i + 1
        newGoalStates = []
        newGoalStatesLabels = []
        for g in goalStates:
            newGoalStates.append(smap[g])
            label = self.stateLabel(varname,smap[g])
            newGoalStatesLabels.append(label)
            #print label 
            #print g
            #print smap[g]
            
        newAvoidStates = []
        for a in avoidStates:
            newAvoidStates.append(smap[a])
        pfr.writeGoalStatesAvoid(newGoalStates,newAvoidStates,varname,fn+'.props')
        pfr.writeGoalStatesAvoidReward(newGoalStates,newAvoidStates,varname,fn+'_rew.props',pfr.rewardNames[0])
        pfr.writeGoalStatesSplitAvoidLabels(newGoalStates,newAvoidStates,varname,fn+'.prop')
        
        #print smap
        return smap 
        
   



if __name__ == "__main__":
    gfr = GeneratePrismFile()
    gfr.testFileGen()
    
            
            

