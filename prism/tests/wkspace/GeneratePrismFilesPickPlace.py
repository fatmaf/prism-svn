import math 
from PrismFileReader import PrismFileReader
from PrismModule import PrismModule
import random
import os
import copy

class GeneratePrismFilePickPlace(object):
    
    failstateVar = 'failstate'
    succProbVar = 'p'
    succProb = 0.8
    pickedVar = 'picked'
    placedVar = 'placed'
    notPPVar = 'notPP'
    hasItemVar = 'hasItemNow'
    hasNoItemVar = 'hasNoItem' 

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

    def stateStringS(self,varname,varvalue,sign):
        return '('+varname+sign+str(varvalue)+')'
    def stateLabel(self,varname,varvalue):
        return varname+str(varvalue)

    def addFailstateConstant(self,pfr):
        if self.failstateVar not in pfr.constants:
            pfr.addConstant(self.failstateVar,'int','-1')
        return pfr
    

    def addPickPlaceConstants(self,pfr):
        if self.notPPVar not in pfr.constants:
            pfr.addConstant(self.notPPVar,'int','0')
        if self.pickedVar not in pfr.constants:
            pfr.addConstant(self.pickedVar,'int','1')
        if self.placedVar not in pfr.constants:
            pfr.addConstant(self.placedVar,'int','2')
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
    
    def createActionsFromLists(self,pfr,mod,xside,yside,varname,initStates,blockedStates,failstates,connectedStates,doFour,ppVars,itemVars):
        #print "Generating Actions"
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
            itemstr = " "
            for ppv in ppVars:
                itemstr = itemstr+ "& " + self.stateStringS(ppv,self.pickedVar,'!=')                
            pfr = self.addLabelToPFR(svar,sstr,pfr,mod)
            xx = min(x+1,xside-1)
            yy = min(y+1,yside-1)
            _x = max(x-1,0)
            _y = max(y-1,0)
            if doFour:
                combs = [(x,yy),(x,_y),(xx,y),(_x,y)]
            else:
                combs = [(x,yy),(x,_y),(xx,y),(_x,y),(_x,yy),(xx,yy),(_x,_y),(xx,_y)]
            combs = list(set(combs))
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
                isFailState = xyp in failstates
                rew = 0 
                rew = math.sqrt(((xp-x)*(xp-x))+((yp-y)*(yp-y)))
                #print rew
                #print xyp
                #print xy
                rewDict = {'"time"':rew}
                #print rewDict 
                if isFailState:
                    pa = mod.createAction(svar+'_'+pvar,copy.deepcopy(rewDict),sstr+itemstr,[pstr,'('+varname+'=failstate)'],['p','1-p'])
                    mod.actions.append(copy.deepcopy(pa))

                    for itemnum in range(len(itemVars)):
                        itemvar = itemVars[itemnum]
                        ppv = ppVars[itemnum]
                        itemMoveStr = " & "+ self.stateString(itemvar,smap[xy])
                        itemPickedStr = " & "+ self.stateString(ppv,self.pickedVar)
                        itemMoveStr = itemMoveStr+itemPickedStr
                        itemMovedStr = "& "+self.stateString(itemvar,smap[xyp])
                        pa = mod.createAction(svar+'_'+pvar,copy.deepcopy(rewDict),sstr+itemMoveStr,[pstr+itemMovedStr,'('+varname+'=failstate)'],['p','1-p'])
                        #raw_input ("Generate Action")
                        mod.actions.append(copy.deepcopy(pa))
                else:
                    pa = mod.createAction(svar+'_'+pvar,copy.deepcopy(rewDict),sstr+itemstr,pstr,1.0)
                    mod.actions.append(copy.deepcopy(pa))
                    #print pa
                    #lets do this
                    
                    for itemnum in range(len(itemVars)):
                        itemvar = itemVars[itemnum]
                        ppv = ppVars[itemnum]
                        itemMoveStr = " & "+ self.stateString(itemvar,smap[xy])
                        itemPickedStr = " & "+ self.stateString(ppv,self.pickedVar)
                        itemMoveStr = itemMoveStr+itemPickedStr
                        itemMovedStr = "& "+self.stateString(itemvar,smap[xyp])
                        pa = mod.createAction(svar+'_'+pvar,copy.deepcopy(rewDict),sstr+itemMoveStr,[pstr+itemMovedStr],[1.0])
                        #raw_input ("Generated Action")
                        mod.actions.append(copy.deepcopy(pa))

                
        if len(failstates)>0:
            pa = mod.createAction('failed',{'"time"':1.0},'('+varname+'=failstate)','('+varname+'=failstate)','1.0')
            mod.actions.append(copy.deepcopy(pa))

        #adding the pick and place actions
        if len(ppVars)>0:
            for i in range(len(ppVars)):
                actionLabel = 'pick'+str(i)
                itemvar = itemVars[i]
                ppv = ppVars[i]
                pickStr = self.stateString(varname,itemvar) + " & " + self.stateString(ppv,self.notPPVar)
                for j in range(len(ppVars)):
                    if i!=j:
                        ppvj = ppVars[j]
                        pickStr = pickStr + "& " + '('+ppvj+'!='+self.pickedVar+')'
                pickedStr = self.stateString(ppv,self.pickedVar)
                pa = mod.createAction(actionLabel,{'"time"':1.0},pickStr,pickedStr,1.0)
                mod.actions.append(copy.deepcopy(pa))
                actionLabel = 'place'+str(i)
                placeStr = self.stateString(varname,itemvar) + " & " +pickedStr
                placedStr = self.stateString(ppv,self.placedVar)
                pa = mod.createAction(actionLabel,{'"time"':1.0},placeStr,placedStr,1.0)
                mod.actions.append(copy.deepcopy(pa))
                
            
        #for pa in mod.actions:
        #    #print pa.prismStringReward('"time"')
            
        return [mod,pfr,smap]

    def createActionsFromListsHasItem(self,pfr,mod,xside,yside,varname,initStates,blockedStates,failstates,connectedStates,doFour,itemVars,hasitemvar):
        #print "Generating Actions"
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
            itemstr = " "

            itemstr = itemstr+ "& " + self.stateString(hasitemvar,self.hasNoItemVar)                
            pfr = self.addLabelToPFR(svar,sstr,pfr,mod)
            xx = min(x+1,xside-1)
            yy = min(y+1,yside-1)
            _x = max(x-1,0)
            _y = max(y-1,0)
            if doFour:
                combs = [(x,yy),(x,_y),(xx,y),(_x,y)]
            else:
                combs = [(x,yy),(x,_y),(xx,y),(_x,y),(_x,yy),(xx,yy),(_x,_y),(xx,_y)]
            combs = list(set(combs))
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
                isFailState = xyp in failstates
                rew = 0 
                rew = math.sqrt(((xp-x)*(xp-x))+((yp-y)*(yp-y)))
                #print rew
                #print xyp
                #print xy
                rewDict = {'"time"':rew}
                #print rewDict 
                if isFailState:
                    pa = mod.createAction(svar+'_'+pvar,copy.deepcopy(rewDict),sstr+itemstr,[pstr,'('+varname+'=failstate)'],['p','1-p'])
                    mod.actions.append(copy.deepcopy(pa))

                    for itemnum in range(len(itemVars)):
                        itemvar = itemVars[itemnum]
                        itemMoveStr = " & "+ self.stateString(itemvar,smap[xy])
                        itemPickedStr = " & "+ self.stateString(hasitemvar,itemnum)
                        itemMoveStr = itemMoveStr+itemPickedStr
                        itemMovedStr = "& "+self.stateString(itemvar,smap[xyp])
                        pa = mod.createAction(svar+'_'+pvar,copy.deepcopy(rewDict),sstr+itemMoveStr,[pstr+itemMovedStr,'('+varname+'=failstate)'],['p','1-p'])
                        #raw_input ("Generate Action")
                        mod.actions.append(copy.deepcopy(pa))
                else:
                    pa = mod.createAction(svar+'_'+pvar,copy.deepcopy(rewDict),sstr+itemstr,pstr,1.0)
                    mod.actions.append(copy.deepcopy(pa))
                    #print pa
                    #lets do this
                    
                    for itemnum in range(len(itemVars)):
                        itemvar = itemVars[itemnum]
                        itemMoveStr = " & "+ self.stateString(itemvar,smap[xy])
                        itemPickedStr = " & "+ self.stateString(hasitemvar,itemnum)
                        itemMoveStr = itemMoveStr+itemPickedStr
                        itemMovedStr = "& "+self.stateString(itemvar,smap[xyp])
                        pa = mod.createAction(svar+'_'+pvar,copy.deepcopy(rewDict),sstr+itemMoveStr,[pstr+itemMovedStr],[1.0])
                        #raw_input ("Generated Action")
                        mod.actions.append(copy.deepcopy(pa))

                
        if len(failstates)>0:
            pa = mod.createAction('failed',{'"time"':1.0},'('+varname+'=failstate)','('+varname+'=failstate)','1.0')
            mod.actions.append(copy.deepcopy(pa))

        #adding the pick and place actions
        if len(itemVars)>0:
            for i in range(len(itemVars)):
                #picking 
                actionLabel = 'pick'+str(i)
                itemvar = itemVars[i]
                pickStr = self.stateString(varname,itemvar) +  " & " + self.stateString(hasitemvar,self.hasNoItemVar)
                pickedStr = self.stateString(hasitemvar,i)
                pa = mod.createAction(actionLabel,{'"time"':1.0},pickStr,pickedStr,1.0)
                mod.actions.append(copy.deepcopy(pa))
                #placing item 
                actionLabel = 'place'+str(i)
                placeStr = self.stateString(varname,itemvar) + " & "  + self.stateString(hasitemvar,i)
                placedStr = self.stateString(hasitemvar,self.hasNoItemVar)
                pa = mod.createAction(actionLabel,{'"time"':1.0},placeStr,placedStr,1.0)
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
    
    def generateModuleLinesFromLists(self,initStates,blockedStates,goalStates,avoidStates,failstates,connectedStates,doorStates,xside,yside,pfr,modName,doFour,numitems):
        pfr = self.addFailstateConstant(pfr)
        pfr = self.addSuccProbConstant(pfr)
        pfr = self.checkModInPFR(pfr,modName)
        pfr = self.addPickPlaceConstants(pfr)
        gmod = PrismModule(modName,[], pfr.constants)
        varname = 's'
        rewName = '"time"'
        #add variable later cuz we dont know the limits
        varlim = len(connectedStates)
        gmod.addVariable(varname,pfr.constants[self.failstateVar],varlim,0,0)
        notPPVars = []
        itemVars = []
        for i in range(numitems):
            
            #adding the stuff for item
            itemvarname = "item"+str(i)
            itemVars.append(itemvarname)
            gmod.addVariable(itemvarname,0,varlim,0,0)
            ppvarname = "pickplace"+str(i)
            notPPVars.append(ppvarname)
            gmod.addVariable(ppvarname,pfr.constants[self.notPPVar],pfr.constants[self.placedVar],pfr.constants[self.notPPVar],pfr.constants[self.notPPVar])
        
        if pfr.rewardNames is None:
            pfr.rewardNames = []
        if rewName not in pfr.rewardNames:
            pfr.rewardNames.append(rewName)
        if pfr.modVars is None:
            pfr.modVars = []
        [gmod,pfr,smap] = self.createActionsFromLists(pfr,gmod,xside,yside,varname,initStates,blockedStates,failstates,connectedStates,doFour,notPPVars,itemVars)
        #for pa in gmod.actions:
        #    print pa.prismStringReward(rewName)
        pfr.modVars.append(gmod)
        hasDoors = False 
        updatedDoorStates = [] 
        for ds in doorStates:
            updatedDoorStates.append(smap[ds])
            hasDoors = True
        if hasDoors:
            pfr = self.createDoorsFromLists(pfr,updatedDoorStates,varname,modName)
        return [pfr,smap,varname]

    def generateModuleLinesFromListsHasItem(self,initStates,blockedStates,goalStates,avoidStates,failstates,connectedStates,doorStates,xside,yside,pfr,modName,doFour,numitems):
        pfr = self.addFailstateConstant(pfr)
        pfr = self.addSuccProbConstant(pfr)
        pfr = self.checkModInPFR(pfr,modName)
        if self.hasNoItemVar not in pfr.constants:
            pfr.addConstant(self.hasNoItemVar,'int','-1')
        
        gmod = PrismModule(modName,[], pfr.constants)
        varname = 's'
        rewName = '"time"'
        #add variable later cuz we dont know the limits
        varlim = len(connectedStates)-1
        gmod.addVariable(varname,pfr.constants[self.failstateVar],varlim,0,0)
        notPPVars = []
        itemVars = []
        for i in range(numitems):          
            #adding the stuff for item
            itemvarname = "item"+str(i)
            itemVars.append(itemvarname)
            gmod.addVariable(itemvarname,0,varlim,0,0)
            
        hasitemvarname = 'hasItem'
        gmod.addVariable(hasitemvarname,pfr.constants[self.hasNoItemVar],numitems,pfr.constants[self.hasNoItemVar],pfr.constants[self.hasNoItemVar])
        
        if pfr.rewardNames is None:
            pfr.rewardNames = []
        if rewName not in pfr.rewardNames:
            pfr.rewardNames.append(rewName)
        if pfr.modVars is None:
            pfr.modVars = []
        [gmod,pfr,smap] = self.createActionsFromListsHasItem(pfr,gmod,xside,yside,varname,initStates,blockedStates,failstates,connectedStates,doFour,itemVars,hasitemvarname)
        #for pa in gmod.actions:
        #    print pa.prismStringReward(rewName)
        pfr.modVars.append(gmod)
        hasDoors = False 
        updatedDoorStates = [] 
        for ds in doorStates:
            updatedDoorStates.append(smap[ds])
            hasDoors = True
        if hasDoors:
            pfr = self.createDoorsFromLists(pfr,updatedDoorStates,varname,modName)
        return [pfr,smap,varname]


    def generateFromGUIGrid(self,initStates,blockedStates,goalStates,avoidStates,failStates,connectedStates,doorStates,xside,yside,fn,doFour,numItems):
        pfr = PrismFileReader(None)
        modName = 'grid'
        [pfr,smap,varname] = self.generateModuleLinesFromListsHasItem(initStates,blockedStates,goalStates,avoidStates,failStates,connectedStates,doorStates,xside,yside,pfr,modName,doFour,numItems)
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
        
        print smap
        return smap 
        
   



if __name__ == "__main__":
    gfr = GeneratePrismFile()
    gfr.testFileGen()
    
            
            

