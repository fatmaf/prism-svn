from PrismVariable import PrismVariable
from PrismVariable import VariableType
from RegexHelper import RegexHelper
from PrismAction import PrismAction

class PrismModule(object):

            
            
    moduleVarsRE = "(.*):\[(.*)\.\.(.*)] init( ?[-]?[\d+]?[.]?[\d+]);"
    moduleVarsREConstants = "(.*):\[(.*)\.\.(.*)] init (.*);"
    basicActionRE = "\[(.*)\] (.*) -> (.*);"
    name = None
    variables = {}
    moduleLines = None
    contants = None
    actions = None
    rewardsRE = "\[(.*)\] (.*):(.*);"
    rewardNames = []
    
    def __init__(self,n,moduleLines,constants):
        print "Initializing Module "+n
        
        self.name = n
        self.moduleLines = moduleLines
        self.constants = constants
        
        #process variables
        self.processVariables()
        #print "Variables"
        #print self.variables

        self.processActions()

    def cleanModule(self):
        self.name = None
        self.variables = {}
        self.constants = None
        self.actions = None
        self.rewardNames = []
        
    def processActions(self):
        self.actions = []
        for i in range(len(self.moduleLines)):
            #what tells me something is an action ?
            line = self.moduleLines[i]
            if RegexHelper.isRegexMatch(self.basicActionRE,line):
                pa = PrismAction(line,self.constants,self.variables)
                self.actions.append(pa)

    def getActionSrcDestValue(self,varname,a):
        statePairs = []
        
        statePairs.append(int(a.getVarValueSrc(varname)))
        statePairs.append(int(a.getVarValueDest(varname)))
        return statePairs
    
    def findActionsToSkip(self,actionsPickedTemp,varname,prevStatePairs):
        statePairs = []
        for act in actionsPickedTemp:
            statePairs.append(self.getActionSrcDestValue(varname,act))
        toSkip = []
        actionsSkipped = []
        #first check against previous state pairs
        for i in range(len(prevStatePairs)):
            a1 = prevStatePairs[i]
            for j in range(len(statePairs)):
                a2 = statePairs[j]
                if (a1[0] == a2[0] or a1[1] == a2[0] or a1[0] == a2[1] or a1[1] == a2[1]):
                    if not j in toSkip:
                        toSkip.append(j)
                        actionsSkipped.append(actionsPickedTemp[j])
                        
        for i in range(len(statePairs)):
            if i not in toSkip:
                a1 = statePairs[i]
                for j in range(i+1,len(statePairs)):
                    if j not in toSkip:
                        a2 = statePairs[j]
                        if (a1[0] == a2[0] or a1[1] == a2[0] or a1[0] == a2[1] or a1[1] == a2[1]):
                            #if not j in toSkip:
                            toSkip.append(j)
                            actionsSkipped.append(actionsPickedTemp[j])
        #actionsSkipped = actionsPickedTemp[toSkip]
        #actionsPickedTemp = list(set(actionsPickedTemp).symmetric_difference(set(actionsSkipped)))
        #if len(toSkip) != 0:
        #    print actionsPickedTemp
        #    print statePairs
        #    print actionsSkipped
        #    print toSkip
            
        #    raise Exception("Repeat!!")
        return actionsSkipped
        
    def pickActions(self,varname,numActionsToPick,prevStatePairs):
        #just so that we can get doors
        #for connected states
        import random
        statePairs = [] 
        actionsPicked = []
        #pick actions at random
        actions = self.actions
        n = numActionsToPick
        #repeated = False 
        while(len(actionsPicked) != numActionsToPick):
            actionsPickedTemp = actionsPicked + random.sample(actions,n)
            print actionsPickedTemp
            
            toSkip = self.findActionsToSkip(actionsPickedTemp,varname,prevStatePairs)
            print toSkip
            
            if (len(toSkip) != 0):
                actions = list(set(actions).symmetric_difference(set(toSkip)))
                actionsPickedTemp = list(set(actionsPickedTemp).symmetric_difference(set(toSkip)))
                #print "Repeat"
                #repeated = True 
                #raw_input()
            actionsPicked = actionsPickedTemp
            print actionsPicked
            n = numActionsToPick - len(actionsPicked)
            print n
            
            
        #last check
        #if repeated:
        #    print actionsPicked
        #    raw_input()
  
        #print "Actions Picked"
        #print actionsPicked
        #now lets add the state pairs
        #so now we only care about this var name
        #for each action
        #we're going to get the value of this variable
        statePairs = []
        for a in actionsPicked:
            statePairs.append(int(a.getVarValueSrc(varname)))
            statePairs.append(int(a.getVarValueDest(varname)))
        #print statePairs
        #raw_input()
        return statePairs
                                
                        
                    
            
        
    def addVariable(self,name,minv,maxv,initv,v):
        pvar = PrismVariable()
        pvar.create(name,v,minv,maxv,initv,VariableType.notConst,"int")
        #print pvar
        self.variables[name]=pvar
        
    def processVariables(self):
        for i in range(len(self.moduleLines)):
            line = self.moduleLines[i]
            if RegexHelper.isRegexMatch(self.moduleVarsREConstants,line):
                if RegexHelper.isRegexMatch(self.moduleVarsRE,line):
                    varInfo = RegexHelper.getRegexMatchName(self.moduleVarsRE,line)
                    #name, minv, maxv ,initv
                    name = varInfo[0].rstrip().lstrip()
                    minv = varInfo[1].rstrip().lstrip()
                    maxv = varInfo[2].rstrip().lstrip()
                    initv = varInfo[3].rstrip().lstrip()
                    if minv in self.constants:
                        minv = self.constants[minv]
                    if maxv in self.constants:
                        maxv = self.constants[maxv]
                    
                        #print name
                        #print minv
                        #print maxv
                        #print initv
                        #print varInfo
                else:
                    varInfo = RegexHelper.getRegexMatchName(self.moduleVarsREConstants,line)
                    name = varInfo[0].rstrip().lstrip()
                    minv = varInfo[1].rstrip().lstrip()
                    maxv = varInfo[2].rstrip().lstrip()
                    
                    initv = varInfo[3].rstrip().lstrip()
                    #print initv

                    if minv in self.constants:
                        minv = self.constants[minv]
                    if maxv in self.constants:
                        maxv = self.constants[maxv]
                    if initv in self.constants:
                        initv = self.constants[initv]
                        #print initv
                        #print type(initv)
                    
                pvar = PrismVariable()
                pvar.create(name,initv,minv,maxv,initv,VariableType.notConst,"int")
                self.variables[name]=pvar

    def changeVariableInitValue(self,name,value):
        if name in self.variables:
            pvar = self.variables[name]
            pvar.initValue = value;
            return True
        return False 

    def changeVariableMinMaxRange(self,name,minV,maxV):
        if name in self.variables:
            pvar = self.variables[name]
            pvar.maxValue = maxV
            pvar.minValue = minV
            return True
        return False


    def changeVariableMinValue(self,name,minV):
        if name in self.variables:
            self.variables[name].minValue = minV
            
        
    def processRewards(self,rewardsLines,rewName):
        #process rewards
        rewNum = 0
        actNum = 0
        if not rewName in self.rewardNames:
            self.rewardNames.append(rewName)
            
        for i in range(len(rewardsLines)):
            line = rewardsLines[i]
            if RegexHelper.isRegexMatch(self.rewardsRE,line):
                resVals = RegexHelper.getRegexMatchName(self.rewardsRE,line)
                name = resVals[0]
                src = resVals[1]
                value = resVals[2]
                src = RegexHelper.processState(src,self.constants,self.variables)
                if(self.actions[actNum].equals(name,src)):
                    try:
                        if value.rstrip().lstrip() in self.constants:
                            value = self.constants[value.rstrip().lstrip()]
                            self.actions[actNum].addReward(rewName,value)
                        else:
                            self.actions[actNum].addReward(rewName,float(value))
                        actNum = actNum+1
                    except:
                        self.actions[actNum].addReward(rewName,value)
                        actNum = actNum+1
                        

    def createDoorCheckAction(self,dv,src,slab,doorUnknown,doorOpen,doorClosed,oprob,cprob):
        slabAltername = src.name+str(src.value) 
        pa = PrismAction(None,None,None,False)
        if slab is not None:
            pa.name = "c"+dv+"_"+slab
        else:
            pa.name = "c"+dv+"_"+slabAltername
        pa.addStateToSource(src)
        pa.addStateToSource(doorUnknown[0])
        pa.addStateToDestination([src,doorOpen[0]],oprob)
        pa.addStateToDestination([src,doorClosed[0]],cprob)
        for rew in self.rewardNames:
            pa.addReward(rew,1.0)
            
        #print pa
        #print pa.prismStringAction()
        return pa

    def createAction(self,name,rewVals,srcString,destString,probs):
        src = []
        dest = [] 
        if type(srcString) is list:
            for srcStr in srcString:
                s = RegexHelper.processState(srcStr,self.constants,self.variables)
                src = src + s
        else:
            src = RegexHelper.processState(srcString,self.constants,self.variables)
        if type(destString) is list:
            for destStr in destString:
                d = RegexHelper.processState(destStr,self.constants,self.variables)
                dest = dest +d
        else:
            dest = RegexHelper.processState(destString,self.constants,self.variables)
        pa = PrismAction(None,None,None,False)
        pa.name = name
        pa.addStateToSource(src)
        pa.addStateToDestination(dest,probs)
        for rew in rewVals:
            if not rew in self.rewardNames:
                self.rewardNames.append(rew)
            pa.addReward(rew,rewVals[rew])
            
        return pa 
            
    def addDoor(self,labels,doorVarName,src1,src2,oprob,cprob):
        #check if this door var exists
        print "Adding Door %s" % doorVarName
        if doorVarName in self.variables:
            #src1 = RegexHelper.processState(s1,self.constants,self.variables)
            #src2 = RegexHelper.processState(s2,self.constants,self.variables)
            #print src1
            #print src2
            src1=src1[0]
            src2=src2[0]
            #so the door check action
            print "Adding Door Check Actions"
            doorUnknown = RegexHelper.processState("("+doorVarName+"=unknown)",self.constants,self.variables)
            doorClosed = RegexHelper.processState("("+doorVarName+"=closed)",self.constants,self.variables)
            doorOpen = RegexHelper.processState("("+doorVarName+"=open)",self.constants,self.variables)
            src1label = labels[0]
            src2label = labels[1]
            pa1 = self.createDoorCheckAction(doorVarName,src1,src1label,doorUnknown,doorOpen,doorClosed,oprob,cprob)
            pa2 = self.createDoorCheckAction(doorVarName,src2,src2label,doorUnknown,doorOpen,doorClosed,oprob,cprob)
            
            #print "Finding Actions"
            #just check if this state is in the actions
            for i in range(len(self.actions)):
                if self.actions[i].matchOnlyOneSrcOrDest(src1) and self.actions[i].matchOnlyOneSrcOrDest(src2):
                    #print "Action Found"
                    #print self.actions[i]
                    self.actions[i].addStateToSource(doorOpen[0])
                    #print self.actions[i]
                    #print ""
            self.actions.append(pa1)
            self.actions.append(pa2)
            #print "Door Check Actions"
            #print pa1
            #print pa2
            #raw_input()
        else:
            print "No such door var "+doorVarName

        print "All done"

        
    def addFailureState(self,actionName,stateString,fprob,sprob,isDest=False):
        #so this action
        #and this state
        src = RegexHelper.processState(stateString,self.constants,self.variables)
        #print src
        #print type(src)
        fvar = PrismVariable()
        fvar.createUsingOtherPrismVariable(src[0],self.constants["failstate"])
        #print fvar
        for i in range(len(self.actions)):
            if isDest:
                thisActionMatches = self.actions[i].name == actionName and self.matchOnlyOneDest(src)
            else:
                thisActionMatches = self.actions[i].equals(actionName,src)
            if thisActionMatches:
                self.actions[i].updateWithFailureState(fvar,fprob,sprob)
                
    def addFailureStateToAllActions(self,stateString,fprob,sprob,isDest=False):
        src = RegexHelper.processState(stateString,self.constants,self.variables)
        fvar = PrismVariable()
        fvar.createUsingOtherPrismVariable(src[0],self.constants["failstate"])
        for i in range(len(self.actions)):
            if isDest:
                thisActionMatches = self.actions[i].matchOnlyOneDest(src[0])
            else:
                thisActionMatches= self.actions[i].matchOnlyOneSrc(src[0])
            if thisActionMatches: #
                self.actions[i].updateWithFailureState(fvar,fprob,sprob)
                

    
        
    def createModuleLines(self):
        lines = ["module " + self.name]
        for var in self.variables:
            #print var
            #print self.variables[var]
            line = self.variables[var].prismString()
            lines.append(line)
            
        for act in self.actions:
            line = act.prismStringAction()
            lines.append(line)

        lines.append("endmodule")
        return lines
    
    def createRewardLines(self,rewname):
        #lines = ["rewards" + ' "' +rewname + '"']
        lines = []
        for act in self.actions:
            line = act.prismStringReward(rewname)
            lines.append(line)

        #lines.append("endrewards")
        return lines

    
        
