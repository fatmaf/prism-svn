from PrismModule import PrismModule
from PrismVariable import PrismVariable
from PrismVariable import VariableType
from RegexHelper import RegexHelper
        
    
class PrismFileReader(object):
    fileType = "mdp"
    moduleBegin = "module"
    moduleEnd = "endmodule"
    rewardsBegin = "rewards"
    rewardsEnd = "endrewards"

    moduleRE = moduleBegin+"(.*)"
    rewardsRE = rewardsBegin+"(.*)"

    #moduleRange= {}
    #rewardsRange = {}
    
    #moduleNames = []
    #rewardNames = []

    #moduleLines = {}
    #rewardsLines = {}

    #labelLines = []
    labelString = "label"

    constREintDouble = "const (?:int|double) (.*) =( ?[-]?[\d+]?[.]?[\d+]);"
    constRE ='const (.*) =( ?[-]?[\d+]?[.]?[\d+]);'

    #modVars = [] 
    
    #constants={}
    
    def __init__(self,fn):
        self.moduleRange= {}
        self.rewardsRange = {}
    
        self.moduleNames = []
        self.rewardNames = []

        self.moduleLines = {}
        self.rewardsLines = {}

        self.labelLines = []
        
        self.modVars = [] 
    
        self.constants={}

        self.labels = None
        if not fn is None:
            print "Initialising file reader"
            self.fileName = fn
            print ("Reading file "+fn)
            self.fileLines = []
            with open(fn) as f:
                for line in f.readlines():
                
                    lline = line.replace('\n','')
                    lline = lline.replace('\t','')
                
                    self.fileLines.append(lline)
                    #print lline
                
            #print self.fileLines
            self.processFile()
        else:
            print "an empty file"
            


    
    def cleanModules(self):
        for mod in self.modVars:
            mod.cleanModule()
            
    def pickActions(self,modname,varname,numActionsToPick,prevStatePairs):
        statePairs = []
        for modVar in self.modVars:
            if self.matchModVarName(modVar.name,modname):
                statePairs = modVar.pickActions(varname,numActionsToPick,prevStatePairs)
                break
        return statePairs

                
    def processFile(self):

        fileIsMDP = False
        
        #get the file type
        for line in self.fileLines:
            if (self.fileType in line):
                fileIsMDP = True
                break
            
        if not fileIsMDP:
            raise Exception('MDP file type not detected')
        else:
            print "File type MDP"

        lastIndexForModule = 0
        rewStart = 0
        
        while(lastIndexForModule != -1):
            lastIndexForModule= self.readModuleLines(lastIndexForModule)
            if (lastIndexForModule != -1):
                rewStart = lastIndexForModule
        while(rewStart != -1):
            rewStart = self.readRewardLines(rewStart)
        self.readLabels(0)
    
        self.processConstants()

        allVariables = {}
        for n in self.moduleNames:
            modVar = PrismModule(n,self.moduleLines[n],self.constants)
            for r in self.rewardNames:
                modVar.processRewards(self.rewardsLines[r],r)
            
            self.modVars.append(modVar)
            allVariables = self.merge_two_dicts(modVar.variables,allVariables)
            
        self.processLabels(self.labelLines,allVariables)
        
    
            
    def merge_two_dicts(self,x, y):
        """Given two dicts, merge them into a new dict as a shallow copy."""
        z = x.copy()
        z.update(y)
        return z
    
    def createFileLinesToWrite(self):
        lines = []
        empties = ["",""]
        lines.append("mdp")
        #create the constants
        for c in self.constants:
            lines.append(self.constants[c].prismString())
      
        mlines = []
        rlines = [] 
        for modVar in self.modVars:
            mlines = mlines + modVar.createModuleLines()
        for r in self.rewardNames:
            rlines.append("rewards" +r)
            for modVar in self.modVars:
                rlines = rlines + modVar.createRewardLines(r)
            rlines.append('endrewards')
            
            #print mlines
                
        llines = self.getLabelLines()        
        #create the modules and rewards
        lines = lines +empties+ mlines+ empties+llines +empties+ rlines
        
        return lines

    def writeLinesToFile(self,lines,fn):
        f = open(fn,"w+")
        for line in lines:
            f.write(line+"\n")
        f.close()

    def createGoalStates(self,gs,gVar):
        goalVariable = None
        for modVar in self.modVars:
            if gVar in modVar.variables:
                goalVariable = modVar.variables[gVar]
        prefix = "Pmax=? [ "
        goalString = prefix
        for i in range(len(gs)):
            stateString = "(F ("+gVar+"="+str(gs[i])+") )"
            if (i != len(gs)-1):
                goalString = goalString + stateString + "& "
            else:
                goalString = goalString + stateString
        goalString = goalString + "]"

        return goalString

    def createGoalStatesSplit(self,gs,gVar):

        #partial(R{"time"}min=? [ F "v7" ], Pmax=? [ F "v8" ], Pmax=? [ F "v9" ], Pmax=? [ G (!"v6") ])
        goalVariable = None
        for modVar in self.modVars:
            if gVar in modVar.variables:
                goalVariable = modVar.variables[gVar]
        prefix = 'partial(R{"time"}min=? '
        goalString = prefix
        for i in range(len(gs)):
            stateString = "[F ("+gVar+"="+str(gs[i])+") ]"
            if (i != len(gs)-1):
                goalString = goalString + stateString + ", "
            else:
                goalString = goalString + stateString
            break
        
        for i in range(1,len(gs)):
            stateString = "Pmax=? [F ("+gVar+"="+str(gs[i])+") ]"
            if (i != len(gs)-1):
                goalString = goalString + stateString + ", "
            else:
                goalString = goalString + stateString
        goalString = goalString + ")"

        return goalString

    def createGoalStatesAvoid(self,gs,avoid,gVar):
        goalVariable = None
        for modVar in self.modVars:
            if gVar in modVar.variables:
                goalVariable = modVar.variables[gVar]
        prefix = "Pmax=? [ "
        goalString = prefix
        for i in range(len(gs)):
            stateString = "(F ("+gVar+"="+str(gs[i])+") )"
            if (i != len(gs)-1):
                goalString = goalString + stateString + "& "
            else:
                goalString = goalString + stateString

        for i in range(len(avoid)):
            stateString = " & (G ( ! ("+gVar+"="+str(avoid[i])+") ) ) "
            goalString = goalString + stateString
            
        goalString = goalString + "]"

        return goalString


    def createGoalStatesAvoidReward(self,gs,avoid,gVar,rewName):
        goalVariable = None
        for modVar in self.modVars:
            if gVar in modVar.variables:
                goalVariable = modVar.variables[gVar]
        
        prefix = "R"+"{"+rewName+"}max=? [ "
        goalString = prefix
        for i in range(len(gs)):
            stateString = "(F ("+gVar+"="+str(gs[i])+") )"
            if (i != len(gs)-1):
                goalString = goalString + stateString + "& "
            else:
                goalString = goalString + stateString

        for i in range(len(avoid)):
            stateString = " & (G ( ! ("+gVar+"="+str(avoid[i])+") ) ) "
            goalString = goalString + stateString
            
        goalString = goalString + "]"

        return goalString
    
    
    
    
    def findLabel(self,src):
        for label in self.labels:
            lsrc = self.labels[label]
            matched = True
            if len(lsrc) == len(src):
                for i in range(len(lsrc)):
                    l = lsrc[i]
                    s = src[i]
                    if not l.equals(s):
                        matched = False
                        break
            else:
                matched = False
            if matched:    
                return label
        return None
    
    def createGoalStatesSplitAvoidLabels(self,gs,avoid,gVar):
        #partial(R{"time"}min=? [ F "v7" ], Pmax=? [ F "v8" ], Pmax=? [ F "v9" ], Pmax=? [ G (!"v6") ])
        goalVariable = None
        
        for modVar in self.modVars:
            if gVar in modVar.variables:
                goalVariable = modVar.variables[gVar]
                variables = modVar.variables 
        prefix = 'partial(R{"time"}min=? '
        goalString = prefix
        for i in range(len(gs)):
            lp = "("+gVar+"="+str(gs[i])+")"
            src= RegexHelper.processState(lp,self.constants,variables)
            label = self.findLabel(src)
            if label is None:
                label = lp
            else:
                label = '"'+label+'"'
            #print label 
                
            stateString = "[F ("+label+") ]"
            if (i != len(gs)-1):
                goalString = goalString + stateString + ", "
            else:
                goalString = goalString + stateString
            break
        
        for i in range(1,len(gs)):
            lp = "("+gVar+"="+str(gs[i])+")"
            src= RegexHelper.processState(lp,self.constants,variables)
            label = self.findLabel(src)
            if label is None:
                label = lp
            else:
                label = '"'+label+'"'
            #print label 
            stateString = "Pmax=? [F ("+label+") ]"
            if (i != len(gs)-1):
                goalString = goalString + stateString + ", "
            else:
                goalString = goalString + stateString

        if len(avoid) > 0:
            goalString = goalString + ', Pmax=? [ '
            for i in range(len(avoid)):
                lp = '('+gVar+'='+str(avoid[i])+') '
                src= RegexHelper.processState(lp,self.constants,variables)
                label = self.findLabel(src)
                if label is None:
                    label = lp
                else:
                    label = '"'+label+'"'    
                stateString = '( G ! ('+label+') )'
                if (i != len(avoid) -1):
                    goalString = goalString + stateString + " & "
                else:
                    goalString = goalString + stateString
        
            goalString = goalString + ']' 
        goalString = goalString + ")"
        #print goalString 

        return goalString

    
    def writeGoalStates(self,gs,gVar,fn):
        goalString = self.createGoalStates(gs,gVar)
        self.writeLinesToFile([goalString],fn)
        
    def writeGoalStatesSplit(self,gs,gVar,fn):
        goalString = self.createGoalStatesSplit(gs,gVar)
        self.writeLinesToFile([goalString],fn)
        
    def writeGoalStatesAvoid(self,gs,avoid,gVar,fn):
        goalString = self.createGoalStatesAvoid(gs,avoid,gVar)
        self.writeLinesToFile([goalString],fn)

    def writeGoalStatesAvoidReward(self,gs,avoid,gVar,fn,rewName):
        goalString = self.createGoalStatesAvoidReward(gs,avoid,gVar,rewName)
        self.writeLinesToFile([goalString],fn)
        
    def writeGoalStatesSplitAvoidLabels(self,gs,avoid,gVar,fn):
        goalString = self.createGoalStatesSplitAvoidLabels(gs,avoid,gVar)
        self.writeLinesToFile([goalString],fn)        
        
    def readRewardLines(self,beginIndex):
        rewardBegan = False
        rewardsRange=[]
        rewardName = None
        
        for i in range(beginIndex,len(self.fileLines)):
            line = self.fileLines[i]
            if not rewardBegan and RegexHelper.isRegexMatch(self.rewardsRE,line):
                #print i
                rewardsRange.append(i)
                rewardBegan = True
                rewardName = RegexHelper.getRegexMatchName(self.rewardsRE,line)
                self.rewardNames.append(rewardName)
                
            if rewardBegan and RegexHelper.isRegexMatch(self.rewardsEnd,line):
                #print i
                rewardsRange.append(i)
                rewardsBegan = False
                break
            
        if rewardName is not None:
            self.rewardsRange[rewardName] = rewardsRange
            self.rewardsLines[rewardName]=self.fileLines[rewardsRange[0]+1:rewardsRange[1]]
        
        if rewardName is None:
            return -1
        return rewardsRange[1]
    
                
            
                
            
    def readModuleLines(self,beginIndex):
        moduleRange = []
        moduleBegan = False
        moduleName = None
        
        #read the module lines
        for i in range(beginIndex,len(self.fileLines)):
            line = self.fileLines[i]
            lineStripped = line
            if not moduleBegan and RegexHelper.isRegexMatch(self.moduleRE,lineStripped):
                #print i
                moduleRange.append(i)
                moduleName = RegexHelper.getRegexMatchName(self.moduleRE,lineStripped)
                self.moduleNames.append(moduleName)
                moduleBegan = True
            if moduleBegan and RegexHelper.isRegexMatch(self.moduleEnd,lineStripped):
                #print i
                moduleRange.append(i)
                moduleBegan = False
                break
            

        if moduleName is not None:
            self.moduleRange[moduleName] = moduleRange
            self.moduleLines[moduleName]=self.fileLines[moduleRange[0]+1:moduleRange[1]]

        if moduleName is None:
            return -1
        return moduleRange[1] 


    def readLabels(self,beginIndex):
        #labelRange = [-1,-1]
        labelsBegan = False
        
        for i in range(beginIndex,len(self.fileLines)):
            line = self.fileLines[i]
            #if "label" in line:
            #    print line
                
            if RegexHelper.isRegexMatch(self.labelString,line):
                #if labelRange[0] == -1:
                #    labelRange[0] = i
                #    labelsBegan = True 
                self.labelLines.append(line)
            #elif labelsBegan == True:
            #   labelRange[1] = i
            #   labelsBegan = False 
            #   break
            
    def processLabels(self,labelLines,variables):
        labelRE = "label \"(.*)\" = (.*);"
        self.labels={}
        for line in labelLines:
            labelParts = RegexHelper.getRegexMatchName(labelRE,line)
            #print labelParts
            label = labelParts[0]
            lp = labelParts[1]
            if lp[0] != '(' and lp[len(lp)-1] != ')':
                lp = '('+lp+')'
            src = RegexHelper.processState(lp,self.constants,variables)
            self.labels[label] = src

    def addLabel(self,label,stateString,variables):
        src = RegexHelper.processState(stateString,self.constants,variables)
        if self.labels is None:
            self.labels = {}
        self.labels[label] = src

        
    def stateString(self,src,isDest):
        app = '&'
        toret = ""
        for i in range(len(src)):
            var = src[i]
            toret = toret + var.stateString(isDest)
            if i != (len(src)-1):
                toret = toret + " & "
        return toret
    
    def getLabelLines(self):
        lines = []
        if not self.labels is None:
            for label in self.labels:
                src = self.labels[label]
                strsrc = self.stateString(src,False)
                line = "label "+'"'+label+'" = '+strsrc+";"
                lines.append(line)
        return lines
    
        
    def processConstants(self):
        beginIndex = 0
        for i in range(beginIndex,len(self.fileLines)):
            line = self.fileLines[i]
            
            #print line
            if RegexHelper.isRegexMatch(self.constRE,line):
                #print line
                #print line.split("=")
                subtype = None
                
                if RegexHelper.isRegexMatch(self.constREintDouble,line):
                    nameValuePair = RegexHelper.getRegexMatchName(self.constREintDouble,line)
                    if " int " in line:
                        subtype = "int"
                    if " double " in line:
                        subtype = "double"
                    
                else:
                    nameValuePair = RegexHelper.getRegexMatchName(self.constRE,line)
                #print "Name"+nameValuePair[0]
                #print "Value"+nameValuePair[1]
                name = nameValuePair[0].rstrip().lstrip()
                value = nameValuePair[1].rstrip().lstrip()
                try:
                    v = int(value)
                    subtype = "int"
                except ValueError:
                    try:
                        v = float(value)
                        subtype = "double"
                    except ValueError:
                        print("error parsing constant")
                        pass
                    
                        
                pvar =  PrismVariable()
                pvar.create(name,value,None,None,value,VariableType.const,subtype)
                self.constants[name] = pvar

    def addConstant(self,name,subType,val):
        pvar = PrismVariable()
        pvar.create(name,val,None,None,val,VariableType.const,subType)
        self.constants[name] = pvar

    def matchModVarName(self,n1,n2):
        n1 = n1.lstrip()
        n1 = n1.rstrip()
        n2 = n2.lstrip()
        n2 = n2.rstrip()
        return n1==n2
    
    def addVariable(self,modname,name,minv,maxv,initv,v):
        #print modname
        #print name
        #print minv
        #print maxv
        #print initv
        #print v
        for modVar in self.modVars:
            if self.matchModVarName(modname,modVar.name):
                modVar.addVariable(name,minv,maxv,initv,v)
                #print "Variable Added to module "+modname 
                #print modVar.variables 
                break

    def getVariables(self,modname):
        for modVar in self.modVars:
            if self.matchModVarName(modname,modVar.name):
                return modVar.variables
        return None
            

    def getLabel(self,src):
        #expecting an array of src
        matchedSoFar = True
        toret = None
        for l in self.labels:
            matchedSoFar = True
            lsrc = self.labels[l]
            if len(lsrc) == len(src):
                for i in range(len(src)):
                    ssrc = src[i]
                    slsrc = lsrc[i]
                    #print ssrc
                    #print slsrc
                    
                    if ssrc.name != slsrc.name:
                        matchedSoFar = False
                        break 
                    elif ssrc.value != slsrc.value:
                        matchedSoFar = False
                        break
    
            if matchedSoFar:
                toret = l
                break
            
        return toret
    
    def addDoor(self,modname,doorname,s1,s2,oprob,cprob):
        for modVar in self.modVars:
            if self.matchModVarName(modname,modVar.name):
                src1 = RegexHelper.processState(s1,self.constants,modVar.variables)
                src2 = RegexHelper.processState(s2,self.constants,modVar.variables)
                l1 = self.getLabel(src1)
                l2 = self.getLabel(src2)
                #print l1
                #print l2
                
                modVar.addDoor([l1,l2],doorname,src1,src2,oprob,cprob)
                break

    
    #change the init value of the module 
    def changeVariableInitValue(self,modname,name,value):
        changed = False 
        for modVar in self.modVars:
            if self.matchModVarName(modVar.name,modname):
                changed = modVar.changeVariableInitValue(name,value)
                break
        if not changed:
            print ("Could not change initial value for "+name+" in "+modname)

    def changeVariableMinMaxRange(self,modname,name,minv,maxv):
        changed = False
        for modVar in self.modVars:
            if self.matchModVarName(modVar.name,modname):
                changed = modVar.changeVariableMinMaxRange(name,minv,maxv)
                break
        if not changed:
            print ("Could not change min max for "+name+" in "+modname)
                

    def changeVariableMinValue(self,modname,name,minv):
        for modVar in self.modVars:
            if self.matchModVarName(modVar.name,modname):
                modVar.changeVariableMinValue(name,minv)
                break
            
    def addFailureState(self,modname,actionName,stateString,fprob,sprob,isDest=False):
        for modVar in self.modVars:
            if self.matchModVarName(modVar.name,modname):
                modVar.addFailureState(actionName,stateString,fprob,sprob,isDest)
                break
        

    def addFailureStateToAllActions(self,modname,stateString,fprob,sprob,isDest=False):
        #print "Adding failure states to module "+modname
        for modVar in self.modVars:
            if self.matchModVarName(modVar.name,modname):
                #print "found module"
                modVar.addFailureStateToAllActions(stateString,fprob,sprob,isDest)
                break
