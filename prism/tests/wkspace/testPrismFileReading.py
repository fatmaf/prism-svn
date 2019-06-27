from PrismFileReader import PrismFileReader
import random
import os

def diffLists(l1,l2):
    l1s = set(l1)
    l2s = set(l2)
    res = l1s.symmetric_difference(l2s)
    return list(res)

def sample(l,n):
    return random.sample(l,n)

def createStateString(variable,s):
    ss = '('+variable+'='+str(s)+')'
    return ss

def pickStates(previousStatesList,numToPick,currentStatesList):
    statesPicked = []
    n = numToPick
    if previousStatesList is not None:
        statesPicked = previousStatesList
        n = n - len(statesPicked)
    if n> 0:
        statesToPickFrom = diffLists(statesPicked,currentStatesList)
        newStatesPicked = sample(statesToPickFrom,n)
        statesPicked = statesPicked + newStatesPicked
    return statesPicked

def ensureDir(dirLoc):
    if not os.path.exists(dirLoc):
        os.makedirs(dirLoc)

def getTermWidth():
    rows, columns = os.popen('stty size', 'r').read().split()
    width = int(columns)
    return width

def printC(string,width):
    #width = os.get_terminal_size().columns
    
    
    print(string.center(width))

    
def generatePrismFiles(sVar,fpath,fn,numAgents,numGoals,numFailStates,numDoorStates,numDoors,saveLoc,previousInitStates,previousGoalStates,previousFailStates,previousDoorStates):
    if numDoors != numDoorStates:
        print "error"
        return 
    resDir = saveLoc+fn+"/"+"r"+str(numAgents)+"/t"+str(numGoals)+"/fs"+str(numFailStates)+"/d"+str(numDoors)+"/"
    saveresdir = resDir +"results/"
    ensureDir(saveresdir)
    
    pfr = PrismFileReader(fpath+fn+".prism")

    #assumption there is no door
    #add door variables
    pfr.addConstant('unknown','int','-1')
    pfr.addConstant('open','int','1')
    pfr.addConstant('closed','int','0')

    #check if it has a failstate
    #otherwise add it
    failstate = 'failstate'
    if not failstate in pfr.constants:
        pfr.addConstant(failstate,'int','-1')

    successProbConstant = 'p'
    succProb = 0.8
    if not successProbConstant in pfr.constants:
        pfr.addConstant(successProbConstant,'double',str(succProb))
        

    #assumption the module is called world
    moduleName = pfr.moduleNames[0]
    print "Module"
    print moduleName

    #get the first variable
    variables = pfr.getVariables(moduleName)

    #cleaning up doors
    doorVars = []
    #does the model have any 'doors' before hand
    for v in variables:
        if 'door' in v:
            doorVars.append(v)
            
            #raise Exception("Door in this model")
    for doorVar in doorVars:
        del variables[doorVar]

    for v in variables:
        if 'door' in v:
            raise Exception("Door in this model:"+str(variables))
    #print "Variables"
    #print variables
    #just get the first one
    if sVar is not None and sVar in variables:
        variable = v
    else:
        #pick the variable with the greatest range
        maxRange = 0
        
        for v in variables:
            vRange = variables[v].getRangeValues()
            varRange = vRange[1]-vRange[0]
            if(varRange > maxRange):
                maxRange = varRange
                variable =v 
            

    print "Variable"
    print variable

    #lets get its range
    varRange = variables[variable].getRangeValues()
    #if range doesnt have a failstate we'll have to modify this variable
    if varRange[0] != -1:
        pfr.changeVariableMinRange(variable,pfr.constants[failstate])
        varRange = variables[variable].getRangeValues()

    varRangeVals = range(varRange[0]+1,varRange[1])

    numStates = len(varRangeVals)


    #generate the states we need
    #initial positions
    #print "Generating states of concern"
    if len(previousDoorStates)>=numDoors*2:
        doorStates = previousDoorStates[0:numDoors*2]
    else:
        previousDoors = len(previousDoorStates)/2
        numDoors = numDoors - previousDoors
        doorStates= pfr.pickActions(moduleName,variable,numDoors)
        doorStates = previousDoorStates + doorStates
        
    if(len(set(doorStates)) != len(doorStates)):
        print doorStates
        print "Door States have repeats!!"
        raise Exception("Door states have repeats ")
        #return 
        #raw_input()
    statesLeft = diffLists(varRangeVals,doorStates)
    initStates = pickStates(previousInitStates,numAgents,statesLeft)
    statesLeft = diffLists(statesLeft,initStates)
    goalStates = pickStates(previousGoalStates,numGoals,statesLeft)
    statesLeft = diffLists(statesLeft,goalStates)
    if numFailStates > numGoals:
        failStates = pickStates(previousFailStates,numFailStates-numGoals,statesLeft)
        statesLeft = diffLists(statesLeft,failStates)
        failStates = failStates + goalStates
    else:
        failStates = goalStates[0:numFailStates]
    #instead of picking doorstates randomly
    #pick two connected ones
    #we can do this by picking from actions instead
    
    #doorStates = pickStates(previousDoorStates,numDoorStates*2,statesLeft)
    #sample(statesLeft,numDoorStates*2)
    #statesLeft = diffLists(statesLeft,doorStates)
    


    #add fail states
    print "Adding failure states"
    for s in failStates:
        ss = createStateString(variable,s)
        #print ss
        pfr.addFailureStateToAllActions(moduleName,ss,'1-p','p',isDest=True)
        

    for i in range(0,numDoors):
        #add door variables
        doorVarName = 'door'+str(i)
        pfr.addVariable(moduleName,doorVarName,pfr.constants['unknown'],pfr.constants['open'],pfr.constants['unknown'],pfr.constants['unknown'])

    numDoorVar = 0
    for i in range(0,len(doorStates),2):
        if(numDoorVar == numDoors):
            break
        doorVarName = 'door'+str(numDoorVar)
        s1 = doorStates[i]
        s2 = doorStates[i+1]
        #print doorVarName
        #print s1
        #print s2

        #raw_input()
        s1s = createStateString(variable,s1)
        s2s = createStateString(variable,s2)
        pfr.addDoor(moduleName,doorVarName,s1s,s2s,'p','1-p')
        numDoorVar = numDoorVar+1

    ensureDir(resDir)
    pfr.writeGoalStates(goalStates,variable,resDir+fn+'.props')
    pfr.writeGoalStatesSplit(goalStates,variable,resDir+fn+'.prop')

    #now lets do stuff
    for i in range(numAgents):
        
        pfr.changeVariableInitValue(moduleName,variable,initStates[i])
        
        lines = pfr.createFileLinesToWrite()
        pfr.writeLinesToFile(lines,resDir+fn+str(i)+'.prism')

    #print "Initial States"
    #print initStates
    #print "Goal States"
    #print goalStates
    #print "Fail States"
    #print failStates
    #print "Doors"
    #print doorStates
    print "File Loc "+resDir
    #raw_input()
    pfr.cleanModules()
    return (initStates,goalStates,failStates,doorStates)

if __name__ == "__main__":
    fpath = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/decomp_tests/"
    saveLoc = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/autogen_testfiles/"
    fn = "example"
    fn = "tro_example"
    fn = "grid_3_topomap"

    sVar = None
    goalsRange = [2,4,6,8,10]
    agentsRange = [2,4,6,8,10]
    fsRange = [0,2,4,8,16,32]
    ssRange = [0,1,2,3,4]

    termWidth = getTermWidth()
    
    totalFiles = len(goalsRange)*len(agentsRange)*len(fsRange)*len(ssRange)
    print "Files to generate = "+str(totalFiles)
    
    numGoals = 2
    numDoors = 1
    numAgents = 2
    numFailStates = 3
    numDoorStates = numDoors
    fnNum = 1
    #previousInitStates,previousGoalStates,previousFailStates,previousDoorStates):
    previousInitStates = [3,49,30,19,14,10,44,40,33,11]
    previousGoalStates = [28,27,59,58,47,8,22,36,50,51,52]
    previousDoorStates = [22,24,38,50,52,10,53,4,21,22]
    for numAgents in agentsRange:
        for numGoals in goalsRange:
            for numFailStates in fsRange:
                for numDoors in ssRange:
                    numDoorStates = numDoors
                    m = "==================== Processing "+str(fnNum)+" of "+str(totalFiles)+" ===================="
                    printC(m,termWidth)
                    m = 'Agents: %d, Tasks: %d, FailStates: %d, Doors: %d' % (numAgents,numGoals,numFailStates,numDoors)
                    printC(m,termWidth)
                    
                    generatePrismFiles(sVar,fpath,fn,numAgents,numGoals,numFailStates,numDoorStates,numDoors,saveLoc,previousInitStates,previousGoalStates,None,previousDoorStates)
                    m = "==================== Generated "+str(fnNum)+" of "+str(totalFiles)+" ===================="
                    printC(m,termWidth)
                    #raw_input()
                    
                    fnNum = fnNum+1

    #Use the stuff below to remind yourself of what is possible with this code
    
    #edit stuff here
    #pfr.addConstant('p1',"double",'0.2')
    #pfr.addConstant('sinit','int','2')
    #pfr.addConstant('unknown','int','-1')
    #pfr.addConstant('open','int','1')
    #pfr.addConstant('closed','int','0')
    #print pfr.constants
    #for x in pfr.constants:
    #    print pfr.constants[x].name
        
    #pfr.addVariable("world",'door',pfr.constants['unknown'],pfr.constants['open'],pfr.constants['unknown'],pfr.constants['unknown'])
    
    #pfr.addDoor("world",'door','(s=2)','(s=3)','p','1-p')
    #pfr.changeVariableInitValue("world",'s',pfr.constants['sinit'])
    #pfr.addFailureState("world",'v0_2','(s=0)','1-p','p')
    #pfr.addFailureStateToAllActions("world",'(s=5)','1-p','p',isDest=True)
    #variables=pfr.getVariables("world")
    #print variables
    #rangeOfVar = variables['s'].getRangeValues()
    #rangeOfVar[0] = rangeOfVar[0]+1
    #init states
    #numInitStates = 2
    #numGoalStates = 2
    #numFailStates = 3
    #numDoors = 1
    #rangeList = range(rangeOfVar[0]+1,rangeOfVar[1])
    #initStates = random.sample(rangeList,numInitStates)
    #print initStates
    #goalStatesRange = list(set(rangeList) - set(initStates))
    #print goalStatesRange
    #goalStates = random.sample(goalStatesRange,numGoalStates)
    #print goalStates
    #failStates = random.sample(list(set(goalStatesRange)-set(goalStates)),numFailStates-numGoalStates)
    #failStates = failStates + goalStates
    #print failStates
    #doorStatesRange = list(set(goalStatesRange)-set(failStates))
    #print doorStatesRange
    #doorStates = random.sample(doorStatesRange,numDoors*2)
    #print doorStates
    
    #just know that the failstate is -1 so start from 0 onwards
    #for i in range(rangeOfVar[0]+1,rangeOfVar[1]+1):
    #    #generate init states 

    #dont know what 
    


