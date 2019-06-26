from PrismFileReader import PrismFileReader
import random

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

def generatePrismFiles(fn,numAgents,numGoals,numFailStates,numDoorStates):
    pfr = PrismFileReader(fn)

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
    print "Variables"
    print variables
    #just get the first one
    for v in variables:
        variable = v
        break

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
    statesLeft = varRangeVals 
    initStates = sample(statesLeft,numAgents)
    statesLeft = diffLists(statesLeft,initStates)
    goalStates = sample(statesLeft,numGoals)
    statesLeft = diffLists(statesLeft,goalStates)
    if numFailStates > numGoals:
        failStates = sample(statesLeft,numFailStates - numGoals)
        statesLeft = diffLists(statesLeft,failStates)
        failStates = failStates + goalStates
    else:
        failStates = goalStates[0:numFailStates]
    doorStates = sample(statesLeft,numDoorStates*2)
    statesLeft = diffLists(statesLeft,doorStates)

    print "Initial States"
    print initStates
    print "Goal States"
    print goalStates
    print "Fail States"
    print failStates
    print "Doors"
    print doorStates
    
    #now lets do some stuff
    #everything is the same except the init states
    #so lets do the other stuff first

    #add fail states
    for s in failStates:
        ss = createStateString(variable,s)
        print ss
        pfr.addFailureStateToAllActions(moduleName,ss,'1-p','p',isDest=True)
        

    for i in range(0,numDoors):
        #add door variables
        doorVarName = 'door'+str(i)
        pfr.addVariable(moduleName,doorVarName,pfr.constants['unknown'],pfr.constants['open'],pfr.constants['unknown'],pfr.constants['unknown'])
        
    for i in range(0,len(doorStates),2):
        s1 = doorStates[i]
        s2 = doorStates[i+1]
        s1s = createStateString(variable,s1)
        s2s = createStateString(variable,s2)
        pfr.addDoor(moduleName,doorVarName,s1s,s2s,'p','1-p')

    pfr.writeGoalStates(goalStates,variable,'temp.props')
    pfr.writeGoalStatesSplit(goalStates,variable,'temp.prop')

    #now lets do stuff
    for i in range(numAgents):
        
        pfr.changeVariableInitValue(moduleName,variable,initStates[i])
        
        lines = pfr.createFileLinesToWrite()
        pfr.writeLinesToFile(lines,'temp'+str(i)+'.prism')

    

if __name__ == "__main__":
    fn = "example.prism"

    numGoals = 2
    numDoors = 1
    numAgents = 2
    numFailStates = 3
    numDoorStates = 1

    generatePrismFiles(fn,numAgents,numGoals,numFailStates,numDoorStates)
        
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
    


