import numpy

class ReadMDPStaTra(object):

    def __init__(self,stafilename,trafilename):
        self.stafilename = stafilename
        self.trafilename = trafilename
        self.varlist = None
        self.invvarlist = None
        self.staDict = None
        self.traDict = None
        self.currentstate = None
        self.doPolicyReactive = False
        self.methodtype = None
        if 'ssi' in self.stafilename:
            self.methodtype = 'Auctioning'
        elif 'stapu' in self.stafilename:
            self.methodtype = 'STAPU'
        elif 'finalJPvi' in self.stafilename:
            self.methodtype = 'STAPU'
        self.hasDoors = False 

    def printMessage(self,text):
        if self.methodtype is not None:
            toprint = self.methodtype + ":"+str(text)
        else:
            toprint = str(text)
        print(toprint)
        
    def removeBrackets(self,line):
        line = line.replace('(','')
        line = line.replace(')','')
        line = line.replace('\n','')
        return line

    def getLines(self,fn):
        lines = None
        with open(fn) as f:
            lines = f.readlines()
        return lines
       
    
    def readsta(self):
        self.printMessage ("Reading sta file: "+str(self.stafilename))
        lines = self.getLines(self.stafilename)
        staDict = {}
        for linenum in range(len(lines)):
            line = lines[linenum]
            if linenum == 0:
                #the first line is just the variables
                self.varlist = {}
                self.invvarlist = {}
                pline = self.removeBrackets(line)
                pline = pline.split(',')
                #each line is the name of a variable
                for varnum in range(len(pline)):
                    varname = pline[varnum]
                    self.varlist[varname]=varnum
                    self.invvarlist[varnum] = varname
                    if 'door' in varname:
                        self.hasDoors = True 
                
                self.printMessage ("Processed var names: "+str(self.varlist))
                self.printMessage ("Processed var nums: "+str(self.invvarlist))
                
            else:
                pline = line.split(':')
                statenum = int(pline[0])
                statevals = eval(pline[1])
                staDict[statenum]=statevals
        self.staDict = staDict
        #self.printMessage("Processed state vals")
        #self.printMessage (staDict)
        self.printMessage ("Read sta file")
        self.printMessage ("Variables "+str(self.varlist.keys()))
        self.printMessage ("Number of states "+str(len(self.staDict)))

    def readtra(self):
        #The first line of the file take the form "n c m", giving the number of states (n), the total number of choices (c) and the total number of transitions (m). The remaining lines are of the form "i k j x" or "i k j x a", where i and j are the row (source) and column (destination) indices of the transition, k is the index of the choice that it belongs to, and x is the probability of the transition. a is optional and gives the action label for the choice of the transition. Action labels can be present for some, all or no states but, in slightly redundant fashion, the action labels, if present, must be the same for all transitions belonging to the same choice.
        
        self.printMessage ("Reading tra file: "+str(self.trafilename))
        lines = self.getLines(self.trafilename)
        self.printMessage (len(lines))
        traDict = {}
        #first line nstates, nchoices, ntransitions
        for linenum in range(len(lines)):
            line = lines[linenum]
            line = line.strip()
            if linenum == 0:
                #reading the initial stuff
                pline = line.split(' ')
                numstates = int(pline[0])
                numchoices = int(pline[1])
                numtrans = int(pline[2])
                self.printMessage ("states,choices,transitions : "+str(numstates)+","+str(numchoices)+","+str(numtrans))
            else:
                pline = line.split(' ')
                hasaction = True
                if len(pline) != 5:
                    hasaction = False
                sourcestate = int(pline[0])
                choiceindex = int(pline[1])
                deststate = int(pline[2])
                tranprob = float(pline[3])
                if hasaction:
                    actionname = pline[4]

                if sourcestate not in traDict:
                    traDict[sourcestate] = {}
                    
                
                choicesDict = traDict[sourcestate]
                if choiceindex not in choicesDict:
                    choicesDict[choiceindex] = {'states':{}}
                    if hasaction:
                        choicesDict[choiceindex]['action']=actionname
                
                choiceDict = choicesDict[choiceindex]['states']
                choiceDict[deststate] = tranprob
                #self.printMessage(traDict)
                #raw_input("continue")
        self.traDict = traDict

    def matchstatevals(self,sv1,sv2):
        matched = True
        if len(sv1) != len(sv2):
            matched = False
        if matched:
            for i in range(len(sv1)):
                if sv1[i] != sv2[i]:
                    matched = False
                    break

        return matched 
            
    def findStateNum(self,stateval,doorStates = None):
        if doorStates is not None:
            newStateVals = [-1]*len(self.varlist)
            for varname in self.varlist:
                if varname in doorStates:
                    newStateVals[self.varlist[varname]]=doorStates[varname]
                else:
                    newStateVals[self.varlist[varname]]=stateval
        else:
            newStateVals = [stateval]

        statevalhere = -1000
        statetoret = -1
        for state in self.staDict:
            statevalhere = self.staDict[state]
            if type(statevalhere) is not tuple:
                statevalhere = list([statevalhere])
            if self.matchstatevals(newStateVals,statevalhere):
                statetoret = state
                break
        print ("State vals "+str(stateval))
        if doorStates is not None:
            print ("doors "+str(doorStates))
        print ("New state vals "+str(newStateVals))
        print ("state "+str(statetoret))
        print("state vals in dict" + str(statevalhere))
        return statetoret

    def getStateVal(self,statenum):
        stateval = None
        if statenum in self.staDict:
            stateval = self.staDict[statenum]
        #just return the agent state
        if type(stateval) is not tuple:
            agentstate = stateval
        else:
            agentstate = stateval[self.varlist['s']]
        return agentstate
    
    
    def findAction(self,state,actionname):
        actionIndex = -1
        if state in self.staDict:
            if state in self.traDict:
                choicesDict = self.traDict[state]
                for choiceindex in choicesDict:
                    if 'action' in choicesDict[choiceindex]:
                        if choicesDict[choiceindex]['action']==actionname:
                            actionIndex = choiceindex
                            break
        return actionIndex
    
    def simulateAction(self,state,actionname):
        
        actionIndex = self.findAction(state,actionname)
        nextstate = None
        if actionIndex!= -1:
            #get all the states
            choicesDict = self.traDict[state][actionIndex]
            states =choicesDict['states']
            a = []
            p =[]
            for s in states:
                a.append(s)
                p.append(states[s])
            nextstate = numpy.random.choice(a=a,p=p)
            nextstatep = states[nextstate]
        return (nextstate,nextstatep)
            

    def breakJointAction(self,actionname):
        if self.methodtype is None:
            splitactions = actionname
        else:
            doSTAPU = False 
            if self.methodtype == 'STAPU':
                doSTAPU = True
            splitactions = None
            if doSTAPU:
                splitactions=self.breakJointActionSTAPU(actionname)
            else:
                splitactions=self.breakJointActionSSI(actionname)
            
        self.printMessage(splitactions)
        #raw_input()
        return splitactions
            

    def breakJointActionSTAPU(self,actionname):
        regexPattern = r"r\d_"
        import re
        splitActions=re.split(regexPattern,actionname)
        splitActions = splitActions[1:]
        #cleanup the trailing _
        newsplitactions = []
        for a in splitActions:
            if a[-1] == '_':
                newsplitactions.append(a[:-1])
            else:
                newsplitactions.append(a)
        return newsplitactions

    def breakJointActionSSI(self,actionname):
        return actionname.split(',')
    
    def testBreakJointAction(self):
        for s in self.traDict:
            for c in self.traDict[s]:
                if 'action' in self.traDict[s][c]:
                    self.breakJointAction(self.traDict[s][c]['action'])

    def getAction(self,state,choiceNum=0):
        actionname = None
        if state in self.traDict:
            choicesDict = self.traDict[state]
            if len(choicesDict) > 1:
                self.printMessage("Not a markov chain")
            if choiceNum in choicesDict:
                if 'action' in choicesDict[choiceNum]:
                    actionname = choicesDict[choiceNum]['action']
        return actionname 
                    
    def getMostProbableStateReactive(self,state):
        otherstates = []
        if state in self.traDict:
            choicesDict = self.traDict[state]
            choiceNum = 0
            if choiceNum in choicesDict:
                choiceDict = choicesDict[choiceNum]['states']
                maxProb = 0
                nextState = -1
                for ns in choiceDict:
                    if choiceDict[ns] > maxProb:
                        maxProb = choiceDict[ns]
                        nextState = ns
                for ns in choiceDict:
                    if ns!=nextState:
                        if ns not in otherstates:
                            otherstates.append(ns)
                return (nextState,otherstates)
            else:
                self.printMessage ("choice "+str(choiceNum)+" for "+str(state)+" not in transitions list")
                        
                
        else:
            self.printMessage (str(state)+":"+str(self.staDict[state])+" not in transitions list")
        return None
            
    def doMostProbablePath(self,startState=0):
        if self.traDict is not None:
            if self.staDict is not None:
                q = [startState]
                while len(q) > 0:
                    currstate = q.pop(0)
                    #start with a state
                    #self.printMessage(currstate)
                    if currstate not in self.traDict:
                        self.printMessage (str(currstate)+" not in transitions list")
                        continue 
                    choicesDict = self.traDict[currstate]
                    choiceNum = 0
                    if choiceNum not in choicesDict:
                        self.printMessage ("choice "+choiceNum+" for "+str(currstate)+" not in transitions list")
                        continue 
                    choiceDict = choicesDict[choiceNum]['states']
                    maxProb = 0
                    nextState = -1
                    for state in choiceDict:
                        if choiceDict[state] > maxProb:
                            maxProb = choiceDict[state]
                            nextState = state
                    if nextState != -1:
                        q.append(nextState)

    def doAllPaths(self,startState = 0):
        if self.traDict is not None:
            if self.staDict is not None:
                startStates = [startState]
                while(len(startStates) > 0):
                    #raw_input("continue")
                    
                    currentstate = startStates.pop(0)
                    self.printMessage("List for "+str(currentstate))
                    q = [currentstate]
                    while(len(q)>0):
                        currstate = q.pop(0)
                        self.printMessage(currstate)
                        if currstate not in self.traDict:
                            self.printMessage (str(currstate)+" not in transitions list")
                            continue 
                        choicesDict = self.traDict[currstate]
                        choiceNum = 0
                        if choiceNum not in choicesDict:
                            self.printMessage ("choice "+choiceNum+" for "+str(currstate)+" not in transitions list")
                            continue 
                        choiceDict = choicesDict[choiceNum]['states']
                        maxProb = 0
                        nextState = -1
                        for state in choiceDict:
                            if choiceDict[state] > maxProb:
                                maxProb = choiceDict[state]
                                nextState = state
                        for state in choiceDict:
                            if state != nextState:
                                if state not in startStates:
                                    startStates.append(state)
                        if nextState != -1:
                            q.append(nextState)

    def getDoorStatesFromState(self,state):
        doorStates = {}
        if state in self.staDict:
            stateVals = self.staDict[state]
            for varname in self.varlist:
                if 'door' in varname:
                    varval = stateVals[self.varlist[varname]]
                    doorStates[varname] = varval

        return doorStates
    
    def getAgentStatesFromState(self,state):
        agentStates = {}
        if state in self.staDict:
            stateVals = self.staDict[state]
            for varname in self.varlist:
                import re
                pattern = re.compile(r"^r\d")
                if(pattern.match(varname)):
                    temp = varname.replace('r','')
                    temp = int(temp)
                    varval = stateVals[self.varlist[varname]]
                    agentStates[temp]=varval
                else:
                    pattern = re.compile(r"^s_r\d")
                    if (pattern.match(varname)):
                        temp = varname.replace('s_r','')
                        temp = int(temp)
                        varval = stateVals[self.varlist[varname]]
                        agentStates[temp]=varval
        #self.printMessage(agentStates)
        return (agentStates)

    def createStateFromAgentStates(self,previousState,agentStates,choiceNum=0):
        agentVals = [-1]*len(self.varlist)
        previousStateVals = self.staDict[previousState]
        varsdone = []
        for agnum in agentStates:
            if self.methodtype is not None:
                if self.methodtype == 'STAPU':
                    varname = 'r'+str(agnum)
                elif self.methodtype == 'Auctioning':
                    varname = 's_r'+str(agnum)
                else:
                    varname = 's'
                varindex = self.varlist[varname]
                agentVals[varindex] = agentStates[agnum]
                varsdone.append(varname)
        actualnextstate = None
        actualstateprob = 0
        if previousState in self.traDict:
            nextStates = self.traDict[previousState][choiceNum]['states']
            for s in nextStates:
                #now we just want to match these agent vals with the other ones
                nextstatevals = self.staDict[s]
                allmatch = True 
                for varname in varsdone:
                    varindex = self.varlist[varname]
                    if agentVals[varindex] != nextstatevals[varindex]:
                        allmatch = False

                if allmatch:
                    actualnextstate = s
                    actualstateprob = nextStates[s]
                    break
                
                    
                        
        return (actualnextstate,actualstateprob)
                
                

    def getDAStatesFromState(self,state):
        daStates={}
        strToPrint = ""
        if state in self.staDict:
            stateVals = self.staDict[state]
            for varname in self.varlist:
                import re
                pattern = re.compile(r"^da\d")
                if pattern.match(varname):
                    
                    danum = self.varlist[varname]
                    if stateVals[danum] == 1:
                        daStates[danum]=True
                    strToPrint= strToPrint+"\n"+str(danum)+":"+varname+"="+str(stateVals[danum])
        #self.printMessage(strToPrint)
        return daStates
    

    def hasDAStateChanged(self,s1,s2):
        
        state1 = self.staDict[s1]
        state2 = self.staDict[s2]
        daIndsChanged={}
        for varname in self.varlist:
            import re
            pattern = re.compile(r"^da\d")
            if(pattern.match(varname)):
                danum = self.varlist[varname]
                if(state1[danum]!=state2[danum]):
                    daIndsChanged[danum]=True
                #else:
                #    daIndsChanged[danum]=False
        return daIndsChanged
    
                

            
        
def printf(t1,t2):
    print (t1+":"+t2)
                    
            

def test():
    baseDir = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/gridIncWithRepeats/results/logs/debugRes/"
    dir = baseDir+"extras/"
    fnstastapu = "r10_g10_a1_grid_5_fsp_10_2___r4g_[7,_9,_0,_1]_finalJPvi.sta"
    fnstassi = "ssir4_g5d0r10_g10_a1_grid_5_fsp_10_2__r[0, 4, 5, 9]g_[7, 9, 0, 1]_vijp.sta"
    fntrastapu = "r10_g10_a1_grid_5_fsp_10_2___r4g_[7,_9,_0,_1]_finalJPvi.tra"
    fntrassi = "ssir4_g5d0r10_g10_a1_grid_5_fsp_10_2__r[0, 4, 5, 9]g_[7, 9, 0, 1]_vijp.tra"
    fnsta = dir + fnstastapu
    fntra = dir + fntrastapu
    stapuobj = ReadMDPStaTra(fnsta,fntra)
    stapuobj.readsta()
    stapuobj.readtra()
    #stapuobj.breakJointAction("r0_s20_s15_r1_switch_1-2_r2_switch_2-3_r3_*")
    stapuobj.testBreakJointAction()
    fnsta = dir + fnstassi
    fntra = dir + fntrassi
    ssiobj = ReadMDPStaTra(fnsta,fntra)
    ssiobj.readsta()
    ssiobj.readtra()
    ssiobj.testBreakJointAction()
    #ssiobj.breakJointAction("s23_s22,*,*,s18_s19")
    fnsta = baseDir + "r10_g10_a1_grid_5_fsp_10_2_0.sta"
    fntra = baseDir + "r10_g10_a1_grid_5_fsp_10_2_0.tra"
    mdpobj = ReadMDPStaTra(fnsta,fntra)
    mdpobj.readsta()
    mdpobj.readtra()

    state = 0
    #get action
    action = stapuobj.getAction(state)
    printf ("action",action)
    #for stapu get the agent statesfrom the firststate
    agentStates = stapuobj.getAgentStatesFromState(state)
    printf ("agent states",str(agentStates))
    #then get the action forthe first state
    brokenaction = stapuobj.breakJointAction(action)
    #then get each agent's action from the first state
    nextStates = {}
    for agnum in agentStates:
        agaction = brokenaction[agnum]
        agstate = agentStates[agnum]
        if 'switch' in agaction or agaction == '*':
            nextStates[agnum] = agstate
            continue
        
        agstate = mdpobj.findStateNum(agstate)
        if agstate != -1:
            nextstatenum = mdpobj.simulateAction(agstate,agaction)
            nextstate = mdpobj.getStateVal(nextstatenum)
            nextStates[agnum]=nextstate
        

    printf("next states",str(nextStates))
    ns = stapuobj.createStateFromAgentStates(state,nextStates)
    printf("js",str(ns))
        
    #then simulate that action
    #get the agents next state
    #create a new joint state #BC
    
    #readMDPStaTra.readsta()
    #readMDPStaTra.readtra()
    #readMDPStaTra.doMostProbablePath()
    #readMDPStaTra.doAllPaths()
    #readMDPStaTra.printMessage(readMDPStaTra.getMostProbableStateReactive(0))
    #readMDPStaTra.getAgentStatesFromState(0)

    
if __name__=="__main__":
    test()
    
