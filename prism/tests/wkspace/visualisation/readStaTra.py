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
        print ("Reading sta file: "+str(self.stafilename))
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
                #print ("Processed var names: "+str(self.varlist))
            else:
                pline = line.split(':')
                statenum = int(pline[0])
                statevals = eval(pline[1])
                staDict[statenum]=statevals
        self.staDict = staDict
        #print("Processed state vals")
        #print (staDict)
        print ("Read sta file")
        print ("Variables "+str(self.varlist.keys()))
        print ("Number of states "+str(len(self.staDict)))

    def readtra(self):
        #The first line of the file take the form "n c m", giving the number of states (n), the total number of choices (c) and the total number of transitions (m). The remaining lines are of the form "i k j x" or "i k j x a", where i and j are the row (source) and column (destination) indices of the transition, k is the index of the choice that it belongs to, and x is the probability of the transition. a is optional and gives the action label for the choice of the transition. Action labels can be present for some, all or no states but, in slightly redundant fashion, the action labels, if present, must be the same for all transitions belonging to the same choice.
        
        print ("Reading tra file: "+str(self.trafilename))
        lines = self.getLines(self.trafilename)
        print (len(lines))
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
                print ("states,choices,transitions : "+str(numstates)+","+str(numchoices)+","+str(numtrans))
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
                #print(traDict)
                #raw_input("continue")
        self.traDict = traDict

    def setToReactiveMode():
        self.doPolicyReactive = True
        
    def turnoffReactiveMode():
        self.doPolicyReactive = False
        
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
                print ("choice "+str(choiceNum)+" for "+str(state)+" not in transitions list")
                        
                
        else:
            print (str(state)+" not in transitions list")
        return None
            
    def doMostProbablePath(self,startState=0):
        if self.traDict is not None:
            if self.staDict is not None:
                q = [startState]
                while len(q) > 0:
                    currstate = q.pop(0)
                    #start with a state
                    print(currstate)
                    if currstate not in self.traDict:
                        print (str(currstate)+" not in transitions list")
                        continue 
                    choicesDict = self.traDict[currstate]
                    choiceNum = 0
                    if choiceNum not in choicesDict:
                        print ("choice "+choiceNum+" for "+str(currstate)+" not in transitions list")
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
                    print("List for "+str(currentstate))
                    q = [currentstate]
                    while(len(q)>0):
                        currstate = q.pop(0)
                        print(currstate)
                        if currstate not in self.traDict:
                            print (str(currstate)+" not in transitions list")
                            continue 
                        choicesDict = self.traDict[currstate]
                        choiceNum = 0
                        if choiceNum not in choicesDict:
                            print ("choice "+choiceNum+" for "+str(currstate)+" not in transitions list")
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
        print(agentStates)
        return (agentStates)

    def getDAStatesFromState(self,state):
        daStates={}
        if state in self.staDict:
            stateVals = self.staDict[state]
            for varname in self.varlist:
                import re
                pattern = re.compile(r"^da\d")
                if pattern.match(varname):
                    danum = self.varlist[varname]
                    if stateVals[danum] == 1:
                        daStates[danum]=True
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
    
                

            
        
        
                    
            

def test():
    dir = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/smallerwhdoors/results/logs/debugRes/extras/"
    fnstastapu = "smallerShelfDepot_r10_g10_a1_fs79_fsp_90_6_d_1___finalJPvi.sta"
    fnstassi = "ssir4_g5d1smallerShelfDepot_r10_g10_a1_fs79_fsp_90_6_d_1__vijp.sta"
    fntrastapu = "smallerShelfDepot_r10_g10_a1_fs79_fsp_90_6_d_1___finalJPvi.tra"
    fntrassi = "ssir4_g5d1smallerShelfDepot_r10_g10_a1_fs79_fsp_90_6_d_1__vijp.tra"
    fnsta = dir + fnstastapu
    fntra = dir + fntrastapu
    readMDPStaTra = ReadMDPStaTra(fnsta,fntra)
    readMDPStaTra.readsta()
    readMDPStaTra.readtra()
    readMDPStaTra.doMostProbablePath()
    readMDPStaTra.doAllPaths()
    print(readMDPStaTra.getMostProbableStateReactive(0))
    readMDPStaTra.getAgentStatesFromState(0)
    #propdir = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/smallerwhdoors/"
    #propfn = "smallerShelfDepot_r10_g10_a1_fs79_fsp_90_0_d_1_.prop"
    #readMDPStaTra.readPropsFile(propdir+propfn)
    
if __name__=="__main__":
    test()
    
