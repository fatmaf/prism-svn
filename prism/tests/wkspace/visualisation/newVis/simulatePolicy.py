from readStaTra import ReadMDPStaTra

class SimulatePolicy(object):
    def __init__(self,fnpol,fnmdp):
        self.fnpol = fnpol
        self.fnmdp = fnmdp
        self.getAllMDPs()
        

    def getAllMDPs(self):
        if self.fnpol is not None:
            self.polobj = self.getPolicyObj(self.fnpol)
        self.mdp = self.getPolicyObj(self.fnmdp)

    def getPolicyObj(self,fn):
        fnsta = fn+'.sta'
        fntra = fn + '.tra'
        policyObj = ReadMDPStaTra(fnsta,fntra)
        policyObj.readsta()
        policyObj.readtra()
        return policyObj

    def getnextstatesMDP(self,agentStates,brokenaction,doorStates):
        nextStates = {}
        mdp = self.mdp
        combp = 1.0
        for agnum in agentStates:
            agaction=brokenaction[agnum]
            agstate = agentStates[agnum]
            if 'switch' in agaction or agaction=='*':
                nextStates[agnum]=agstate
            else:
                    
                agstate = mdp.findStateNum(agstate,doorStates)
                if agstate != -1:
                    (nextstatenum,nsp) = mdp.simulateAction(agstate,agaction)
                    nextstate = mdp.getStateVal(nextstatenum)
                    nextStates[agnum] = nextstate
                    combp = combp*nsp
                    print ("State prob for "+str(nextstatenum)+" "+str(nsp))
                    

        return (nextStates,combp)
    

    def getNextState(self,state,polobj=None):
        ns = None
        if state is None:
            return ns
        
        if polobj is None:
            polobj = self.polobj
        print(state)
        action = polobj.getAction(state)
        if action is not None:
            print (action)
            agentStates = polobj.getAgentStatesFromState(state)
            brokenaction = polobj.breakJointAction(action)
            doorStates = None
            if self.mdp.hasDoors:
                doorStates = polobj.getDoorStatesFromState(state)
            (nextstates,combp) = self.getnextstatesMDP(agentStates,brokenaction,doorStates)
            (ns,nsp) = polobj.createStateFromAgentStates(state,nextstates)
            self.polobj.printMessage(ns)
            if(combp != nsp):
                self.polobj.printMessage ("Unexpected probabilities: Expected "+str(combp)+ " got "+str(nsp))
                self.polobj.printMessage (" Expected state "+str(nextstates)+ " got "+str(ns))
        return ns
        
    def simulatePolicy(self,polobj=None):
        if polobj is None:
            polobj = self.polobj
            if polobj is None:
                return 
        stateq = [0]
        while len(stateq) != 0:
            state = stateq.pop()
            ns = self.getNextState(state,polobj)
            if ns is not None:
                stateq.append(ns)
                
        
    
def test():
    baseDir = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/gridIncWithRepeats/results/logs/debugRes/"
    dir = baseDir+"extras/"
    fnstastapu = "r10_g10_a1_grid_5_fsp_10_2___r4g_[7,_9,_0,_1]_finalJPvi"
    fnstassi = "ssir4_g5d0r10_g10_a1_grid_5_fsp_10_2__r[0, 4, 5, 9]g_[7, 9, 0, 1]_vijp"
    fntrastapu = "r10_g10_a1_grid_5_fsp_10_2___r4g_[7,_9,_0,_1]_finalJPvi.tra"
    fntrassi = "ssir4_g5d0r10_g10_a1_grid_5_fsp_10_2__r[0, 4, 5, 9]g_[7, 9, 0, 1]_vijp.tra"
    fnstapu = dir + fnstastapu
    fnssi = dir + fnstassi
    fnmdp = baseDir + "r10_g10_a1_grid_5_fsp_10_2_0"
    sp = SimulatePolicy(fnstapu,fnmdp)
    sp.simulatePolicy()
    sp = SimulatePolicy(fnssi,fnmdp)
    sp.simulatePolicy(sp.polobj)
    


    
if __name__=="__main__":
    test()
