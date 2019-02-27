def breakpoint():
    import pdb
    pdb.set_trace()
    
class prob_tran:
    def __init__(self):
        #self.ps = None
        self.cs = None
        self.prob = None

    def __str__(self):
        return self.cs+":"+self.prob

    def __repr__(self):
        return self.cs+":"+self.prob


class action:
    def __init__(self):
        self.prob_trans = None
        self.name = None

    def __str__(self):
        if self.name is not None:
            strs = self.name+": ["
        else:
            strs = "["
        for pt in self.prob_trans:
            strs = strs + str(pt)+ " "
        strs = strs + "]"
        return strs

    def __repr__(self):
        if self.name is not None:
            strs = self.name+": ["
        else:
            strs = "["
        for pt in self.prob_trans:
            strs = strs +str(pt)+ " "
        strs = strs + "]"
        return strs

class policy_manip:

    def __init__(self,folder,probName):
        print "initializing policy_manip"
        self.fileExts = {'actions': '.actions', 'tra':'.tra','states':'.sta','dot':'.dot','accStates':'.acc', 'essentialStates':'.ess'}
        self.fileSuffs = {'initialStrat':'_initialStrat', 'computeUntil':'_computeUntilProbsStrat','mdp':'_mdp','jointPolicy':'_newjointPolicy','teamMDP':'_teamMDP','teamMDPTemplate':'_teamMDPTemplate','teamMDPWithSwitches':'_teamMDPWithSwitches','statra':'sta_tra'};
        self.folder=folder
        self.probName = probName
        self.sep = "_"

    def printLineMarker(self):
        print "============================================================================================================="
        
    def read_teamMDP_sta_tra(self):
        self.read_teamMDP_sta_tra_states()
        self.read_teamMDP_sta_tra_transitions()
        self.read_initialStrat_trans()
        self.read_initialStrat_actions()
        if "three_robot_one_door" in pm.probName:
            self.trace_from_expanded_state('0,0,0,0,2,0,-1',True)
            self.trace_from_expanded_state('0,0,0,0,1,0,0',True)
            self.trace_from_expanded_state('1,0,0,0,1,5,0',True)
            self.printLineMarker()
            self.trace_from_state('128',False)
            self.printLineMarker()
            self.trace_from_state('128',True)
            self.trace_from_state('844',True)
            self.trace_from_state('1584',True)
            self.printLineMarker()
            self.trace_from_state('80',True)
            self.trace_from_state('853',True)
            self.trace_from_state('1566',True)

        elif "two_robot_no_door" in pm.probName:
            #self.trace_from_state('128',most_likely=True)
            self.trace_from_state('84',most_likely=True)
            self.trace_from_state('8',most_likely=True)
            self.trace_from_state('14',most_likely=True)
            self.trace_from_state('90',most_likely=True)
            self.trace_from_expanded_state('1,0,0,1,2',most_likely=True)
        


    def trace_from_expanded_state(self,state,most_likely):
        print "In expanded state"
        for key in self.states:
            if state in self.states[key]:
                self.trace_from_state(key,most_likely)
                break
            
                
    def trace_from_state(self,state,most_likely):
        #begin in the state
        if not most_likely:
            print "not implemented"
            totProb = 1.0
            print "Tracing tree from state "+state
            from collections import deque
            from anytree import Node, RenderTree
            
            stateQ = deque()
            nodeQ = deque()
            stateQ.append(state)
            
            rootT = Node(state,sta=self.states[state])
            parentT=rootT
            nodeQ.append(parentT)
            while not len(stateQ)<1:
                state = stateQ.popleft()
                parentT=nodeQ.popleft()
                if state in self.transitions:
                    act = self.transitions[state]
                    #strs = "("+state+self.states[state]+","+act.name+",";
                    for pt in act.prob_trans:
                        stateQ.append(pt.cs)
                        childT = Node(pt.cs,parent=parentT,prob=pt.prob,action=act.name,sta=self.states[pt.cs])
                        nodeQ.append(childT)
            print RenderTree(rootT)
            #def nodenamefunc(node):
            #    return '%s\n%s' % (node.name, node.sta)
            #def edgeattrfunc(node, child):
            #    return 'label="%s:%s"' % (child.action,child.prob)
            #from anytree.exporter import DotExporter
            #dottext = ""
            #DotExporter(rootT, graph="graph",
            #                        nodenamefunc=nodenamefunc,
            #                        edgeattrfunc=edgeattrfunc).to_dotfile("sample1.dot")
            
                
            
        else:
            totProb = 1.0
            print "Tracing most likely actions from state "+state
            while state in self.transitions:
                act = self.transitions[state]
                strs = "("+state+self.states[state]+","+act.name+",";
                hp = 0.0
                ns = ""
                #print act.prob_trans
                for pt in act.prob_trans:
                    if (float(pt.prob) > hp):
                        ns = pt.cs
                        hp = float(pt.prob)
                state = ns
                strs = strs + state +self.states[state]+ ")"+str(hp)+"\n"
                totProb = totProb*hp
                print strs
            print totProb
        
        
    def read_teamMDP_sta_tra_states(self):
        #reading team mdp with switches states and transitions
        
        fn= self.probName+self.fileSuffs['teamMDPWithSwitches']+self.sep+self.fileSuffs['statra']+self.fileExts['states']
        if self.folder is not None and self.folder is not "":
            fn = self.folder+"/"+fn
        print "reading "+fn
        
        self.states = self.read_states_file(fn)
        #if self.states is not None:
        #    print self.states
        
    def read_states_file(self,fn):
        states = None
        with open(fn,"r") as f:
            #skip the first line
            lines = f.readlines()
            states={}
            for i in range(1,len(lines)):
                lines[i] = lines[i].replace("\n","")
                line_split = lines[i].split(':')
                states[line_split[0]] = line_split[1]
            self.states_labels = ((lines[0].replace("(","")).replace(")","")).split(",")
        return states

    def read_initialStrat_trans(self):
    
        fn= self.probName+self.fileSuffs['initialStrat']+self.fileExts['tra']
        if self.folder is not None and self.folder is not "":
            fn = self.folder+"/"+fn
        print "reading "+fn
        
        self.transitions = self.read_transitions_file(fn)
        #if self.transitions is not None:
        #    print self.transitions

    def read_initialStrat_actions(self):
        fn= self.probName+self.fileSuffs['initialStrat']+self.fileExts['actions']
        if self.folder is not None and self.folder is not "":
            fn = self.folder+"/"+fn
        print "reading "+fn
        
        self.read_transition_actions(fn)
        #if self.transitions is not None:
        #    print self.transitions

    def read_transition_actions(self,fn):
        with open(fn,"r") as f:
            lines = f.readlines()
            if self.transitions is not None:
                if self.transitions_type == 2:
                    for i in range(0,len(lines)):
                        line = lines[i].replace("\n","")
                        split_line = line.split(":")
                        if split_line[0] in self.transitions:
                            self.transitions[split_line[0]].name = split_line[1]
                else:
                    print "wrong transitions loaded"
            else:
                print "empty transitions, load transitions first"
                        
        return self.transitions
        
    def read_teamMDP_sta_tra_transitions(self):
    
        fn= self.probName+self.fileSuffs['teamMDPWithSwitches']+self.sep+self.fileSuffs['statra']+self.fileExts['tra']
        if self.folder is not None and self.folder is not "":
            fn = self.folder+"/"+fn
        print "reading "+fn
        self.transitions = self.read_transitions_file(fn)
        #if self.transitions is not None:
        #    print self.transitions
        

    def read_transitions_file(self,fn):
        #checking type
        transitions = None
        self.transitions_type = None
        with open(fn,"r") as f:
            lines = f.readlines()
            #read the first line
            #to determine type
            lines[0] = lines[0].replace("\n","")
            fl = lines[0].split(" ")
            print fl
            
            if len(fl) == 2:
                self.transitions_type=2
            else:
                self.transitions_type=3
            transitions = {}
            print self.transitions_type
            raw_input("pak")
            if self.transitions_type == 2:
                for i in range(1,len(lines)):
                    lines[i] = lines[i].replace("\n","")
                    split_line = lines[i].split(" ")
                    if split_line[0] not in transitions:
                        act = action()
                        act.prob_trans=[]
                        transitions[split_line[0]] = act
                    act = transitions[split_line[0]]
                    prob_t = prob_tran()
                    prob_t.cs = split_line[1]
                    prob_t.prob=split_line[2]
                    act.prob_trans.append(prob_t)
                    transitions[split_line[0]] = act
            else:
                tran_num = None
                sta_num = None
                for i in range(1,len(lines)):
                    lines[i] = lines[i].replace("\n","")
                    split_line = lines[i].split(" ")
                    #print split_line
                    #raw_input("pak1")
                    #if split_line[0] == '108':
                    #    breakpoint()
                    if tran_num is None:
                        tran_num = split_line[1]
                        sta_num = split_line[0]
                        
                    if split_line[0] not in transitions:
                        act = action()
                        act.prob_trans=[]
                        if (len(split_line) > 4):
                            act.name = split_line[4]
                        transitions[split_line[0]] = []
                        transitions[split_line[0]].append(act)
                        sta_num = split_line[0]
                        tran_num = split_line[1]
                        
                    if sta_num == split_line[0]:
                        if tran_num == split_line[1]:
                            act = transitions[split_line[0]][int(tran_num)]
                        else:
                            act = action()
                            act.prob_trans=[]
                            if (len(split_line) > 4):
                                act.name = split_line[4]
                            transitions[split_line[0]].append(act)
                            tran_num = split_line[1]
                    else:
                        sta_num = split_line[0]
                    #print tran_num
                    #print sta_num
                    #print split_line
                    #raw_input("pak2")
                    act = transitions[split_line[0]][int(split_line[1])]
                    prob_t = prob_tran()
                    prob_t.cs = split_line[2]
                    prob_t.prob=split_line[3]
                    #print split_line
                    #print prob_t.cs
                    #print prob_t.prob
                    act.prob_trans.append(prob_t)
                    #print act
                    #raw_input("pak3")
                    transitions[split_line[0]][int(split_line[1])] = act
                
                    
                        
        return transitions    

if __name__=="__main__":
    
    #pm = policy_manip("","two_robot_no_door")#"three_robot_one_door")
    pm = policy_manip("","three_robot_one_door")
    pm.read_teamMDP_sta_tra()
