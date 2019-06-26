from PrismVariable import PrismVariable
from PrismVariable import VariableType
from RegexHelper import RegexHelper

class PrismAction(object):
    name = None
    src = None
    dest = None
    rewards={}
    basicActionRE = "\[(.*)\] (.*) -> (.*);"
    
    def __init__(self,line,constants,variables):
        #print line
        actInfo = RegexHelper.getRegexMatchName(self.basicActionRE,line)
        self.name = actInfo[0]
        #print self.name
        #split on and
        source = RegexHelper.processState(actInfo[1],constants,variables)
        #print source
        destination = actInfo[2]
        dest = self.processDestination(destination,constants,variables)
        #print dest
        self.src = source
        self.dest = dest
        
        #print actInfo[1]
        #print actInfo[2]

    def processDestination(self,dest,constants,variables):
        dest = dest.split('+')
        dests = []
        for d in dest:
            dests.append(self.processProbDest(d,constants,variables))
        return dests
            
    def processProbDest(self,dest,constants,variables):
        pdest = dest.split(':')
        if len(pdest) == 1:
            return {"prob":1,"states":RegexHelper.processState(dest,constants,variables)}
        else:
            #first bit is the probability
            #strip any brackets
            prob = pdest[0]
            dest = pdest[1]
            dest = RegexHelper.processState(dest,constants,variables)
            #prob = prob.replace('(','')
            #prob = prob.replace(')','')
            #lets just save this as that
            return {"prob":prob,"states":dest}
            
            

    def matchSrc(self,src):
        matched = True 
        if(len(self.src)==len(src)):
            for i in range(len(src)):
                s = src[i]
                ss = self.src[i]
                if not s.equals(ss):
                    matched = False
                    break
        else:
            matched = False
        return matched 
                    
            

    def equals(self,name,src):
        if self.name == name:
            if self.matchSrc:
                return True 
        return False

    def addReward(self,name,value):
        self.rewards[name]=value
        
    def __str__(self):
        return self.name+str(self.src)+str(self.dest)

    def __repr__(self):
        return self.__str__()

    def prismStringAction(self):
        toret = "["+self.name + "] "
        toret = toret + self.stateString(self.src,False)
        toret = toret + " -> " +self.destString(self.dest) + ";"
        
        
        return toret 

    def destString(self,dest):
        dests = [] 
        for d in dest:
            p = d["prob"]
            src = d["states"]
            #print src
            #print dest 
            #src = dest[d]
            toret = str(p) + ":" + self.stateString(src,True)
            dests.append(toret)
        toret = ""
        for i in range(len(dests)):
            toret = toret + dests[i]
            if i != len(dests)-1:
                toret = toret + " + "
        return toret
    
    def stateString(self,src,isDest):
        app = '&'
        toret = ""
        for i in range(len(src)):
            var = src[i]
            toret = toret + var.stateString(isDest)
            if i != (len(src)-1):
                toret = toret + " & "
        return toret
    
        
    def prismStringReward(self,rewname):
        toret = "["+self.name + "] "
        toret = toret + self.stateString(self.src,False)
        toret = toret + " : " + str(self.rewards[rewname]) + ";"
        
        return toret
    
