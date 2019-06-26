from PrismVariable import PrismVariable
from PrismVariable import VariableType
from RegexHelper import RegexHelper

class PrismAction(object):
    name = None
    src = None # a list of states and values - a list of PrismVariables 
    dest = None # a list of dictionaries - each dictionary has a prob and states
    ############# where states is a list of lists like in src - of PrismVariables
    rewards={}
    basicActionRE = "\[(.*)\] (.*) -> (.*);"
    
    def __init__(self,line,constants,variables,createFromText=True):
        if createFromText:
            actInfo = RegexHelper.getRegexMatchName(self.basicActionRE,line)
            self.name = actInfo[0]
            #split on and
            source = RegexHelper.processState(actInfo[1],constants,variables)
            destination = actInfo[2]
            dest = self.processDestination(destination,constants,variables)
            self.src = source
            self.dest = dest
        else:
            #dont create it from text
            #just do nothing
            self.name = ""
            self.src = []
            self.dest = [] 
        
    def addStateToSource(self,pvar):
        self.src.append(pvar)

    def addStateToDestination(self,pvar,prob):
        self.dest.append(self.createDestinationDict(pvar,prob))

    def updateWithFailureState(self,fvar,fprob,sprob):
        #fprob = failure probability
        #sprob = success probability
        #only if there is a single destination
        if(len(self.dest))==1:
            self.dest[0]["prob"] = sprob
            self.dest.append(self.createDestinationDict(fvar,fprob))

        
    def createDestinationDict(self,pvar,prob):
        if type(pvar) is not list:
            pvar = [pvar]
        return {"prob":prob,"states":pvar}
    

    def processDestination(self,dest,constants,variables):
        dest = dest.split('+')
        dests = []
        for d in dest:
            dests.append(self.processProbDest(d,constants,variables))
        return dests
            
    def processProbDest(self,dest,constants,variables):
        pdest = dest.split(':')
        if len(pdest) == 1:
            return self.createDestinationDict(RegexHelper.processState(dest,constants,variables),1.0)
        else:
            #first bit is the probability
            #strip any brackets
            prob = pdest[0]
            dest = pdest[1]
            dest = RegexHelper.processState(dest,constants,variables)
            #lets just save this as that
            return self.createDestinationDict(dest,prob)
            
            

    def matchSrc(self,src,tm=None):
        if tm is None:
            tm = self.src
        matched = True 
        if(len(tm)==len(src)):
            for i in range(len(src)):
                s = src[i]
                ss = tm[i]
                if not s.equals(ss):
                    matched = False
                    break
        else:
            matched = False
        return matched 
                    
            

    def matchOnlyOneSrc(self,src,tm=None):
        if tm is None:
            tm = self.src
        matched = False
        for s in tm:
            if s.equals(src):
                matched = True
                break
        return matched

    def matchOnlyOneDest(self,d):
        matched = False
        for des in self.dest:
            src = des["states"]
            if self.matchOnlyOneSrc(d,src):
                matched=True
                break
        return matched

    def matchOnlyOneSrcOrDest(self,src):
        if not self.matchOnlyOneSrc(src):
            return self.matchOnlyOneDest(src)
        return True 
            
    
    def equals(self,name,src):
        if self.name == name:
            if self.matchSrc(src):
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
    
