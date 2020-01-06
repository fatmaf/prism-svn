from PrismVariable import PrismVariable
from PrismVariable import VariableType
from RegexHelper import RegexHelper

class PrismAction(object):
    #name = None
    #src = None # a list of states and values - a list of PrismVariables 
    #dest = None # a list of dictionaries - each dictionary has a prob and states
    ############# where states is a list of lists like in src - of PrismVariables
    #rewards={}
    basicActionRE = "\[(.*)\] (.*) -> (.*);"
    
    def __init__(self,line,constants,variables,createFromText=True):
        self.name = None
        self.src = None
        self.dest = None
        self.rewards={}
    
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
        #print "Adding state to destination"
        #print pvar
        #print prob
        
        self.dest.append(self.createDestinationDict(pvar,prob))

    def updateWithFailureState(self,fvar,fprob,sprob):
        #fprob = failure probability
        #sprob = success probability
        #only if there is a single destination
        if(len(self.dest))==1:
            self.dest[0]["prob"] = sprob
            self.dest.append(self.createDestinationDict(fvar,fprob))

        
    def createDestinationDict(self,pvar,prob):
        #print "Creating destination dict"
        #print pvar
        #print prob 
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
            
            

    def getVarValueSrc(self,varname,tm=None):
        if tm is None:
            tm = self.src
        for i in range(len(tm)):
            if tm[i].name == varname:
                return tm[i].value
        return None
    def getVarValueDest(self,varname):
        toret = None
        for d in self.dest:
            src = d["states"]
            toret = self.getVarValueSrc(varname,src)
            if not toret is None:
                break
        
        return toret
    
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
    
    def matchAnyoneSrc(self,src,tm=None):
        if tm is None:
            tm = self.src
        matched = False
        for s in tm:
            for ss in src:
                if s.equals(ss):
                    matched = True
                    break
            if matched:
                break
        return matched

    def matchAnyoneDest(self,src):
        matched = False
        for des in self.dest:
            d = des["states"]
            if self.matchAnyoneSrc(src,d):
                matched = True
                break
        return matched
    
    def matchAnyoneSrcOrDestDest(self,dest):
        matched = False 
        for d in dest:
            s = d["states"]
            if self.matchAnyoneSrc(s):
                matched = True
                break
            if self.matchAnyoneDest(s):
                matched = True
                break
        return matched 
        
    def matchAnyoneSrcOrDest(self,src):
        if not self.matchAnyoneSrc(src):
            return self.matchAnyoneDest(src)
        return True 

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
        #print "In destination string"
        #print dest 
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
        #print "Destination string"
        #print toret
        
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
        toret = toret + " : "
        rewValue = self.rewards[rewname]
        if type(rewValue) is str:
            rewValueS = rewValue
        elif type(rewValue) is float:
            rewValueS = str(rewValue)
        elif type(rewValue) is PrismVariable:
            rewValueS = rewValue.name
        else:
            rewValueS = str(rewValue)
            
        #rewValue = str(self.rewards[rewname]) 
        toret = toret + rewValueS + ";"
        
        return toret
    
