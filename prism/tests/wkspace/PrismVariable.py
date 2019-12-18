from enum import Enum


class VariableType(Enum):
    const = 1
    notConst = 2
    
class PrismVariable(object):

    def __init__(self):
        self.name = None
        self.value = None
        self.minValue = None
        self.maxValue = None
        self.initValue = None
        self.varType = None
        self.subType = None
        self.operator = '='

        

    def getAssignment(self,v,isInt):
        value = None
        #print(isinstance(v,PrismVariable))
        if type(v) is PrismVariable:
            value = v
        else:
            if v is not None:
                if isInt:
                    value = int(v)
                else:
                    value = float(v)
        return value
                    

        
    def create(self,n,v,minv,maxv,initv,t,subT):
        #print n
        #print v
        #print minv
        #print maxv
        #print initv
        #print t
        
        self.name = n
        self.subType = subT
        self.varType = t
        
        isInt = False
        if(subT == "int"):
            isInt = True

        #print minv
        #print type(minv)
        #assign v
        self.value = self.getAssignment(v,isInt)
        
        #assign initv
        self.initValue = self.getAssignment(initv,isInt)

        #assign minv
        self.minValue = self.getAssignment(minv,isInt)

        self.maxValue = self.getAssignment(maxv,isInt)


    def createUsingOtherPrismVariable(self,var,v):
        if type(var) is PrismVariable:
            self.assign(var)
            self.value = v 

    def stateString(self,isDest):
        toret = '('+self.name;
        if isDest:
            toret = toret + "'"
        toret = toret + self.operator+self.getStringRep(self.value)+")"
        return toret
    
        
    def prismString(self):
        if self.varType is VariableType.const:
            if self.subType is None:
                return "const "+self.name+" = "+self.getStringRep(self.value)+";"
            else:
                return "const "+self.subType+" "+self.name+" = "+self.getStringRep(self.value)+";"
        else:
            toret = self.name+":[";
            toret = toret + self.getStringRep(self.minValue);
            toret = toret + " .. " + self.getStringRep(self.maxValue)+"]"
            toret = toret + " init "+self.getStringRep(self.initValue)+";"
            return toret
        

    def getStringRep(self,var):
        if type(var) is str:
            return var
        elif type(var) is float:
            return str(var)
        elif type(var) is int:
            return str(var)
        elif type(var) is PrismVariable:
            
            return self.getStringRep(var.name)
        return ""


    def getRangeValues(self):
        r = []
        if type(self.minValue) is PrismVariable:
            r.append(self.minValue.value)
        else:
            r.append(self.minValue)
        if type(self.maxValue) is PrismVariable:
            r.append(self.maxValue.value)
        else:
            r.append(self.maxValue)
        return r 
            
            
    def assign(self,var):
        if type(var) is PrismVariable:
            self.name = var.name
            self.initValue = var.initValue
            self.minValue = var.minValue
            self.maxValue = var.maxValue
            self.value = var.value
            self.varType = var.varType
            self.subType = var.subType

            
    def equals(self,var,justName=False):
        if type(var) is PrismVariable:
            if (self.name == var.name):
                if justName:
                    return True
                else:
                    #print self.value
                    #print var.value 
                    if(self.value == var.value):
                        return True
                    else:
                        return False 
        return False
    
    def __repr__(self):
        return self.__str__()
    

    def __str__(self):
        
        #if type(self.value) == PrismVariable:
        #    return self.name+":"+str(self.value)
        
        if self.varType == VariableType.const:
            return self.name + ":"+self.getStringRep(self.value)+"("+self.getStringRep(self.initValue)+ ")"
        else:
            return self.getStringRep(self.name) + ":"+self.getStringRep(self.value)+"("+ self.getStringRep(self.initValue)+ ") range:["+self.getStringRep(self.minValue)+","+self.getStringRep(self.maxValue)+"]"
                
