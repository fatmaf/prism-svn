import re
from enum import Enum
from PrismVariable import PrismVariable
from PrismVariable import VariableType

class RegexHelper:
    
    @staticmethod
    def isRegexMatch(pattern,string):
        printThis = False
        string = string.lstrip()
        #if "const" in string:
        #    printThis = True 
        #    print (pattern)
        #    print (string)
        #    for i in range(len(string)):
        #        print string[i]
            
        p = re.compile(pattern,re.IGNORECASE)

        result = p.match(string)
        
        if result:
            return True
        else:
            return False

    @staticmethod
    def getRegexMatchName(pattern,string):
        string = string.lstrip()
        
        toret = None
        p = re.compile(pattern,re.IGNORECASE)
        result = p.match(string)
        if result:
            #get the first group
            groups = result.groups()
            if len(groups) > 0:
                if len(groups) == 1:
                    toret = groups[0]
                else:
                    toret = groups
        if toret is None:
            #trying the other regex
            raise Exception('No regex match name:'+pattern+" "+string)
        return toret
    
    
    @staticmethod
    def processState(state,constants,variables):
        states = state.split('&')
        stateRE = "\((\w*)([=,<,>]*)(\w*|\w*[+,-]\w*)\)" #"\((.*)=(.*)\)"
        varValues = [] 
        for nv in states:
            nameValuePair = RegexHelper.getRegexMatchName(stateRE,nv)
            name = nameValuePair[0]
            name = name.replace("'",'')
            operator = nameValuePair[1]
            value = nameValuePair[2]
            if value in constants:
                value = constants[value]
            #print name
            #print value
            if name in variables:
                var = PrismVariable()
                var.createUsingOtherPrismVariable(variables[name],value)
            else:
                if name in constants:
                    var = PrismVariable()
                    var.createUsingOtherPrismVariable(constants[name],value)
                else:
                    #print states
                    var = PrismVariable()
                    var.create(name,value,None,None,None,VariableType.notConst,"double")
            var.operator = operator
            varValues.append(var)
        return varValues

    
 
