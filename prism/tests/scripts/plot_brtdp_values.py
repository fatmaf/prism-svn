import re
import matplotlib.pyplot as plt

def find_numbers(text):
    #expr =r"[-+]?\d*\.\d+|\d+"
    expr = r"^-?[0-9]+(?:\.[0-9]+)?|-?[0-9]+(?:\.[0-9]+)?"
    matches=re.findall(expr,text)
    #print (matches)
    return matches



class stateBounds():
    def __init__(self,up,lp,ur,lr,uc,lc,arr):
        if arr:
            self.upperP = up
            self.lowerP = lp
            self.upperR = ur
            self.lowerR = lr
            self.upperC = uc
            self.lowerC = lc

        else:
            self.upperP = float(up)
            self.lowerP = float(lp)
            self.upperR = float(ur)
            self.lowerR = float(lr)
            self.upperC = float(uc)
            self.lowerC = float(lc)
        self.isArr = arr

    def __str__(self):
        if self.isArr:
            return "[P:%s,%s|R:%s,%s|C:%s,%s]" % (str(self.upperP),str(self.lowerP),
                                                  str(self.upperR),str(self.lowerR),str(self.upperC),str(self.lowerC))
        else:
            return "[P:%f,%f|R:%f,%f|C:%f,%f]" % (self.upperP,self.lowerP,self.upperR,self.lowerR,self.upperC,self.lowerC)

    def __repr__(self):
        return str(self)

def extractBoundsFromDict(bounds):
    ups =[]
    lps =[]
    urs = []
    lrs = []
    ucs = []
    lcs = []
    states=[]
    for s in bounds:
        sb = bounds[s]
        ups.append(sb.upperP)
        lps.append(sb.lowerP)
        urs.append(sb.upperR)
        lrs.append(sb.lowerR)
        ucs.append(sb.upperC)
        lcs.append(sb.lowerC)
        states.append(s)
    return (states,ups,lps,urs,lrs,ucs,lcs)

def plot_bounds(bounds):
    (states,ups,lps,urs,lrs,ucs,lcs) = extractBoundsFromDict(bounds)
    plt.plot(ups)
    


def process_bounds(fn):
    numStateVars = 3
    pBounds = [3,4]
    rBounds = [5,6]
    cBounds = [7,8]
    bounds={}

    with open(fn) as f:
        for line in f.readlines():
            matches = find_numbers(line)
            if(len(matches))>8:
                state = tuple(matches[0:numStateVars])
                #print(state)
                upperP = matches[pBounds[0]]
                lowerP = matches[pBounds[1]]
                upperR = matches[rBounds[0]]
                lowerR = matches[rBounds[1]]
                upperC = matches[cBounds[0]]
                lowerC = matches[cBounds[1]]
                sb = stateBounds(upperP, lowerP, upperR, lowerR, upperC, lowerC,False)
                bounds[state]=sb
    return bounds


def process_allBounds(allBounds):
    allBs={}
    for bounds in allBounds:
        #print (bounds)
        for state in bounds:
            #print(state)
            if state in allBs:
                s = bounds[state]
                allBs[state].upperP.append(s.upperP)
                allBs[state].lowerP.append(s.lowerP)
                allBs[state].upperR.append(s.upperR)
                allBs[state].lowerR.append(s.lowerR)
                allBs[state].upperC.append(s.upperC)
                allBs[state].lowerC.append(s.lowerC)
            else:
                s = bounds[state]
                sb = stateBounds([s.upperP],[s.lowerP],[s.upperR],[s.lowerR],[s.upperC],[s.lowerC],True)
                allBs[state] = sb 
    return allBs 
    
def plot_allBsForState(allBounds, state):
    if(state in allBounds):
        b = allBounds[state]
        x = range(len(allBounds[state].upperP))
        #print(x)
        plt.figure(str(state))
        plt.subplot(131)
        #plt.subplot(121)
        up, = plt.plot(x,b.upperP,"r--o",label="uP")
        #plt.legend(handles=[up]) 
        #plt.title(str(state))
        #plt.subplot(122)
        lp, = plt.plot(x,b.lowerP,"b--o",label="lP")
        plt.legend(handles=[up,lp]) 
        plt.title(str(state))
        
        #plt.show()
        
        plt.subplot(132)
        #plt.subplot(121)
        ur, = plt.plot(x,b.upperR,"r--o",label="uR")
        #plt.legend(handles=[ur])
        #plt.title(str(state))
        #plt.subplot(122)
        lr, = plt.plot(x,b.lowerR,"b--o",label="lR")
        plt.legend(handles=[ur,lr])
        plt.title(str(state))
        
        #plt.show()
        
        plt.subplot(133)
        #plt.subplot(121)
        uc, = plt.plot(x,b.upperC,"r--o",label="uC")
        #plt.legend(handles=[uc])
        #plt.title(str(state))
        #plt.subplot(122)
        lc, = plt.plot(x,b.lowerC,"b--o",label="lC")
        plt.legend(handles=[uc,lc])
        plt.title(str(state))
        #plt.legend(handles=[up,lp,ur,lr,uc,lc])
        plt.show(block=False)

def plot_allBsForStateList(allBounds, states):
    for state in states:
        plot_allBsForState(allBounds,state)
        
def plot_allBs(allBounds):
    for state in allBounds:
        plot_allBsForState(allBounds,state)


def strictly_increasing(L):
    return all(x<y for x, y in zip(L, L[1:]))

def strictly_decreasing(L):
    return all(x>y for x, y in zip(L, L[1:]))

def non_increasing(L):
    return all(x>=y for x, y in zip(L, L[1:]))

def non_decreasing(L):
    return all(x<=y for x, y in zip(L, L[1:]))

def monotonic(L):
    return non_increasing(L) or non_decreasing(L)

def lb_higher_ub_lower(ub,lb):
    allthesame = False
    count_same = 0
    ishigher = False
    for u in ub:
        for l in lb:
            if( l >= u):
                ishigher=True
                break
    if ishigher:
        for i in range(len(ub)):
            if(ub[i]==lb[i]):
                count_same = count_same+1
        #print (count_same)
        #print (len(ub))
        #print (len(lb))
        if(len(ub) == count_same):
            ishigher=False
    return ishigher

        
def plot_errorStatesOnly(allBounds):
    for state in allBounds:
        sb = allBounds[state]
        sstr = str(state)

        mp = not lb_higher_ub_lower(sb.upperP,sb.lowerP)
        m = mp
        if mp:
            mr = not lb_higher_ub_lower(sb.upperR,sb.lowerR)
            m = m and mr
            if mr:
                mc = not lb_higher_ub_lower(sb.upperC,sb.lowerC)
                m = m and mc
                if mc:
                    mup = non_increasing(sb.upperP)
                    m = m and mup
                    if mup:
                        mlp = non_decreasing(sb.lowerP)
                        m = m and mlp
                        if mlp:
                            mlr=non_decreasing(sb.lowerR)
                            m = m and mlr
                            if mlr:
                                mur=non_increasing(sb.upperR)
                                m = m and mur
                                if mur:
                                    mlc=non_decreasing(sb.lowerC)
                                    m = m and mlc
                                    if mlc:
                                        muc=non_increasing(sb.upperC)
                                        m = m and muc
                                        if not muc:
                                            sstr = sstr + "uC not monotonic"
                                    else:
                                        sstr = sstr + "lC not monotonic"
                                else:
                                    sstr = sstr + "uR not monotonic"
                            else:
                                sstr = sstr + "lR not monotonic"
                        else:
                            sstr = sstr + "lP not monotonic"
                    else:
                        sstr = sstr + "uP not monotonic"
                else:
                    sstr = sstr + "uc lc overlap"
            else:
                sstr = sstr + "ur lr overlap"
        else:
            sstr = sstr + "up lp overlap"
        
        if not m:
            print(sstr)
            plot_allBsForState(allBounds,state)
    print('all done')
                            
        
    
#fnnum = 0
#fnmax = 9
fns=[]
allBounds = []
#for i in range(1,fnmax):
#    fn = "values_"+str(i)+".txt"
#    fns.append(fn)
def list_files():
    import os 
    for r, d, f in os.walk('../results'):
        #boundsf.sort(key=os.path.getmtime)
        for fn in f:
            if fn.startswith('values') and fn.endswith('.txt'):
            
                fns.append(r+"/"+fn)
def get_trial_number_from_fn(fn):
    #print fn
    num_t_s = fn.index('values_')
    num_t_s = num_t_s + len('values_')
    num_t_e = fn.index('-')
    num_t_str = fn[num_t_s:num_t_e]
    #print (num_t_str)
    num_t = int(num_t_str)
    return num_t
def get_iteration_number_from_fn(fn,num_t):
    if('values_'+str(num_t)+'-0' in fn):
        return 0
    num_i_s = fn.index('values_'+str(num_t)+'-')
    num_i_s = num_i_s+len('values_'+str(num_t)+'-')
    num_i_e = fn.index('_',num_i_s)
    num_i_str = fn[num_i_s:num_i_e]
    num_i = int(num_i_str)
    #print(num_i)
    return num_i

    
def sort_files_properly(filesl):
    #so we start with num_trials
    # -0 is the first one
    # -x_1-3 are the rest
    num_trials = 0
    num_files_in_trials ={}
    files_in_trials = {}
    trial_iterations={}
    first_trial = -1
    prefix = None
    for fn in filesl:
        if prefix == None:
            prefix = fn[0:fn.index('values_')]
        num_t = get_trial_number_from_fn(fn)
        if(num_t > num_trials):
            num_trials = num_t
        if first_trial < 0:
            first_trial=num_t
        if num_t < first_trial:
            first_trial=num_t
            
        if num_t not in num_files_in_trials:
            num_files_in_trials[num_t] = 1
            files_in_trials[num_t] = [fn]
        
        else:
            num_files_in_trials[num_t]=num_files_in_trials[num_t]+1
            files_in_trials[num_t].append(fn)
    
    for i in range(first_trial,num_trials+1):
        trial_files = files_in_trials[i]
        #lets get the number of iterations
        start_iter = -1
        last_iter = 0
        for fn in trial_files:
            fit=get_iteration_number_from_fn(fn,i)
            if(start_iter) < 0:
                start_iter = fit
            if (fit < start_iter):
                start_iter = fit
            if (fit > last_iter):
                last_iter = fit
        trial_iterations[i]=[start_iter,last_iter]
        
    #print trial_iterations
    sorted_files = []

    for i in range(first_trial,num_trials+1):
        #fn format
        #prefix+'values_'+str(num_t)+'-0' or
        #prefix+'values_'+str(num_t)+'-'+str(num_i)+'_'[0-3]
        tis = trial_iterations[i]
        tiss = tis[0]
        tise= tis[1]
        prefix_t = prefix +'values_'+str(i)+'-'
        suffix = '.txt'
        for j in range(tiss,tise+1):
            if j==0:
                fn = prefix_t+'0'+suffix
                sorted_files.append(fn)
            else:
                for k in range(0,4):
                    fn = prefix_t+str(j)+'_'+str(k)+suffix
                    sorted_files.append(fn)

    #print (sorted_files)
    return sorted_files

def list_files_by_date():
    import glob
    import os

    files = glob.glob("../results/*.txt")
    files.sort(key=os.path.getmtime)
    for f in files:
        #print f
        if f.startswith('../results/values') and f.endswith('.txt'):
            fns.append(f)

list_files_by_date()
#list_files()

fns = sort_files_properly(fns)
#print(fns)
#fns=[]    
for fn in fns:
    bounds = process_bounds(fn)
    allBounds.append(bounds)
#print(allBounds)
allBs = process_allBounds(allBounds)
#print (allBs)
#plot_allBs(allBs)


#plot_allBsForStateList(allBs,[('0','1','3'),('2','4','3'),
#                              ('0','-1','3'),('0','6','1'),
#                              ('0','7','3'),('0','3','3'),
#                              ('-1','1','3'),('3','1','3'),
#                              ('-1','-1','3'),('-1','6','1'),('3','-1','3'),('3','6','1'),
#                              ('-1','7','3'),('-1','3','3'),('3','7','3'),('3','3','3'),
#                              ('-1','1','3'),('5','1','2'),
#                              ('-1','-1','3'),('-1','6','1'),('5','-1','2'),('5','6','0'),
#                              ('-1','7','3'),('-1','3','3'),('5','7','2'),('5','3','2')])

#plot_allBsForStateList(allBs,[('0','6','1')])

#plot_allBs(allBs)

plot_errorStatesOnly(allBs)

plt.show()
