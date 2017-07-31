import os 
from subprocess import call 

prodloc = '.'; 
advloc = '.'; 
traext = '_prod.tra'; 
staext = '_prod.sta'; 
advext = '_adv.tra';
tarext = '_tar.lab';
ns = 1
global expandcount;
expandcount = 0;

def readprodstate(fn):
    f = open(fn,'r');
    f.readline()
    indToState={}
    for line in f:
        y = line.split(':')
        y[1] = y[1].strip('\n')
        indToState[y[0]]=y[1]
    f.close();
    return indToState;

def getAllfiles(loc,ext):
    fileslist = [x for x in os.listdir(loc) if x.endswith(ext)];
    return fileslist


def getfiles(prodloc,advloc,traext,staext,advext,tarext):
    trafiles = getAllfiles(prodloc,traext);
    stafiles = getAllfiles(prodloc,staext);
    advfiles = getAllfiles(advloc,advext);
    tarfiles = getAllfiles(advloc,tarext);
    modnames = []; 
    for fn in trafiles:
        temp = fn[1:len(fn)-len(traext)];
        modnames.append(temp); 
    return (modnames,trafiles,stafiles,advfiles,tarfiles)

def getinitstate(advloc,advfile):
    f=open(advloc+advfile);
    f.readline();
    line = f.readline();
    f.close();
    line = line.split(':');
    init_s = int(line[0]);
    return init_s;

def getfilelines(prodloc,stafn):
    if prodloc == '.':
        f = open(stafn,'r'); 
    else:
        f = open(prodloc+stafn,'r');
    f.readline(); 
    filelines = []; 
    for line in f:
#        temp = line.split(' '); 
        filelines.append(line);
    f.close();
    return filelines;

def count_policies(ind,pol_lines):
    nl = len(file_lines)
    nump = 0
    starter = [];
    lines = [];
    for j in range(nl):
        line = pol_lines[j]
        temp = line.split(' ')
        temp0 = int(temp[0])
        if(temp0 == ind):
            nump= nump+1;
            temp1 = int(temp[1]);
            starter.append(temp1);
            lines.append(line)
    return (nump,starter,lines)


def get_policy(ind,pol_lines,ns):
    (nump,next_s,crap) = count_policies(ind,pol_lines); 
    policy = [] 
    #next_s = []
    for i in range(nump):
        policy.append([next_s[i]])
    #    next_s.append(ind)
    rep = True
    while(rep == True):
        rep = False
        for j in range(num_lines):
            line=file_lines[j];
            temp = line.split(' ');
            for i in range(nump):
                temp0 = int(temp[0]);
                if (temp0 == next_s[i]):
                    #print(str(next_s[i])+' to '+ temp[ns]);
                    next_s[i] = int(temp[ns]);
                    policy[i].append(next_s[i]);
                    if(temp0 > next_s[i]):
                        rep = True
                    #    print('repeat')
    return policy

def get_policy_six_room(ind,pol_lines,ns):
    (nump,next_s,crap) = count_policies(ind,pol_lines); 
    #print ind
    
    policy = [] 
    #next_s = []
    moves=[]
    for i in range(nump):
        policy.append([next_s[i]])
        moves.append([pol_lines[ind+i]]);
    #    next_s.append(ind)
    rep = True
    while(rep == True):
        rep = False
        for j in range(num_lines):
            line=file_lines[j];
            temp = line.split(' ');
            for i in range(nump):
                temp0 = int(temp[0]);
                if (temp0 == next_s[i]):
                    #print(str(next_s[i])+' to '+ temp[ns]);
                    next_s[i] = int(temp[ns]);
                    moves[i].append(line)
                    #print temp
                    #print next_s[i]
                    policy[i].append(next_s[i]);
                    if(temp0 > next_s[i]):
                        rep = True
                    #    print('repeat')
    return moves

def get_policy_adv(ind,pol_lines):
    return get_policy(ind,pol_lines,1);

def get_policy_tras(ind,pol_lines):
    return get_policy(ind,pol_lines,2);

def flat_list(alist,initl):
    
    for j in range(len(alist)):
        #print initl
        #print j
        #print type(alist[j])
        if (type(alist[j])==str):
            if not alist[j]=='':
                initl.append(alist[j])
            #print "in if"
        else:
            #print "in else"
            #print j
            #print initl
            initl=flat_list(alist[j],initl)
            
    return initl

def moves_pp(pol):
    for j in range(len(pol)):
        if (type(pol[j])==str):
            print str(pol[j].replace('\n',''))+",",
        else:
            print "\n\t",
            moves_pp(pol[j]);

def test(pol,plines,file_lines):
    #print "in test with pol "
    #print pol
    cpol = []
    cplines=[]
    lenp = len(pol)
    for j in range(0,lenp):
        #print "pol j =",
        #print j,
        #print ",",
        #print pol[j]
        if(type(pol[j])==int):
            #print "adding to cpol",
            #print cpol,
            cpol.append(pol[j])
            cplines.append(plines[j])
            #print cpol
            (nump,temp,lines)=count_policies(pol[j],file_lines)
            #print "numbers and temp",
            #print nump,
            #print temp
            #if nump>1:
            #print "greater than 1",
            #print temp
            (ccpol,ccplines)=test(temp,lines,file_lines)
            #print "printing ccpol",
            #print ccpol,
            cpol.append(ccpol)
            cplines.append(ccplines)
            #print "appending to cpol"
            #print cpol,
                
        else:
            #print "calling the function again"
            (cpol,cplines) = test(pol[j],file_lines)
            #print "new pol done"
    #print "done",
    #print cpol
    return (cpol,cplines)
                

def expand_moves_pp(pol,moves,file_lines):
    global expandcount
    expandcount = expandcount+1
    print expandcount
    if expandcount < 100:
        for j in range(0,len(pol)):
            #print "Range ",
            #print range(1,len(pol)),
            #print " Value ",
            #print j
            if j<len(pol):
                if(type(pol[j])==int):
                    (nump,temp,crap) = count_policies(pol[j],file_lines); 
                    if nump>1:
                        imp = pol[j]
                        pol[j] = get_policy_adv(pol[j],file_lines); 
                        moves[j] = get_policy_six_room(imp,file_lines,1); 
                        (pol[j],moves[j])=expand_moves_pp(pol[j],moves[j],file_lines)
                        expandcount=expandcount-1
                else:
                    (pol[j],moves[j])=expand_moves_pp(pol[j],moves[j],file_lines)
                    expandcount=expandcount-1
            #except:
                #print "Error ",
                #print pol,
                #print len(pol)
    return (pol,moves)


def policy_pp(pol):
    for j in range(len(pol)):
        if (type(pol[j])==int):
            print str(pol[j])+",",
        else:
            print "\n\t",
            policy_pp(pol[j]);

def policy_ppt(pol,numt):
    for j in range(len(pol)):
        if (type(pol[j])==int):
            print str(pol[j])+",",
        else:
            print "\n",
            
            policy_pp(pol[j]);

def policy_pp_ind2state(pol,indtostate,f):
    writefn = (not (f is None));
    #if writefn:
     #   f = open(fn,'w');

    for j in range(len(pol)):
        if (type(pol[j])==int):
            print indtostate[str(pol[j])]+" ,",
            if writefn:
                f.write(indtostate[str(pol[j])]+" ,");
        else:
            print "\n\t",
            if writefn:
                f.write("\n\t")
            policy_pp_ind2state(pol[j],indtostate,f);
    #if writefn:
    #    f.close();

def get_file_with_name(name,fls):
    for fl in fls:
        #print fl
        #print name
        if (name+"_tar" in fl) or (name+"_prod" in fl) or (name+"_adv") in fl:
            #print fl;
            return fl;
    return '';

def fixstate(tofix):
    t0 = tofix
    t0 = t0.strip('()')
    t0 = t0.split(',')
    t0 = '_'.join(t0[1:len(t0)])
    t0 = t0.replace('-1','x')
    return t0

def get_tra_lines(file_lines):
    tra_lines=[];
    for line in file_lines:
        temp = (line.strip('\n')).split(' ')
        tra_line = [temp[0],temp[1],temp[2],''.join(temp[3:len(temp)])]
        tra_lines.append(tra_line)
    return tra_lines

def create_easytra(fn,tra_lines):
    f=open(fn,'w')
    f.write(str(len(tra_lines))+'\n')
    for tra_line in tra_lines:
        tra_line[0] = 's'+indToState[tra_line[0]].strip('()').replace('-1','x').replace(',','_')
        tra_line[1] = 's'+indToState[tra_line[1]].strip('()').replace('-1','x').replace(',','_')
        temp = ' '.join(tra_line)
        f.write(temp+'\n')
        #print tra_lines
    f.close()


def create_easytra_dot(fn,tra_lines):
    f=open(fn,'w')
    f.write('digraph adv {\nrankdir=LR\n')
    d=dict()
    for tra_line in tra_lines:
        towrite = tra_line[0]+' -> '+tra_line[1]+' [label="'+tra_line[3]+'('+tra_line[2]+')"];\n'
        f.write(towrite)
        if tra_line[0] in d:
            d[tra_line[0]] +=1
        else:
            d[tra_line[0]]=1
        if tra_line[1] in d:
            d[tra_line[1]] +=1
        else:
            d[tra_line[1]]=1
    f.write('}')
    f.close()


(names,tras,stas,advs,tars)=getfiles(prodloc,advloc,traext,staext,advext,tarext); 
#i = 0; 
#if i == 0:
for i in range(len(names)):
    #if not 'six_room_office' in names[i]:
    #    continue
    tar = get_file_with_name(names[i],tars);
    sta = get_file_with_name(names[i],stas); 
    adv = get_file_with_name(names[i],advs);
    print ("Reading policy for "+names[i]+" from files\n\t"+tar+"\n\t"+sta+"\n\t"+adv+"\n");
    init_s = getinitstate('',tar);
    model_name = names[i]; 
    indToState = readprodstate(sta); 
    policy = []; 
    next_s = []; 
    num_policices = 0; 
    file_lines = getfilelines(advloc,adv);
    tra_lines = get_tra_lines(file_lines);
    easytrafn = names[i]+"_adv_easy.tra";
    easyadvfn = names[i]+"_adv_easy.dot";
    create_easytra(easytrafn,tra_lines);
    create_easytra_dot(easyadvfn,tra_lines);
    if not "six" in names[i]:
        call("dot -Tpdf "+easyadvfn+" -o "+names[i]+"_adv_easy.pdf",shell=True)
    num_lines = len(file_lines);
    print ("Reading policy for "+names[i]+" from files\n\t"+tar+"\n\t"+sta+"\n\t"+adv);
    (num_policies,next_s,crap) = count_policies(init_s,file_lines);
    #print ('done')
    policy = get_policy_adv(init_s,file_lines);
    moves = get_policy_six_room(init_s,file_lines,ns);
    #print moves
    print 'redo'
    toexp=[];
    
    for k in range(num_policies):
        toexp.append([]);
        for j in range(1,len(policy[k])):
            (nump,temp,crap) = count_policies(policy[k][j],file_lines);
            if nump > 1:
                imp = policy[k][j]
                policy[k][j] = get_policy_adv(policy[k][j],file_lines);
                moves[k][j] = get_policy_six_room(imp,file_lines,1);
                toexp[k].append(j)
    print("policy for "+names[i])
    policy_pp(policy)
    print("moves for "+names[i])
    moves_pp(moves)
    #print("\neasy policy")
    #f = open(names[i]+"easy_pol.txt",'w');
    #policy_pp_ind2state(policy,indToState,f);
    #f.close();
    (polp,mp)=test([init_s],[''],file_lines)
    print("better stuff")
    policy_pp(polp)
    print("moves")
    moves_pp(mp)
    flatmp = flat_list(mp,[])
    initlines = get_tra_lines(flatmp)
    create_easytra_dot(names[i]+"_adv_easy_init.dot",initlines)
    raw_input()


    

    


