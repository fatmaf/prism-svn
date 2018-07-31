import random
def get_n_states_at_random(n,numStates,prevlist):
    states_list = prevlist[:] 
    prevn = len(prevlist)
    rval = n-prevn
    for i in range(n-prevn):
        rand_state = random.randint(0,numStates)
        while rand_state in states_list: 
            rand_state = random.randint(0,numStates)
        states_list.append(rand_state)
    return states_list

def get_module_lines(prism_model_lines):
    start = -1
    end = -1
    startword = "module"
    endword = "endmodule"
    for i in range(len(prism_model_lines)):
        line = prism_model_lines[i]
        if endword in line: 
            end = i
        elif startword in line:
            start = i 
        if (start != -1 and end != -1):
            break 
    return (start+1,end,prism_model_lines[start+1:end])

def add_failure_to_states(state, module_lines):
    state_line = "1:(waypoint'="+str(state)+")"
    rep_line = "p:(waypoint'="+str(state)+") + (1-p):(waypoint'=failstate)"
    new_module_lines = []
    for line in module_lines:
        if state_line in line:
            line = line.replace(state_line,rep_line)
        new_module_lines.append(line)
    return new_module_lines

def create_new_prism_model_lines(new_module_lines,module_start,module_end,prism_model_lines):
    new_prism_model = list(prism_model_lines)
    new_prism_model[module_start:module_end]=new_module_lines
    return new_prism_model

def write_model_to_file(filename,model_lines):
    f=open(filename,"w")
    f.writelines(model_lines)
    f.close()

def create_model_name(filename,num_fail_states,model_ext):
    nfilename = filename+"_fs"+str(num_fail_states)+model_ext   
    return nfilename

def get_init_state_for_model_num(num_model):
    init_states = [21,24,6,15,5,1,19,8]
    return init_states[num_model]

def modify_init_line(module_lines,num_model):
    init_line_text="waypoint:[failstate..28] init "
    for i in range(len(module_lines)):
        if init_line_text in module_lines[i]:
            module_lines[i]=init_line_text+str(get_init_state_for_model_num(num_model))+";\n"
            break
    return module_lines

def create_new_prism_model(folder,new_folder,filename,prism_model_ext,numStates,num_rand_states,num_models,states_to_fail_at):
    #read in the model and extract the module bit
    prism_model_file = open(folder+filename+prism_model_ext)
    prism_model_lines = prism_model_file.readlines()
    prism_model_file.close()
    module_start,module_end,module_lines = get_module_lines(prism_model_lines)
    
    
    #generate states to fail at
    states_to_fail_at=get_n_states_at_random(num_rand_states,numStates,states_to_fail_at)
    
    #create the new model 
    for i in range(num_rand_states):
        new_module_lines=add_failure_to_states(states_to_fail_at[i],module_lines)
        module_lines = new_module_lines
        
    new_prism_model_lines = create_new_prism_model_lines(new_module_lines,module_start,module_end,prism_model_lines)
    
    for i in range(num_models):
        new_prism_model_lines = modify_init_line(new_prism_model_lines,i)
        #write to file
        new_filename = create_model_name(new_folder+filename,num_rand_states,"_"+str(i)+prism_model_ext)
        write_model_to_file(new_filename,new_prism_model_lines)
    return states_to_fail_at

folder = "../decomp_tests/IROS_2018_final_submission/inc/"
filename = "topo_map_failbase"
prism_model_ext = ".prism"
#we know that there are 0 to 28 states 
numStates = 28 
num_models = 8


#test 
fs_states_list = [5,10,15,20,25]
states_to_fail_at=[]
for num_rand_states in fs_states_list:
    new_folder = folder+"fs_"+str(num_rand_states)+"/"
    import os
    newpath = new_folder
    if not os.path.exists(newpath):
        os.makedirs(newpath)
    states_to_fail_at=create_new_prism_model(folder,new_folder,filename,prism_model_ext,numStates,num_rand_states,num_models,states_to_fail_at)
    print states_to_fail_at

    
