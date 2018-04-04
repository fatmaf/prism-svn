#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys

def build_robot_module(n_columns):
    s="mdp\n\nmodule robot\n\n"

    for i in range(0,n_columns):
        for j in range(0,n_columns-i):
            s+='rob' + str(i) + str(j) +':[0..1] init '
            if i==0 and j==0:
                s+=str(1)
            else:
                s+=str(0)
            s+=';\n'
    s+='\n'
    
    for i in range(0,n_columns):
        for j in range(0,n_columns-i):
            current_state_true = "(rob" + str(i) + str(j) + "=1)"
            current_state_false = "(rob" + str(i) + str(j) + "'=0)" 
            if j > 0:
                if i%2==0:
                    s+='[n] ' + current_state_true + ' -> 1.0:' + current_state_false + " & (rob" + str(i) + str(j-1) + "'=1);\n"
                s+='[ne] ' + current_state_true + ' -> 1.0:' + current_state_false + " & (rob" + str(i+1) + str(j-1) + "'=1);\n"
            if i > 0:
                s+='[nw] ' + current_state_true + ' -> 1.0:' + current_state_false + " & (rob" + str(i-1) + str(j) + "'=1);\n"
                s+='[sw] ' + current_state_true + ' -> 1.0:' + current_state_false + " & (rob" + str(i-1) + str(j+1) + "'=1);\n"
            if i + j < n_columns-1:
                s+='[se] ' + current_state_true + ' -> 1.0:' + current_state_false + " & (rob" + str(i+1) + str(j) + "'=1);\n"
                if i%2==0:
                    s+='[s] ' + current_state_true + ' -> 1.0:' + current_state_false + " & (rob" + str(i) + str(j+1) + "'=1);\n"
                
    s += "\nendmodule\n"
    return s
   
def build_tyre_module(n_columns, prob_flat):
    s = "\nmodule tyre\n\n"
    s+= "flat:[0..1] init 0;\n"
    for i in range(0,n_columns):
        for j in range(0,n_columns-i):
            s+='tyre' + str(i) + str(j) +':[0..1] init '
            if j==0 or i + j == n_columns -1 or i%2==1:
                s+=str(1)
            else:
                s+=str(0)
            s+=';\n'
    s+='\n'
    s+="[n] (flat=0) -> " + str(prob_flat) + ":(flat'=1) + " + str(1-prob_flat) + ":(flat'=0);\n"
    s+="[ne] (flat=0) -> " + str(prob_flat) + ":(flat'=1) + " + str(1-prob_flat) + ":(flat'=0);\n"
    s+="[nw] (flat=0) -> " + str(prob_flat) + ":(flat'=1) + " + str(1-prob_flat) + ":(flat'=0);\n"
    s+="[sw] (flat=0) -> " + str(prob_flat) + ":(flat'=1) + " + str(1-prob_flat) + ":(flat'=0);\n"
    s+="[se] (flat=0) -> " + str(prob_flat) + ":(flat'=1) + " + str(1-prob_flat) + ":(flat'=0);\n"
    s+="[s] (flat=0) -> " + str(prob_flat) + ":(flat'=1) + " + str(1-prob_flat) + ":(flat'=0);\n"
    for i in range(0,n_columns):
        for j in range(0,n_columns-i):
            s+="[replace_flat] (rob" + str(i) + str(j) + "=1) & (flat=1) & (tyre" + str(i) + str(j) + "=1) -> 1.0:(flat'=0) & (tyre" + str(i) + str(j) + "'=0);\n" 

    s+="\nendmodule\n"
    return s


if __name__ == '__main__':
    n_columns=int(sys.argv[1])
    
    prob_flat=0.5
    s=build_robot_module(n_columns)
    s+=build_tyre_module(n_columns, prob_flat)
    
    print s
    
    with open('triangletireworld' + str(n_columns) + '.prism', 'w') as f:
       f.write(s)

    s = 'Pmax=? [ F rob0' + str(n_columns-1) + '=1]'
    with open('triangletireworld' +str(n_columns) + '.prop', 'w') as f:
       f.write(s)
