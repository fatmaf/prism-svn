#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys

def build_robot_module(dim_x, dim_y):
    dim_x = str(dim_x)
    dim_y = str(dim_y)
    s="mdp\n\nmodule robot\n\n"
    s+="rob_x:[1.." + dim_x + "] init " + dim_x + ";\n"
    s+="rob_y:[1.." + dim_y + "] init " + dim_y + ";\n\n"
    s+="[up] (rob_y<" + dim_y + ") ->  1.0:(rob_y' = rob_y + 1);\n"
    s+="[down] (rob_y>1) -> 1.0:(rob_y' = rob_y - 1);\n"
    s+="[right] (rob_x<" + dim_x + ") -> 1.0:(rob_x' = rob_x + 1);\n"
    s+="[left] (rob_x>1) -> 1.0:(rob_x' = rob_x - 1);\n"
    s+="[noop] true -> true;\n\n"
    s+="endmodule\n\n"
    return s

def build_right_cell(pos_x, pos_y, prob):
    pos_x = str(pos_x)
    pos_y = str(pos_y)
    prob2= str(1 - prob)
    prob = str(prob)
    var_name = "x" + pos_x + "y" + pos_y
    s="module " + var_name + "\n\n"
    s+=var_name + ":[0..1] init 0;\n\n"
    s+="[up] true -> " + prob + ":(" + var_name + "'=0) + " + prob2 + ":(" + var_name + "'=1);\n"
    s+="[down] true -> " + prob + ":(" + var_name + "'=0) + " + prob2 + ":(" + var_name + "'=1);\n"
    s+="[right] true -> " + prob + ":(" + var_name + "'=0) + " + prob2 + ":(" + var_name + "'=1);\n"
    s+="[left] true -> " + prob + ":(" + var_name + "'=0) + " + prob2 + ":(" + var_name + "'=1);\n"
    s+="[noop] true -> " + prob + ":(" + var_name + "'=0) + " + prob2 + ":(" + var_name + "'=1);\n\n"
    s+="endmodule\n\n"
    return s

    

def build_mid_cell_module(pos_x, pos_y):
    pos_y = str(pos_y)
    var_name = "x" + str(pos_x) + "y" + pos_y
    righ_cell_var_name = "x" + str(pos_x + 1) + "y" + pos_y
    s="module " + var_name + "\n\n"
    s+=var_name + ":[0..1] init 0;\n\n"
    s+="[up] (" + righ_cell_var_name + "=1) -> 1:(" + var_name + "'=1);\n"
    s+="[up] (" + righ_cell_var_name + "=0) -> 1:(" + var_name + "'=0);\n"
    s+="[down] (" + righ_cell_var_name + "=1) -> 1:(" + var_name + "'=1);\n"
    s+="[down] (" + righ_cell_var_name + "=0) -> 1:(" + var_name + "'=0);\n"
    s+="[right] (" + righ_cell_var_name + "=1) -> 1:(" + var_name + "'=1);\n"
    s+="[right] (" + righ_cell_var_name + "=0) -> 1:(" + var_name + "'=0);\n"
    s+="[left] (" + righ_cell_var_name + "=1) -> 1:(" + var_name + "'=1);\n"
    s+="[left] (" + righ_cell_var_name + "=0) -> 1:(" + var_name + "'=0);\n"
    s+="[noop] (" + righ_cell_var_name + "=1) -> 1:(" + var_name + "'=1);\n"
    s+="[noop] (" + righ_cell_var_name + "=0) -> 1:(" + var_name + "'=0);\n\n"
    s+="endmodule\n\n"
    return s


def build_crash_spec(dim_x, dim_y):
    s='(G (!('
    for i in range(1, dim_x + 1):
        for j in range(2, dim_y):
            s+='(rob_x=' + str(i) + ' & rob_y=' + str(j) + ' & x' + str(i) + 'y' + str(j) + '=1) | '
    s=s[0:-3] + ')))'
    return s

if __name__ == '__main__':
    dim_x=int(sys.argv[1])
    dim_y=int(sys.argv[2])
    prob=0.7
    
    
    s=build_robot_module(dim_x, dim_y)
    
    for i in range(2, dim_y):
        s+=build_right_cell(dim_x, i, prob)
    
    for i in range(1, dim_x):
        for j in range(2, dim_y):
            s+=build_mid_cell_module(i,j)
    
    with open('crossing' + str(dim_x) + ',' + str(dim_y) + '.prism', 'w') as f:
       f.write(s)
       #(G (F (rob_x=1 & rob_y = 1))) & (G (F (rob_x=3 & rob_y = 3)))]

    s = 'Pmax=? [' + build_crash_spec(dim_x, dim_y) + ' & (G (F (rob_x=1 & rob_y = 1))) & (G (F (rob_x=' + str(dim_x) + ' & rob_y=' + str(dim_y) + ')))]'
    with open('crossing' + str(dim_x) + ',' + str(dim_y) + '.prop', 'w') as f:
       f.write(s)
