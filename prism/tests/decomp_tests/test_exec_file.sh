#!/bin/bash

tfn=$1
PRISM_JAVAMAXMEM=6g
export PRISM_JAVAMAXMEM

prismloc="${PWD%%/prism/*}/prism/bin/prism"
tloc="${PWD}/" #"/home/fatma/hubic/phD/work/code/mdpltl/prism-svn/prism/tests/fatma_tests/tests_final/"tloc="/home/fatma/hubic/phD/work/code/mdpltl/prism-svn/prism/tests/fatma_tests/tests_final/"
trloc="${PWD}/res" #/home/fatma/hubic/phD/work/code/mdpltl/prism-svn/prism/tests/fatma_tests/tests_final/res/"trloc="/home/fatma/hubic/phD/work/code/mdpltl/prism-svn/prism/tests/fatma_tests/tests_final/res/"


    echo "=================================================="
    echo "evaluating file $tfn"
    echo "=================================================="
    comm="${prismloc} ${tloc}${tfn}.prism ${tloc}${tfn}.prop -ex -exportadv ${trloc}r${tfn}_adv.tra -exportprodstates ${trloc}r${tfn}_prod.sta -exportstates ${trloc}r${tfn}.sta -exportlabels ${trloc}r${tfn}.lab -exportprodtrans ${trloc}r${tfn}_prod.tra -exporttarget ${trloc}r${tfn}_tar.lab -exporttransdot ${trloc}r${tfn}_trans.dot -exporttransdotstates ${trloc}r${tfn}_sta.dot |tee ${trloc}r${tfn}_output.txt"
    echo "================================================="
    eval $comm
