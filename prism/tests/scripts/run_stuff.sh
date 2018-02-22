tloc=$"/home/fatma/Data/phD/work/code/mdpltl/prism-svn/prism/tests/decomp_tests/"
tfn4=$"two_room_three_robot_extended"
trloc=$"/home/fatma/Data/phD/work/code/mdpltl/prism-svn/prism/tests/decomp_tests/res/"
prismloc=$"/home/fatma/Data/phD/work/code/mdpltl/prism-svn/prism/bin/prism"
${prismloc} -javamaxmem 8g ${tloc}${tfn4}.prism ${tloc}${tfn4}.prop -ex -exportadv ${trloc}r${tfn4}_adv.tra -exportprodstates ${trloc}r${tfn4}_prod.sta -exportstates ${trloc}r${tfn4}.sta -exportlabels ${trloc}r${tfn4}.lab -exportprodtrans ${trloc}r${tfn4}_prod.tra -exporttarget ${trloc}r${tfn4}_tar.lab -exporttransdot ${trloc}r${tfn4}_trans.dot -exporttransdotstates ${trloc}r${tfn4}_sta.dot
