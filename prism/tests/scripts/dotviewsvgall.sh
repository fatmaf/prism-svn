#!/bin/bash
pwd=$PWD
skip='teamMDP'

for f in *.dot; do

    if [[ $f == *$skip* ]];then
	continue
    else
	echo "converting $f"
	dot -Tsvg "$pwd/$f" -o "$pwd/$f.svg"
	echo "opening $f"
	google-chrome-stable "$pwd/$f.svg"
   fi
    

done

#echo "converting $pwd'/'$1"
#dot -Tsvg "$pwd/$1" -o "$pwd/$1.svg"
#echo "opening $pwd'/'$1"
#google-chrome-stable "$pwd/$1.svg"
  
