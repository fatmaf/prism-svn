#! /usr/bin/env bash

for file in `ls *.dot`
do 
    dot -Tpdf -o ${file/.dot/}.pdf $file
done
