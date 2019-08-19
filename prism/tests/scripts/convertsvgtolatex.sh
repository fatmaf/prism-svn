#!/bin/bash

if (-f $1.svg) then
  inkscape -D -z --file=$1.svg --export-pdf=$1.pdf --export-latex
  
else
  echo 'dotview: cannot find file "'$1.svg'"'
fi
