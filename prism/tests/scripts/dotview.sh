#!/bin/bash

if (-f $1.dot) then
  dot -Tpdf $1.dot -o $1.dot.pdf
  if ("$OSTYPE" == "darwin") then
    open $1.dot.pdf
  else
    acroread $1.dot.pdf
  fi
else
  echo 'dotview: cannot find file "'$1.dot'"'
fi
