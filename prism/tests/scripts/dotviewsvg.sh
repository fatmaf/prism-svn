#!/bin/bash

if (-f $1.dot) then
  dot -Tsvg $1.dot -o $1.dot.svg
  if ("$OSTYPE" == "darwin") then
    open $1.dot.pdf
  else
    google-chrome-stable $1.dot.pdf
  fi
else
  echo 'dotview: cannot find file "'$1.dot'"'
fi
