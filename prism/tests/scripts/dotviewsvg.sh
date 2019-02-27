#!/bin/bash
pwd=$PWD
echo "opening $pwd'/'$1.dot"

dot -Tsvg "$pwd/$1.dot" -o "$pwd/$1.dot.svg"
google-chrome-stable "$pwd/$1.dot.svg"
  
