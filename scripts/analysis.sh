#!/bin/bash

if [ $# -eq 0 ]; then
  echo "Please name of plot"
  exit 1
fi


echo "Creating plots for experiment $1"
cd ../analysis 
python3 analysis.py "$1"
