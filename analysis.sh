# if [ $# -eq 0 ]; then
#   echo "Please provide no. of particles"
#   exit 1
# fi

# npart="$1"

# cd analysis 
# cat > truth.txt
# cat > guess.txt 
# cd ../build ./tracking $npart

cd analysis 
python3 analysis.py

