import sys
import pandas as pd
import matplotlib.pyplot as plt 
import numpy as np

# Find the experiment type 
experiment_type = sys.argv[1]
truth = pd.read_csv('results/truth-exp3-bunny.txt', delimiter=',').to_numpy()