import os
import pstats

# Runs main_single_sim and spits out a txt file.
# The txt file is readable by pstats, which spits out
# sorted data in the terminal. 

os.system("python -m cProfile -o profileSim.txt main_single_sim.py easyTest.yml")
p=pstats.Stats("profileSim.txt")
p.sort_stats('time').print_stats(30) # Sorts by total time in each function, excluding sub-functions
#p.sort_stats('cumulative').print_stats(10) # Sorts by total time in each function, including sub-functions
