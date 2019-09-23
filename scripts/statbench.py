import numpy as np
import scipy.stats as stats
import subprocess
import sys

data = []

# Wilcoxon Signed Rank Test w/ confidence interval in R?

def sample():
  global data
  proc = subprocess.run(sys.argv[1:], stdout=subprocess.PIPE)
  assert proc.returncode == 0
  data.append(float(proc.stdout))
  if len(data) > 2:
    #mu = np.mean(data)
    #sigma = np.std(data, ddof=1) / np.sqrt(len(data))
    #interval = stats.norm.interval(0.95, loc=mu, scale=sigma)
    data = list(sorted(data))
    med = np.median(data)
    n = len(data)
    z =  1.96
    p = 0.5
    lo = int(np.floor(n * p - z * np.sqrt(n * p * (1 - p))))
    hi = int(np.ceil(1 + n * p + z * np.sqrt(n * p * (1 - p))))
    print(lo, hi, n)
    if hi >= n: return False
    interval = (data[lo], data[hi])
    width = (interval[1] - interval[0]) / (interval[0] + interval[0]) * 2 
    print(width, interval, np.median(data))
    return width < 0.05

for i in range(5):
  sample()

while True:
  if sample():
    break 
