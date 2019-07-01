import os
import json
import pandas as pd
import matplotlib.pyplot as plt

data = []
path_to_json = '/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/autogen_testfiles/grid_3_topomap/'
json_ext = '.json'
for path, subdirs, files in os.walk(path_to_json):
    for name in files:
        if name.endswith(json_ext):
            fullfn = os.path.join(path, name)
            with open(fullfn) as json_file:
                data.append(json.load(json_file))

df = pd.DataFrame(data)

print df.columns
print df.ndim
print df.size

df.plot('robots','Total Time')
