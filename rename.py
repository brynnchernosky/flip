import os 
from pathlib import Path

folder = "damBreak_v1"

filenames = []
directory = os.fsencode(folder)
for file in os.listdir(directory):
    filename = os.fsdecode(file)
    if filename.endswith(".png"):
        filenames.append(filename)

filenames = sorted(filenames, key=lambda x: float(Path(x).stem))

for i, filename in enumerate(filenames):
    filepath = os.path.join(folder, filename)
    new_name = os.path.join(folder, str(i) + ".png")
    print(filepath)
    print(new_name)
    os.rename(filepath, new_name)