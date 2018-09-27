import json

with open('shyam_newdata.json') as f:
    data = json.load(f)
    
with open('shyam_2.json', 'w') as f:
    json.dump(data, f, indent=2)