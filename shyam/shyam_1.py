import json

with open('shyam_json_1.json') as f:
    data = json.load(f)

    
for item in data['Template']['motions_'][0]['references_']['states_'][0]['joints_']:
    print (item['value_'])