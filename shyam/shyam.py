import json

with open('shyam.json') as f:
    data = json.load(f)
    
for item in data['Template']['motions_'][0]['references_']['states_']:
    print("joint")
    for item2 in item['joints_']:
        a = item2['name_'], item2['value_']
        with open('shyam_new1.txt', 'w') as f:
            json.dumps(a,f)
        print a
    