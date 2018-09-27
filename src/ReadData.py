import json
import time

with open('/home/jcl-mh/ropha_ws/src/ropha_controller_interface/src/shyam_json_1.json') as f:
    data = json.load(f)
    
with open("/home/jcl-mh/ropha_ws/src/ropha_controller_interface/src/shyam_final.txt", "w") as f:
    
    tic = time.clock()
    for i in range(len(data['Template']['motions_'][0]['references_']['states_'])):
        print (json.dump("Positions", f))
        print ("Positions")
        for item in data['Template']['motions_'][0]['references_']['states_'][0]['joints_']:
            b = item ['name_']
            a = item['value_']
            print (b, a)
            print (json.dump((b,a), f))
            #print(json.dump("---"*10, f, indent=2))
        #for item2 in range(len(data['Template']['motions_'][0]['properties_']['dynamics_']['velocities_'])):
        c = data['Template']['motions_'][0]['properties_']['dynamics_']['velocities_']
        print ("Velocity: ", c)
        print (json.dump("Velocity: ", f))
        print (json.dump(c, f))
        d = data['Template']['motions_'][0]['properties_']['dynamics_']['accelerations_']
        print ("Acceleration: ", d)
        print (json.dump("Accelerarion: ", f))
        print (json.dump(d, f))
        print ("\n \n \n")
    toc = time.clock()
    print(json.dump((toc - tic)*1000, f))
    print((toc - tic)*1000)
