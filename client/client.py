from networktables import NetworkTables
from math import pi
import json
import sys

tests = ['fast-backward', 'fast-forward', 'slow-backward', 'slow-forward']

if __name__ == '__main__':
    data = {}
    try:
        with open('client.json') as f:
            data = json.load(f)
    except:
        exit('Error: must create "client.json" file in same directory, see README.md for content specifications')

    ### Networktables ###

    NetworkTables.initialize(server=data['ip'])

    sd = NetworkTables.getTable('SmartDashboard')

    while not NetworkTables.isConnected():
        ...

    json_str = '{'

    for test in tests:
        s = sd.getString(test, None)
        if s is None:
            continue

        s = s.replace('E', 'e')

        if len(json_str) > 1:
            json_str += ','
        
        json_str += f'"{test}": [{s}]'
    
    json_str += f',"sysid":true,"test":"Drivetrain","units":"Meters","unitsPerRotation":{data["wheelDiameter"] * pi}}}'
    
    with open(data['outputFile'], 'w') as f:
        json.dump(json.loads(json_str), f, indent=4)
