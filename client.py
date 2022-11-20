from networktables import NetworkTables
import json
import sys

NetworkTables.initialize(server='127.0.0.1')

tests = ['fast-backward', 'fast-forward', 'slow-backward', 'slow-forward']

if __name__ == '__main__':
    sd = NetworkTables.getTable('SmartDashboard')

    while not NetworkTables.isConnected():
        ...

    first_test = True
    with open(sys.argv[1], 'w') as f:
        f.write('{')

        for test in tests:
            s = sd.getString(test, None)
            if s is None:
                continue

            s = s.replace('E', 'e')
            s = s.replace('[', '[\n')
            s = s.replace(']', '\n]')
            s = s.replace(',', ',\n')

            if not first_test:
                f.write(',')
            else:
                first_test = False
            
            f.write(f'"{test}": [{s}]')
        
        f.write(', "sysid": true,"test": "Drivetrain", "units": "Meters", "unitsPerRotation": 0.319185814}')
