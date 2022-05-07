#python

def sysCall_init():
    global leftMotor
    global rightMotor
    global robotHandle
    
    global ultrasonicSensors

    global simTimePub
    global sensorPub
    global positionPub
    global orientationPub

    robotHandle=sim.getObject('.')
    leftMotor=sim.getObject("./leftMotor")
    rightMotor=sim.getObject("./rightMotor")
    ultrasonicSensors = []
    
    
    for i in range(8):
        sensor = sim.getObject("./ultrasonicSensor", {"index": i})
        ultrasonicSensors.append(sensor)
    
    if simROS:
        sim.addLog(sim.verbosity_scriptinfos,"ROS interface was found.")
        sysTime = sim.getSystemTimeInMs(-1)
        leftMotorTopic="leftMotorSpeed"
        rightMotorTopic="rightMotorSpeed"
        simTimeTopic="simTime"
        sensorsTopic="p3dxSensors"
        positionTopic="p3dxPosition"
        orientationTopic = 'PioneerOrientation'
        
        sensorPub=simROS.advertise('/'+sensorsTopic, 'std_msgs/Float32MultiArray')
        simTimePub=simROS.advertise('/'+simTimeTopic,'std_msgs/Float32')
        positionPub=simROS.advertise('/'+positionTopic, 'std_msgs/Float32MultiArray')
        leftMotorSub=simROS.subscribe('/'+leftMotorTopic,'std_msgs/Float32','setLeftMotorVelocity_cb')
        rightMotorSub=simROS.subscribe('/'+rightMotorTopic,'std_msgs/Float32','setRightMotorVelocity_cb')
        orientationPub = simROS.advertise('/'+orientationTopic, 'std_msgs/Float32')
    else:
        sim.addLog(sim.verbosity_scripterrors,"ROS interface was not found. Cannot run.")

def setLeftMotorVelocity_cb(msg):
    global leftMotor
    sim.setJointTargetVelocity(leftMotor, msg['data'])
    
def setRightMotorVelocity_cb(msg):
    global rightMotor
    sim.setJointTargetVelocity(rightMotor, msg['data'])
    
def sysCall_actuation():
    global positionPub
    
    pos = sim.getObjectPosition(robotHandle, sim.getObjectParent(robotHandle))
    ori = sim.getObjectOrientation(robotHandle, sim.getObjectParent(robotHandle))
    
    positionMsg = {'data': [ pos[0], pos[1], ori[2] ]}
    simROS.publish(positionPub, positionMsg)
    

def sysCall_sensing():
    global ultrasonicSensors
    global sensorPub
    global orientationPub

    sensorMsg = {'data': [0,0,0,0,0,0,0,0]}
    for i in range(8):
        res = sim.readProximitySensor(ultrasonicSensors[i])
        if res != 0:
            res = res[1]
        sensorMsg['data'][i] = res
    
    print(sensorMsg['data'])
    simROS.publish(orientationPub, {'data': math.degrees(sim.getObjectOrientation(robotHandle, -1)[2])})
    simROS.publish(sensorPub, sensorMsg)
    simROS.publish(simTimePub, {'data': sim.getSimulationTime()})
    
def sysCall_cleanup():
    # do some clean-up here
    pass

# See the user manual or the available code snippets for additional callback functions and details
