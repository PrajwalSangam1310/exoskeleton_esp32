import paho.mqtt.client as paho
import sys
import numpy as np


    # goal_position
    # goal_velocity
    # effort
    # mode
    # emergency_command

def onMessage(client, userdata, msg):
    print(msg.topic+": "+ msg.payload.decode('utf-8'))
    print(msg.payload)
    print(np.frombuffer(msg.payload,dtype=np.float32,count=1))

    # print(np.array(msg.payload, dtype = np.float32))

client = paho.Client()
client.on_message = onMessage

if client.connect("192.168.187.251", 1883, 60) !=0:
    print("couldnt connect to the brker, exititng")
    sys.exit(-1)

# client.subscribe("/RIGHT_THIGH/command_data")

# command payload
example_float_data = np.array([0.0,-1400.0,100.0], dtype=np.float32).tobytes()
# example_float_data = np.array([0.0,1000.0,0.0], dtype=np.float32).tobytes()
example_int_data = np.array([1.0,0.0], dtype=np.int32).tobytes()
send_data  = example_float_data + example_int_data

client.publish("/RIGHT_KNEE/command_data", send_data,0)

#pid payload
pid_payload_data = np.array([0,0,0],  dtype=np.float32).tobytes()
client.publish("/RIGHT_THIGH/pid", pid_payload_data,0)

try:
    print("press ctrl+c to stop the execution")
    client.loop_forever()
except:
    print("Disconnected from the server")

client.disconnect()