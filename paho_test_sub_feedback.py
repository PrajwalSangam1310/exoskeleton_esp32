import paho.mqtt.client as paho
import sys
import numpy as np

# @datatype
# class points:
#     a1: float

def onMessage(client, userdata, msg):
    print(msg.topic+": ")
    print(msg.payload)
    print(np.frombuffer(msg.payload,dtype=np.float32,count=8))

    # print(np.array(msg.payload, dtype = np.float32))

client = paho.Client()
client.on_message = onMessage

if client.connect("192.168.185.251", 1883, 60) !=0:
    print("couldnt connect to the brker, exititng")
    sys.exit(-1)

client.subscribe("/RIGHT_THIGH/feedback_data")
client.loop_forever()
try:
    print("press ctrl+c to stop the execution")
    client.loop_forever()
except:
    print("Disconnected from the server")

client.disconnect()