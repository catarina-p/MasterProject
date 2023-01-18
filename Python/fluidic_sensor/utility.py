from time import time
import numpy as np

def find_offset(arduino, msg_names, sample_time = 1):
    num_sensor = len(msg_names)

    offsets = np.zeros(num_sensor)
    count = 0
    timer = time()
    while 1: 
        # Receive message from arduino
        arduino.receive_message()

        if arduino.newMsgReceived:
            for i in range(num_sensor):
                offsets[i] += float(arduino.receivedMessages[msg_names[i]])
            count += 1
        
        if time() - timer > sample_time:
            for i in range(num_sensor):
                offsets[i] /= count
            break

    return offsets