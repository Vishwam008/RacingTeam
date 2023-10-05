import can
import time
from datetime import datetime
import random


class Bus:
    def __init__(self):  
        self.bus = can.interface.Bus('node', bustype='virtual') # creates a virtual bus with channel name as node
        self.file = open("log.txt", "a") # opens a log file instance


    def node(self):
        while True:
            l = [random.randint(0, 100), random.randint(0, 100), random.randint(0, 100)] # Generating random data
            msg = can.Message(arbitration_id=0xabcde, data=l) # Generating a message with arbitration_id and data= random data we produced
            self.bus.send(msg) # sending the created message onto the bus
            self.file.write("Published: "+str(datetime.now())+"\t"+str(list(msg.data))+"\n") # logging the send
            self.file.flush() # saving the data onto the file (flushing the buffer)
            time.sleep(1) # program works at (almost) 1Hz 
            

b = Bus()
b.node()