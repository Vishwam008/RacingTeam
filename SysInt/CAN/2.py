import can
bus = can.interface.Bus(interface='socketcan', channel='vcan0', bitrate=500000)
msg = can.Message(arbitration_id=0xabcde, data=[1,2,3])
bus.send(msg)