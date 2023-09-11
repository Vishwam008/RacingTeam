import cantools
from pprint import pprint
import can

can_bus = can.interface.Bus('vcan0', bustype='socketcan')
db = cantools.database.load_file('ADSDV_2021_VCU_AI_interface_v2.dbc')

recmessage = can_bus.recv()
decoded = db.decode_message(recmessage.arbitration_id, recmessage.data)
print(decoded)