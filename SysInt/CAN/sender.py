import cantools
from pprint import pprint
import can

can_bus = can.interface.Bus('vcan0', bustype='socketcan')
db = cantools.database.load_file('ADSDV_2021_VCU_AI_interface_v2.dbc')
example = db.get_message_by_name('AI2VCU_Brake')

# pprint(example.signals)

data = example.encode({'HYD_PRESS_F_REQ_pct': 50, 'HYD_PRESS_R_REQ_pct': 75})
message = can.Message(arbitration_id=example.frame_id, data=data)
can_bus.send(message)
