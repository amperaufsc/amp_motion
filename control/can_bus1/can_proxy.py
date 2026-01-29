import can

class CanBusCommunicator:
    def __init__(self, nchannel, nbustype, filters = None):
        
        self.channel = nchannel
        self.bus = can.interface.Bus(channel=nchannel, bustype=nbustype, bitrate=500000)
        
        self.command = 0
        self.data = 100

        if filters:
            self.bus.set_filters(filters)
    
    def send_message(self, id,data) -> None:
        msg = can.Message(arbitration_id=id, data=data, is_extended_id=False)
        self.bus.send(msg)

    def read_message(self):
        return self.bus.recv(0.01)
    