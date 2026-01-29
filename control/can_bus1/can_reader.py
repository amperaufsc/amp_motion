import struct
import logging
import cantools
import can
import cantools.database
import numpy as np

from can_proxy import CanBusCommunicator

class StateCanReader():
    def __init__(self) -> None:
        DBC_FILE = "src/as_amp/can_bus/config/AS_CAN.dbc"
        BUSTYPE = "socketcan"
        CHANNEL = "can0"

        self.db = cantools.database.load_file(DBC_FILE)

        filters = [
            {"can_id": 1104, "can_mask": 0x7FF, "extended": False},
            {"can_id": 1105, "can_mask": 0x7FF, "extended": False},
            {"can_id": 1106, "can_mask": 0x7FF, "extended": False},
            {"can_id": 865, "can_mask": 0x7FF, "extended": False},
            {"can_id": 881, "can_mask": 0x7FF, "extended": False},
            {"can_id": 882, "can_mask": 0x7FF, "extended": False},
            {"can_id": 883, "can_mask": 0x7FF, "extended": False},
            {"can_id": 884, "can_mask": 0x7FF, "extended": False},
            {"can_id": 885, "can_mask": 0x7FF, "extended": False},
            {"can_id": 886, "can_mask": 0x7FF, "extended": False},
            {"can_id": 887, "can_mask": 0x7FF, "extended": False},
            {"can_id": 888, "can_mask": 0x7FF, "extended": False},
            {"can_id": 288, "can_mask": 0x7FF, "extended": False},
            {"can_id": 273, "can_mask": 0x7FF, "extended": False},
            {"can_id": 1088, "can_mask": 0x7FF, "extended": False},
            {"can_id": 1089, "can_mask": 0x7FF, "extended": False},
            {"can_id": 560, "can_mask": 0x7FF, "extended": False},

        ]

        self.logger = logging.getLogger('CAN_Reader')
        self.logger.setLevel(logging.INFO)

        handler = logging.StreamHandler()
        formatter = logging.Formatter('%(name)s - %(levelname)s - %(message)s')
        handler.setFormatter(formatter)
        self.logger.addHandler(handler)

        self.logger.info("CAN Reader inicializado")

        self.can_listener = CanBusCommunicator(CHANNEL, BUSTYPE, filters)

    def can_reader(self, message):
        try:
            can_message = self.db.decode_message(message.arbitration_id, message.data)
            self.logger.debug(f"ID {message.arbitration_id}: {can_message}")
        except Exception as e:
            self.logger.error(f"Erro ao decodificar ID {message.arbitration_id} (Dados: {message.data.hex()}): {str(e)}")
            raise RuntimeError(e) from e

        values = {

            "Estercamento_Atuador": None,
        }

        try:
            match message.arbitration_id:
            
                case 560:
                    for key in ['Estercamento_Atuador']:
                        if key in can_message:
                            values[key] = can_message[key]
         
        except Exception as e:
            self.logger.error(f"Erro ao mapear valores da mensagem CAN ID: {str(e)}")

        return values


    def receive_message(self):
        try:
            return self.can_listener.read_message()
        except Exception as e:
            self.logger.error(f"Erro ao ler mensagem CAN: {str(e)}")
            return None