#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from can_classes.can_reader import StateCanReader
from fs_msgs.msg import ControlCommand, GoSignal
from std_msgs.msg import Float32, UInt8, UInt16

class CanToRosNode(Node):
    def __init__(self):
        super().__init__('can_to_ros_node')
        
        # Inicialização CAN
        self.can_reader = StateCanReader()

        self.latest_control_command = None
        
        # Subscriber

        self.control_reference = self.create_subscription(ControlCommand, "/control_command", 
                                                    self.control_callback, 10)
        
        self.go_publisher = self.create_publisher(GoSignal, "/signal/go", 10)
        
        # Publishers Float32
        self.float_publishers = {
            '/can/steering_angle': self.create_publisher(Float32, '/can/steering_angle', 10),
            '/can/motor_temp': self.create_publisher(Float32, '/can/motor_temp', 10),
            '/can/motor_torque': self.create_publisher(Float32, '/can/motor_torque', 10),
            '/can/motor_rpm': self.create_publisher(Float32, '/can/motor_rpm', 10),
            '/can/motor_input_power': self.create_publisher(Float32, '/can/motor_input_power', 10),
            '/can/motor_output_power': self.create_publisher(Float32, '/can/motor_output_power', 10),
            '/can/inverter_current': self.create_publisher(Float32, '/can/inverter_current', 10),
            '/can/inverter_voltage': self.create_publisher(Float32, '/can/inverter_voltage', 10),
            '/can/inverter_temp': self.create_publisher(Float32, '/can/inverter_temp', 10),
            '/can/pressure_t': self.create_publisher(Float32, '/can/pressure_t', 10),
            '/can/pressure_f': self.create_publisher(Float32, '/can/pressure_f', 10),
            '/can/encoder_fl': self.create_publisher(Float32, '/can/encoder_front_left', 10),
            '/can/encoder_fr': self.create_publisher(Float32, '/can/encoder_front_right', 10),
            '/can/encoder_rl': self.create_publisher(Float32, '/can/encoder_rear_left', 10),
            '/can/encoder_rr': self.create_publisher(Float32, '/can/encoder_rear_right', 10),
            '/can/BMS_current': self.create_publisher(Float32, '/can/BMS_current', 10),
            '/can/total_voltage': self.create_publisher(Float32, '/can/total_voltage', 10),
            '/can/hv_voltage': self.create_publisher(Float32, '/can/hv_voltage', 10),
            '/can/Car_speed': self.create_publisher(Float32, '/can/car_speed', 10),
            '/can/protection_flags': self.create_publisher(Float32, '/can/protection_flags', 10)
        }
        
        # Publishers UInt8
        self.uint8_publishers = {
            '/can/pedal_angle': self.create_publisher(UInt8, '/can/pedal_angle', 10),
            '/can/brake_pedal': self.create_publisher(UInt8, '/can/brake_pedal', 10),
            '/can/ACC_pedal': self.create_publisher(UInt8, '/can/ACC_pedal', 10),
            '/can/res': self.create_publisher(UInt8, '/can/res', 10),
            '/can/task_mode': self.create_publisher(UInt8, '/can/task_mode', 10),
            '/can/torque_mode': self.create_publisher(UInt8, '/can/torque_mode', 10),
            '/can/cebolinha': self.create_publisher(UInt8, '/can/cebolinha', 10),
            '/can/max_cell_temperature': self.create_publisher(UInt8, '/can/max_cell_temperature', 10),
            '/can/avarage_cell_temperature': self.create_publisher(UInt8, '/can/avarage_cell_temperature', 10),
            '/can/estercamento_atuador': self.create_publisher(UInt8, '/can/estercamento_atuador', 10)
        }
        
        # Publishers UInt16 
        self.uint16_publishers = {
            '/can/susp_te': self.create_publisher(UInt16, '/can/susp_te', 10),
            '/can/susp_fd': self.create_publisher(UInt16, '/can/susp_fd', 10),
            '/can/susp_fe': self.create_publisher(UInt16, '/can/susp_fe', 10),
            '/can/susp_td': self.create_publisher(UInt16, '/can/susp_td', 10)
        }
        
        self.timer = self.create_timer(0.01, self.publish_can_data)
        
    def publish_can_data(self):
        try:
            msg = self.can_reader.receive_message()
            if msg is None:
                self.get_logger().debug("Nenhuma mensagem CAN recebida", throttle_duration_sec=1)
                return

            can_data = self.can_reader.can_reader(msg)
            self.get_logger().debug(f"Dados CAN recebidos: {can_data}", throttle_duration_sec=1)

            self.publish_float_data(can_data)
            self.publish_uint8_data(can_data)
            self.publish_uint16_data(can_data)

            go_value = can_data.get("Go_State")
            if go_value == 1:
                go_msg = GoSignal()
                go_msg.mission = "trackdrive"
                self.go_publisher.publish(go_msg)
                self.get_logger().info("Sinal GO recebido - Modo 'trackdrive' ativado")

        except Exception as e:
            self.get_logger().warn(f"Erro durante processamento da mensagem CAN: {e}")

    def publish_float_data(self, can_data):
        float_fields = {
            'Steering_angle': '/can/steering_angle',
            'Motor_Temperature': '/can/motor_temp',
            'Motor_Torque': '/can/motor_torque',
            'Motor_RPM': '/can/motor_rpm',
            'Motor_Input_Power': '/can/motor_input_power',
            'Motor_Output_Power': '/can/motor_output_power',
            'Inverter_Current': '/can/inverter_current',
            'Inverter_Voltage': '/can/inverter_voltage',
            'Inverter_Temperature': '/can/inverter_temp',
            'Pressure_T': '/can/pressure_t',
            'Pressure_F': '/can/pressure_f',
            'Encoder_Front_Left': '/can/encoder_fl',
            'Encoder_Front_Right': '/can/encoder_fr',
            'Encoder_Rear_Left': '/can/encoder_rl',
            'Encoder_Rear_Right': '/can/encoder_rr',
            'BMS_Current': '/can/BMS_current',
            'HV_Voltage': '/can/hv_voltage',
            'Car_Speed': '/can/Car_speed',
            'Protection_Flags': '/can/protection_flags'
        }

        for field, topic in float_fields.items():
            try:
                if can_data.get(field) is not None:
                    msg = Float32()
                    msg.data = float(can_data[field])
                    self.float_publishers[topic].publish(msg)
                    self.get_logger().debug(f"Publicado {topic}: {msg.data}", throttle_duration_sec=1)
            except Exception as e:
                self.get_logger().warn(f"Erro ao publicar float '{field}': {e}")

    def publish_uint8_data(self, can_data):
        uint8_fields = {
            'Pedal_angle': '/can/pedal_angle',
            'Brake_Pedal': '/can/brake_pedal',
            'ACC_Pedal': '/can/ACC_pedal',
            'RES': '/can/res',
            'Task_Mode': '/can/task_mode',
            'Torque_Mode': '/can/torque_mode',
            'Cebolinha': '/can/cebolinha',
            'Avarage_Temperature': '/can/avarage_cell_temperature',
            'Max_Temperature': '/can/max_cell_temperature',
            'Estercamento_Atuador': '/can/estercamento_atuador'
        }

        for field, topic in uint8_fields.items():
            try:
                if can_data.get(field) is not None:
                    msg = UInt8()
                    msg.data = int(can_data[field]) & 0xFF
                    self.uint8_publishers[topic].publish(msg)
                    self.get_logger().debug(f"Publicado {topic}: {msg.data}", throttle_duration_sec=1)
            except Exception as e:
                self.get_logger().warn(f"Erro ao publicar uint8 '{field}': {e}")

    def publish_uint16_data(self, can_data):
        uint16_fields = {
            'Susp_TE': '/can/susp_te',
            'Susp_FD': '/can/susp_fd',
            'Susp_FE': '/can/susp_fe',
            'Susp_TD': '/can/susp_td'
        }

        for field, topic in uint16_fields.items():
            try:
                if can_data.get(field) is not None:
                    msg = UInt16()
                    msg.data = int(can_data[field]) & 0xFFFF
                    self.uint16_publishers[topic].publish(msg)
                    self.get_logger().debug(f"Publicado {topic}: {msg.data}", throttle_duration_sec=1)
            except Exception as e:
                self.get_logger().warn(f"Erro ao publicar uint16 '{field}': {e}")

    def control_callback(self, msg: ControlCommand):
        self.latest_control_command = msg
        self.send_reference()

    def send_reference(self):

        ref_values = {
            "Referencia": None,
            "Velocidade": None
        }
        
        if self.latest_control_command:
        
            float_steering_value = self.latest_control_command.steering
            # float_throttle_value = self.latest_control_command.throttle

            steering_fixed  = ((float_steering_value + 1) * 100)

            fixed_throttle_uint16 = 10000  

            clamped_steering = max(0.0, min(200.0, steering_fixed))
            steering_byte = int((clamped_steering)) & 0xFF
            
            velocidade_uint16 = fixed_throttle_uint16
            
            self.get_logger().info(f"Steering_byte: {steering_byte}")
            self.get_logger().info(f"Throttle uint16: {velocidade_uint16}")
            
            ref_values['Referencia'] = steering_byte
            ref_values['Velocidade'] = velocidade_uint16

        if ref_values:
            self.can_reader.send_references_throttle(ref_values)
            self.can_reader.send_references_steering(ref_values)
            self.get_logger().info(f"Enviado ao barramento CAN: {ref_values}")

def main(args=None):
    rclpy.init(args=args)
    node = CanToRosNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
