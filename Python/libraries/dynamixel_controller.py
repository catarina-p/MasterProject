import sys
from dynamixel_sdk import *
from libraries.dynamixel_address_book import *

class dynamixel:
    
    def __init__(self, ID, descriptive_device_name, port_name, baudrate, series_name = "xm"):
        # Communication inputs
        self.ID = ID
        self.descriptive_device_name = descriptive_device_name
        self.series_name = series_name # This information may be used later to specify register addresses
        self.port_name = port_name
        self.baudrate = baudrate

        # Check for series name
        all_series_names = ["xm", "xl"]
        if self.series_name not in all_series_names:
            print("Series name invalid. Choose one of:", all_series_names)
            sys.exit()

        # Communication settings
        self.port_handler = PortHandler(self.port_name)
        self.packet_handler = PacketHandler(2)

    def begin_communication(self, enable_torque = True):
        # Open port
        try: 
            self.port_handler.openPort()
            print("Port open successfully for:", self.descriptive_device_name)
        except:
            print("!! Failed to open port for:", self.descriptive_device_name)
            print("Check: \n1. If correct port name is specified\n2. If dynamixel wizard isn't connected")
            sys.exit()

        # Set port baudrate
        try:
            self.port_handler.setBaudRate(self.baudrate)
            print("Baudrate set successfully for:", self.descriptive_device_name)
        except:
            print("!! Failed to set baudrate for:", self.descriptive_device_name)
            sys.exit()

        if enable_torque:
            self.enable_torque()

    def end_communication(self, disable_torque = True):
        if disable_torque:
            self.disable_torque()

        # Close port
        try: 
            self.port_handler.closePort()
            print("Port closed successfully for:", self.descriptive_device_name)
        except:
            print("!! Failed to close port for:", self.descriptive_device_name)
            sys.exit()

    def _print_error_msg(self, process_name, dxl_comm_result, dxl_error, print_only_if_error = False):
        if dxl_comm_result != COMM_SUCCESS:
            print("!!", process_name, "failed for:", self.descriptive_device_name)
            print("Communication error:", self.packet_handler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("!!", process_name, "failed for:", self.descriptive_device_name)
            print("Dynamixel error:", self.packet_handler.getRxPacketError(dxl_error))
        else:
            if not print_only_if_error:
                print(process_name, "successful for:", self.descriptive_device_name, "ID:", self.ID)

    def enable_torque(self, print_only_if_error=False):
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, self.ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        self._print_error_msg("Torque enable", dxl_comm_result=dxl_comm_result, dxl_error=dxl_error, print_only_if_error=print_only_if_error)

    def disable_torque(self, print_only_if_error=False):
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, self.ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        self._print_error_msg("Torque disable", dxl_comm_result=dxl_comm_result, dxl_error=dxl_error, print_only_if_error=print_only_if_error)
    
    def is_torque_on(self, print_only_if_error=False):
        torque_status, dxl_comm_result, dxl_error = self.packet_handler.read1ByteTxRx(self.port_handler, self.ID, ADDR_TORQUE_ENABLE)
        self._print_error_msg("Read torque status", dxl_comm_result=dxl_comm_result, dxl_error=dxl_error, print_only_if_error=print_only_if_error)
        return torque_status

    def ping(self):
        _, dxl_comm_result, dxl_error = self.packet_handler.ping(self.port_handler, self.ID)
        self._print_error_msg("Ping", dxl_comm_result=dxl_comm_result, dxl_error=dxl_error)

    def set_operating_mode(self, mode):

        if self.series_name == "xm": 
            operating_modes = operating_modes_xm
        elif self.series_name == "xl":
            operating_modes = operating_modes_xl

        if mode in operating_modes:
            # Check if torque was enabled
            was_torque_on = False
            if self.is_torque_on(print_only_if_error = True):
                was_torque_on = True
                self.disable_torque(print_only_if_error = True)

            mode_id = operating_modes[mode]
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, self.ID, ADDR_OPERATING_MODE, mode_id)
            self._print_error_msg("Mode set to " + mode + " control", dxl_comm_result=dxl_comm_result, dxl_error=dxl_error)

            if was_torque_on:
                self.enable_torque(print_only_if_error=True)
        else:
            print("Enter valid operating mode. Select one of:\n" + str(list(operating_modes.keys())))

    def compensate_twos_complement(self, value, quantity):
        if quantity in max_register_value:
            max_value = max_register_value[quantity]

            if value < max_value/2:
                return value
            else:
                return value - max_value
        else:
            print("Enter valid operating mode. Select one of:\n" + str(list(max_register_value.keys())))

    def read_position(self):
        position, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(self.port_handler, self.ID, ADDR_PRESENT_POSITION)
        self._print_error_msg("Read position", dxl_comm_result=dxl_comm_result, dxl_error=dxl_error, print_only_if_error=True)
        return self.compensate_twos_complement(position, "position")

    def read_velocity(self):
        velocity, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(self.port_handler, self.ID, ADDR_PRESENT_VELOCITY)
        self._print_error_msg("Read velocity", dxl_comm_result=dxl_comm_result, dxl_error=dxl_error, print_only_if_error=True)
        return self.compensate_twos_complement(velocity, "velocity")

    def read_current(self):
        current, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(self.port_handler, self.ID, ADDR_PRESENT_CURRENT)
        self._print_error_msg("Read cuurent", dxl_comm_result=dxl_comm_result, dxl_error=dxl_error, print_only_if_error=True)
        return self.compensate_twos_complement(current, "current")

    def read_pwm(self):
        pwm, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(self.port_handler, self.ID, ADDR_PRESENT_PWM)
        self._print_error_msg("Read pwm", dxl_comm_result=dxl_comm_result, dxl_error=dxl_error, print_only_if_error=True)
        return self.compensate_twos_complement(pwm, "pwm")

    def write_position(self, pos):
        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(self.port_handler, self.ID, ADDR_GOAL_POSITION, int(pos))
        self._print_error_msg("Write position", dxl_comm_result=dxl_comm_result, dxl_error=dxl_error, print_only_if_error=True)

    def write_velocity(self, vel):
        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(self.port_handler, self.ID, ADDR_GOAL_VELOCITY, int(vel))
        self._print_error_msg("Write velocity", dxl_comm_result=dxl_comm_result, dxl_error=dxl_error, print_only_if_error=True)
    
    def write_current(self, current):
        dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(self.port_handler, self.ID, ADDR_GOAL_CURRENT, int(current))
        self._print_error_msg("Write current", dxl_comm_result=dxl_comm_result, dxl_error=dxl_error, print_only_if_error=True)
   
    def write_pwm(self, pwm):
        dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(self.port_handler, self.ID, ADDR_GOAL_PWM, int(pwm))
        self._print_error_msg("Write pwm", dxl_comm_result=dxl_comm_result, dxl_error=dxl_error, print_only_if_error=True)