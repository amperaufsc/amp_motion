class KLS_Lateral_Motion_Controller_Gains:
    def __init__(self, lateral_position_error_gain, orientation_error_gain):
        self.lateral_position_error_gain = lateral_position_error_gain
        self.orientation_error_gain = orientation_error_gain
