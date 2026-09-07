# adapters/motion_adapter.py

class MotionAdapter:
    """
    Stub for robot motion state.
    Later the motion team can replace internals with serial / ROS / socket integration.
    """

    def __init__(self):
        self.moving = False
        self.arrived = False
        self.fault = False

    def get_state(self):
        return {
            "moving": self.moving,
            "arrived": self.arrived,
            "fault": self.fault,
        }

    def set_moving(self):
        self.moving = True
        self.arrived = False
        self.fault = False

    def set_arrived(self):
        self.moving = False
        self.arrived = True
        self.fault = False

    def set_fault(self):
        self.moving = False
        self.arrived = False
        self.fault = True

    def set_idle(self):
        self.moving = False
        self.arrived = False
        self.fault = False