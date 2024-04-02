
class ActuatorSystemModeManager:
    def __init__(self):
        self.Mode = None
        self.Idle = 0
        self.Intaking = 1
        self.Shooting = 2
        self.Amping = 3
        self.Reset = 4

    def toggleIntaking(self):
        if self.Mode == self.Idle:
            self.Mode = self.Intaking
        else:
            self.Mode = self.Idle

    def toggleReset(self):
        if self.Mode == self.Idle:
            self.Mode = self.Reset
        else:
            self.Mode = self.Idle

    def toggleAmping(self):
        if self.Mode == self.Idle:
            self.Mode = self.Amping
        else:
            self.Mode = self.Idle

    def setIdle(self):
        self.Mode = self.Idle

    def toggleShooting(self):
        if self.Mode == self.Idle:
            self.Mode = self.Shooting
        else:
            self.Mode = self.Idle
