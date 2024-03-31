import wpilib
import commands2


# Note: Look here
# https://github.com/robotpy/examples/blob/main/GyroDriveCommands/commands/turntoangle.py
# https://docs.wpilib.org/en/stable/docs/software/pathplanning/trajectory-tutorial/characterizing-drive.html
# https://docs.wpilib.org/en/stable/docs/software/advanced-controls/index.html

# Get the rotation of the robot from the gyro.
# Put Wrist Into Shooting Position
# self.wrist.set(-0.004 * self.wristPID.calculate(self.wristEncoder.get(), -4500))
# Start Shooter Motors
# self.shooter.set(0.005 * self.shooterPID.calculate(self.shooterEncoder.getVelocity(), 3500) + self.shooterVValueSub.get())
# self.shooterHelper.set(-0.005 * self.shooterHelperPID.calculate(self.shooterHelperEncoder.getVelocity(), 3500) - self.shooterHelperVValueSub.get())

# If Wrist in position start arm movement
# if self.wristEncoder.get() <= -4400:
#     self.autoArmDownStart = True

# If Arm in position after wrist is in position shoot
# if self.autoArmDownStart and (self.armBuiltinEncoder.getPosition() <= 30):
#     self.intake.set(0.5)  # Spit note out of jaws

# if self.autoArmDownStart:
#     self.arm.set(self.armPID.calculate(self.armBuiltinEncoder.getPosition(), 0))
# else:
#     self.arm.set(self.armPID.calculate(self.armBuiltinEncoder.getPosition(), 120))
class IntakeSystem(commands2.Subsystem):
    def __init__(self, intakeMotor, maxPower=0.5):
        super().__init__()
        self.intakeMaxPower = maxPower
        self.motor = intakeMotor
        self.noteDirection = "Neither"

    def noteOut(self):
        self.noteDirection = "Out"

    def noteIn(self):
        self.noteDirection = "In"

    def off(self):
        self.noteDirection = "Neither"

    def periodic(self):
        if self.noteDirection == "In":
            self.motor.set(-self.intakeMaxPower)
        elif self.noteDirection == "Out":
            self.motor.set(self.intakeMaxPower)
        else:
            self.motor.set(0)


class IntakeOff(commands2.InstantCommand):
    def __init__(self, intake: IntakeSystem):
        super().__init__()
        self._sub = intake
        self.addRequirements(self._sub)

    def initialize(self):
        pass

    def execute(self):
        self.intake.off()


class IntakeNote(commands2.InstantCommand):
    def __init__(self, intake: IntakeSystem):
        super().__init__()
        self._sub = intake
        self.addRequirements(self._sub)

    def initialize(self):
        pass

    def execute(self):
        self.intake.noteIn()


class IntakeSpitNote(commands2.InstantCommand):
    def __init__(self, intake: IntakeSystem):
        super().__init__()
        self._sub = intake
        self.addRequirements(self._sub)

    def initialize(self):
        pass

    def execute(self):
        self.intake.noteOut()
