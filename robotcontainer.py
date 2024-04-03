import commands2
import wpilib
from pathplannerlib.auto import PathPlannerAuto, NamedCommands, AutoBuilder
import pathplannerlib.auto
from wpilib import SmartDashboard
from drivetrain import Drivetrain


class RobotContainer:
    def __init__(self, mecanumSubsystem, stopIntakeFunction, wristFunction, ender2):
        self.mechanumSubsystem = mecanumSubsystem
        self.wristFunction = wristFunction
        self.ender2 = ender2
        self.configureBindings()
        NamedCommands.registerCommand("intakeOff", Intake(stopIntakeFunction))
        NamedCommands.registerCommand("endGame", EndRobotAuto(self.wristFunction))
        self.autoChooser = wpilib.SendableChooser()
        self.autoChooser.setDefaultOption("Amp Side", pathplannerlib.auto.AutoBuilder.buildAuto("AmpAuto"))
        self.autoChooser.addOption("Testing", pathplannerlib.auto.AutoBuilder.buildAuto("Testing"))
        self.autoChooser.addOption("Center Auto", pathplannerlib.auto.AutoBuilder.buildAuto("CenterAuto"))
        self.autoChooser.addOption("Stage Side", pathplannerlib.auto.AutoBuilder.buildAuto("StageAuto"))
        self.autoChooser.addOption("Amp Side", pathplannerlib.auto.AutoBuilder.buildAuto("AmpAuto"))
        NamedCommands.registerCommand("stop", StopRobotAuto(self.mechanumSubsystem, self.autoChooser.getSelected().cancel, self.ender2))
        SmartDashboard.putData("Auto Mode", self.autoChooser)

    def configureBindings(self):
        pass

    def getAutonomousCommand(self):
        return self.autoChooser.getSelected()


class StopRobotAuto(commands2.InstantCommand):
    def __init__(self, base, ender, ender2):
        super().__init__()
        self._sub = base
        self._sub2 = ender
        self._sub3 = ender2
        self.addRequirements(self._sub, self._sub2, self._sub3)

    def initialize(self):
        pass

    def execute(self):
        self._sub3()
        self._sub2()
        self._sub.stopRobot()


class EndRobotAuto(commands2.InstantCommand):
    def __init__(self, wristFunction):
        super().__init__()
        self._sub2 = wristFunction
        self.addRequirements(self._sub2)

    def initialize(self):
        pass

    def execute(self):
        self._sub2()


class Intake(commands2.InstantCommand):
    def __init__(self, stopIntakeFunction):
        super().__init__()
        self._sub = stopIntakeFunction
        self.addRequirements(self._sub)

    def initialize(self):
        pass

    def execute(self):
        self._sub()

