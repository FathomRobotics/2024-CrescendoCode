from pathplannerlib.auto import PathPlannerAuto, NamedCommands, AutoBuilder
from wpilib import SmartDashboard
from drivetrain import Drivetrain


class RobotContainer:
    def __init__(self):
        self.mechanumSubsystem = Drivetrain()
        self.configureBindings()
        self.autoChooser = AutoBuilder.buildAutoChooser("SamAuto")
        SmartDashboard.putData("Auto Mode", self.autoChooser)

    def configureBindings(self):
        pass

    def getAutonomousCommand(self):
        return self.autoChooser.getSelected()
