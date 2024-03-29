from pathplannerlib.auto import PathPlannerAuto, NamedCommands, AutoBuilder
import pathplannerlib.auto
from wpilib import SmartDashboard
from drivetrain import Drivetrain


class RobotContainer:
    def __init__(self):
        self.mechanumSubsystem = Drivetrain()
        self.configureBindings()
        self.autoChooser = pathplannerlib.auto.AutoBuilder.buildAuto("SamAuto")
        SmartDashboard.putData("Auto Mode", self.autoChooser)

    def configureBindings(self):
        pass

    def getAutonomousCommand(self):
        return self.autoChooser
