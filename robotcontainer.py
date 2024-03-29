from pathplannerlib.auto import PathPlannerAuto

class RobotContainer:
    def getAutonomousCommand(self):
        return PathPlannerAuto('SamAuto')
