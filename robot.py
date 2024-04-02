#!/usr/bin/env python3
#
# Copyright (c) Fathom Robotics
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import math
import threading
import time

import commands2
import navx
import ntcore
import phoenix5
import phoenix5.sensors
import rev
import wpilib
import wpilib.drive
import wpimath
import pathplannerlib
from wpimath.geometry import Translation2d
from wpimath.kinematics import MecanumDriveKinematics
from wpimath.kinematics import MecanumDriveWheelPositions
from wpimath.kinematics import MecanumDriveOdometry
from wpimath.geometry import Pose2d
from wpimath.geometry import Rotation2d
from wpimath.controller import PIDController
import pathplannerlib
import pathplannerlib.path
import wpimath.filter
import cscore

from drivetrain import Drivetrain
from robotcontainer import RobotContainer
from actuatorsystemmodemanager import ActuatorSystemModeManager
# python -m robotpy deploy --nc --skip-tests


class MyRobot(commands2.TimedCommandRobot):
    """
    This is a demo program showing how to use Mecanum control with the
    MecanumDrive class.
    """

    def robotInit(self):
        """Robot initialization function"""
        cscore.CameraServer.startAutomaticCapture()
        # Drivetrain PID Network table Values
        self.drivetrainNetworkTable = ntcore.NetworkTableInstance.getDefault().getTable("DriveTrainPID")

        # Set Persistence
        self.drivetrainNetworkTable.getDoubleTopic("kPn").setPersistent(True)
        self.drivetrainNetworkTable.getDoubleTopic("kIn").setPersistent(True)
        self.drivetrainNetworkTable.getDoubleTopic("kDn").setPersistent(True)
        self.drivetrainNetworkTable.getDoubleTopic("kMn").setPersistent(True)

        # Publish
        self.kPnPub = self.drivetrainNetworkTable.getDoubleTopic("kPn").publish()
        self.kPnPub.set(3)
        self.kInPub = self.drivetrainNetworkTable.getDoubleTopic("kIn").publish()
        self.kInPub.set(0)
        self.kDnPub = self.drivetrainNetworkTable.getDoubleTopic("kDn").publish()
        self.kDnPub.set(0)
        self.kMnPub = self.drivetrainNetworkTable.getDoubleTopic("kMn").publish()
        self.kMnPub.set(2)

        # Subscribe
        self.kPnSub = self.drivetrainNetworkTable.getDoubleTopic("kPn").subscribe(3)
        self.kInSub = self.drivetrainNetworkTable.getDoubleTopic("kIn").subscribe(0)
        self.kDnSub = self.drivetrainNetworkTable.getDoubleTopic("kDn").subscribe(0)
        self.kMnSub = self.drivetrainNetworkTable.getDoubleTopic("kMn").subscribe(2)
        self.kPnSub.get()
        self.kInSub.get()
        self.kDnSub.get()
        self.kMnSub.get()

        # Sticky Vars
        self.armEncoderReseting = False
        self.armPositionChanged = False
        self.autoArmDownStart = False
        self.teleautoArmUp = False
        self.teleautoWristOut = False
        self.inStartingPosition = True  # Assume starting in starting position
        self.isTuningDrivePID = False
        self.shooterOnAuto = True
        self.runAuto = False
        self.autoRan = False
        self.intakeOffAuto = False
        self.autoEndWristVal = False
        self.disableAutoWheels = False

        # Kinematics and Odometry
        # Locations of the wheels relative to the robot center.
        # 21.375in Wide
        # 20.5in Long
        wpilib.reportWarning("STARTING!!!", False)
        wpilib.SmartDashboard.init()
        self.field2d = wpilib.Field2d()
        # Drive Encoders
        # (360/2048) degrees per pulse
        # 6in wheel
        # 0.1524pi/2048
        a = 2
        b = 4.36
        distancePerPulse = (a/b)/2048
        # Wheels have a radius
        self.frontLeftMotorEncoder = wpilib.Encoder(0, 1, False)
        self.frontLeftMotorEncoder.setDistancePerPulse(distancePerPulse)
        self.rearLeftMotorEncoder = wpilib.Encoder(2, 3, False)
        self.rearLeftMotorEncoder.setDistancePerPulse(distancePerPulse)
        self.frontRightMotorEncoder = wpilib.Encoder(4, 5, True)
        self.frontRightMotorEncoder.setDistancePerPulse(distancePerPulse)
        self.rearRightMotorEncoder = wpilib.Encoder(6, 7, True)
        self.rearRightMotorEncoder.setDistancePerPulse(distancePerPulse)

        self.mecanum = Drivetrain(
            self.frontLeftMotorEncoder,
            self.rearLeftMotorEncoder,
            self.frontRightMotorEncoder,
            self.rearRightMotorEncoder,
            self.kPnSub,
            self.kInSub,
            self.kDnSub,
            self.kMnSub
        )

        self.drive = wpilib.drive.MecanumDrive(
            self.mecanum.frontLeftMotor,
            self.mecanum.rearLeftMotor,
            self.mecanum.frontRightMotor,
            self.mecanum.rearRightMotor,
        )

        self.robotContainer = RobotContainer(self.mecanum, self.intakeOffAutoFunction, self.autoEndWristFunction, self.ender2)
        # self.mecanum.resetPose(wpimath.geometry.Pose2d(2.84, 5.52, 0))
        # Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
        self.xspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.yspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.rotLimiter = wpimath.filter.SlewRateLimiter(3)

        # NavX Initialization
        self.gyro = navx.AHRS(wpilib.SPI.Port.kMXP)
        gyroThread = threading.Thread(target=self.resetGryoThread)
        gyroThread.run()

        # DIO Encoders
        self.wristEncoder = wpilib.Encoder(23, 22)

        # Novelty Battery Percentage
        self.minVoltage = 11.9
        self.maxVoltage = 13
        self.maxCurrentDrawWhenCheckingPercentage = 0.05
        self._maxmandiffVoltage = self.maxVoltage - self.minVoltage
        if self._maxmandiffVoltage <= 0:
            wpilib.reportWarning("Min Voltage Variable is Larger or Equal to Max", printTrace=False)

        self.powerDistribution = wpilib.PowerDistribution(module=6, moduleType=wpilib.PowerDistribution.ModuleType.kRev)

        # Control Devices
        self.pidgen = phoenix5.sensors.Pigeon2(5)
        self.intake = rev.CANSparkMax(7, type=rev.CANSparkLowLevel.MotorType.kBrushless)
        self.arm = rev.CANSparkMax(8, type=rev.CANSparkLowLevel.MotorType.kBrushless)
        self.wrist = rev.CANSparkMax(9, type=rev.CANSparkLowLevel.MotorType.kBrushless)
        self.shooter = rev.CANSparkMax(10, type=rev.CANSparkLowLevel.MotorType.kBrushless)
        self.shooterHelper = rev.CANSparkMax(21, type=rev.CANSparkLowLevel.MotorType.kBrushless)
        self.pnumaticsHub = wpilib.PneumaticHub(17)

        # Enbeded Encoders
        self.intakeEncoder = self.intake.getEncoder()
        self.shooterEncoder = self.shooter.getEncoder()
        self.shooterHelperEncoder = self.shooterHelper.getEncoder()
        self.armBuiltinEncoder = self.arm.getEncoder()

        # Limit Switch
        self.armDownLimitSwitch = wpilib.DigitalInput(9)
        self.armUpLimitSwitch = wpilib.DigitalInput(8)

        # Pneumatics Hub Devices
        self.solenoidRed = self.pnumaticsHub.makeSolenoid(0)
        self.solenoidBlue = self.pnumaticsHub.makeSolenoid(1)
        self.compressor = self.pnumaticsHub.makeCompressor()
        self.compressor.isEnabled()
        # self.frontLeftMotor.setInverted(True)

        # Network Tables Initialization
        inst = ntcore.NetworkTableInstance.getDefault()
        table = inst.getTable("SmartDashboard")
        # TODO: Implement Field 2d View
        # TODO: Implement Speedometer
        # TODO: Implement warning lights (Power Usage, Arm Limits, PDH Temp)
        self.fieldPose = table.getStringTopic("fieldPose").publish()
        self.wristEncoderNetwork = table.getDoubleTopic("wristEncoderRaw").publish()
        self.armDownLimitSwitchNetwork = table.getBooleanTopic("ArmDownLimitSwitch").publish()
        self.armUpLimitSwitchNetwork = table.getBooleanTopic("ArmUpLimitSwitch").publish()
        self.GyroPub = table.getDoubleTopic("Gyro-Degrees").publish()  # Gyro Value
        self.GryoConnected = table.getBooleanTopic("NavX-Connected").publish()  # NavX Connected?
        self.shooterSpunUp = table.getBooleanTopic("Shooter-Ready").publish()  # Shooter Ready
        self.shooterRPM = table.getDoubleTopic("ShooterRPM").publish()
        self.shooterHelperRPM = table.getDoubleTopic("ShooterHelperRPM").publish()
        self.PidgeonCompass = table.getDoubleTopic("Pigeon-Degrees").publish()  # Gyro Pigeon Value
        self.armPVar = table.getDoubleTopic("Arm_P").publish()
        self.armIVar = table.getDoubleTopic("Arm_I").publish()
        self.armDVar = table.getDoubleTopic("Arm_D").publish()
        self.armPVar.set(0.01)
        self.armIVar.set(0)
        self.armDVar.set(0)
        self.armSPVar = table.getDoubleTopic("Arm_SP").publish()
        self.armPVarSub = self.armPVar.getTopic().subscribe(0)
        self.armIVarSub = self.armIVar.getTopic().subscribe(0)
        self.armDVarSub = self.armDVar.getTopic().subscribe(0)
        self.armSPVarSub = self.armSPVar.getTopic().subscribe(0)
        self.armEncoderValueNet = table.getDoubleTopic("ArmEncoder").publish()
        self.shooterVValue = table.getDoubleTopic("ShooterVValue").publish()
        self.shooterVValue.set(0.60)
        self.shooterPValue = table.getDoubleTopic("ShooterPValue").publish()
        self.shooterPValue.set(0.0125)
        self.shooterIValue = table.getDoubleTopic("ShooterIValue").publish()
        self.shooterIValue.set(0)
        self.shooterDValue = table.getDoubleTopic("ShooterDValue").publish()
        self.shooterDValue.set(0)
        self.shooterVValueSub = table.getDoubleTopic("ShooterVValue").subscribe(0)
        self.shooterPValueSub = table.getDoubleTopic("ShooterPValue").subscribe(0)
        self.shooterIValueSub = table.getDoubleTopic("ShooterIValue").subscribe(0)
        self.shooterDValueSub = table.getDoubleTopic("ShooterDValue").subscribe(0)

        self.shooterHelperVValue = table.getDoubleTopic("ShooterVValue").publish()
        self.shooterHelperVValue.set(0.60)
        self.shooterHelperPValue = table.getDoubleTopic("ShooterPValue").publish()
        self.shooterHelperPValue.set(0.0125)
        self.shooterHelperIValue = table.getDoubleTopic("ShooterIValue").publish()
        self.shooterHelperIValue.set(0)
        self.shooterHelperDValue = table.getDoubleTopic("ShooterDValue").publish()
        self.shooterHelperDValue.set(0)

        self.fieldposeNet = table.getStringTopic("self.mecanum.getPose()").publish()

        self.wristP = table.getDoubleTopic("WristP").publish()
        self.wristP.set(0.05)
        self.wristI = table.getDoubleTopic("WristI").publish()
        self.wristI.set(0)
        self.wristD = table.getDoubleTopic("WristD").publish()
        self.wristD.set(0)
        self.wristSP = table.getDoubleTopic("WristSP").publish()
        self.wristSP.set(0)

        self.wristPSub = table.getDoubleTopic("WristP").subscribe(0)
        self.wristISub = table.getDoubleTopic("WristI").subscribe(0)
        self.wristDSub = table.getDoubleTopic("WristD").subscribe(0)
        self.wristSPSub = table.getDoubleTopic("WristSP").subscribe(0)

        self.shooterHelperPValueSub = self.shooterHelperPValue.getTopic().subscribe(0.05)
        self.shooterHelperIValueSub = self.shooterHelperIValue.getTopic().subscribe(0)
        self.shooterHelperDValueSub = self.shooterHelperDValue.getTopic().subscribe(0)
        self.shooterHelperVValueSub = self.shooterHelperVValue.getTopic().subscribe(0.75)
        self.BatteryPercentageEstimationTopic = table.getDoubleTopic("BatteryPercentageEstimation").publish()  # Battery
        self.frontLeftMotorEncoderNetworkTopic = table.getDoubleTopic("frontLeftMotorEncoder").publish()
        self.rearLeftMotorEncoderNetworkTopic = table.getDoubleTopic("rearLeftMotorEncoder").publish()
        self.frontRightMotorEncoderNetworkTopic = table.getDoubleTopic("frontRightMotorEncoder").publish()
        self.rearRightMotorEncoderNetworkTopic = table.getDoubleTopic("rearRightMotorEncoder").publish()
        self.frontLeftMotorTempPub = table.getDoubleTopic("frontLeftMotorTemp").publish()
        self.frontRightMotorTempPub = table.getDoubleTopic("frontRightMotorTemp").publish()
        self.rearLeftMotorTempPub = table.getDoubleTopic("rearLeftMotorTemp").publish()
        self.rearRightMotorTempPub = table.getDoubleTopic("rearRightMotorTemp").publish()
        self.wristTempPub = table.getDoubleTopic("WristTemp").publish()
        self.armTempPub = table.getDoubleTopic("ArmTemp").publish()
        self.PidgeonCompass.set(self.pidgen.getCompassHeading())
        self.GryoConnected.set(False)

        # Intake Encoder Value
        self.intakeEncoderNet = table.getDoubleTopic("IntakeEncoder").publish()

        # Drive PID
        self.drivePPub = table.getDoubleTopic("DriveP").publish()
        self.drivePPub.set(self.mecanum.kP)
        self.driveIPub = table.getDoubleTopic("DriveI").publish()
        self.driveIPub.set(self.mecanum.kI)
        self.driveDPub = table.getDoubleTopic("DriveD").publish()
        self.driveDPub.set(self.mecanum.kD)
        self.drivePSub = table.getDoubleTopic("DriveP").subscribe(0)
        self.driveISub = table.getDoubleTopic("DriveI").subscribe(0)
        self.driveDSub = table.getDoubleTopic("DriveD").subscribe(0)
        self.driveFeedforwardPub = table.getDoubleTopic("DriveFeedforward").publish()
        self.driveFeedforwardPub.set(self.mecanum.feedforwardValue)
        self.driveFeedforwardSub = table.getDoubleTopic("DriveFeedforward").subscribe(0)

        self.chassisSpeedsPublisher = table.getStringTopic("ChassisSpeed").publish()

        # Define the Controller
        self.driver1 = wpilib.XboxController(0)
        self.driver2 = wpilib.XboxController(1)
        self.stickXYToggle = False
        self.shooterOn = False

        # PID Variable Declaration
        self.armPID = PIDController(
            self.armPVarSub.get(),
            self.armIVarSub.get(),
            self.armDVarSub.get()
        )
        self.shooterPID = PIDController(
            self.shooterPValueSub.get(),
            self.shooterIValueSub.get(),
            self.shooterDValueSub.get()
        )
        self.shooterHelperPID = PIDController(
            self.shooterHelperPValueSub.get(),
            self.shooterHelperIValueSub.get(),
            self.shooterHelperDValueSub.get()
        )
        self.wristPID = PIDController(
            self.wristPSub.get(),
            self.wristISub.get(),
            self.wristDSub.get()
        )
        # Arm Positions (0 is Down, 120 is Up)
        self.armPositions = [0, 120, 198]
        self.currentArmPosition = 1
        self.armSPVar.set(120)

        # Wrist Positions (-1000 is Intake, -4500 is Idle)
        self.wristPositions = [-1500, -4500, 0, -4200]
        self.currentWristPosition = 1
        self.wristSP.set(0)

        # Actuator Mode
        self.actuatorMode = ActuatorSystemModeManager()
        self.actuatorMode.setIdle()

    def robotPeriodic(self):
        self.wristTempPub.set((self.wrist.getMotorTemperature()*(9/5))+32)
        self.armTempPub.set((self.arm.getMotorTemperature()*(9/5))+32)
        self.frontLeftMotorTempPub.set(self.mecanum.frontLeftMotor.getTemperature())
        self.frontRightMotorTempPub.set(self.mecanum.frontRightMotor.getTemperature())
        self.rearLeftMotorTempPub.set(self.mecanum.rearLeftMotor.getTemperature())
        self.rearRightMotorTempPub.set(self.mecanum.rearRightMotor.getTemperature())
        self.intakeEncoderNet.set(self.intakeEncoder.getPosition())
        self.armDownLimitSwitchNetwork.set(self.armDownLimitSwitch.get())
        self.armUpLimitSwitchNetwork.set(self.armUpLimitSwitch.get())
        self.wristEncoderNetwork.set(self.wristEncoder.get())
        self.chassisSpeedsPublisher.set(str(self.mecanum.getCurrentSpeeds()))
        self.drivePPub.set(self.mecanum.frontLeftPIDController.getP())
        if self.isTuningDrivePID:
            wpilib.reportWarning("---------- TUNING DRIVE PID ------------")
            self.mecanum.feedforward = wpimath.controller.SimpleMotorFeedforwardMeters(self.driveFeedforwardSub.get(),
                                                                                       self.driveFeedforwardSub.get())
            # Front Left Controller
            self.mecanum.frontLeftPIDController.setP(self.drivePSub.get())
            self.mecanum.frontLeftPIDController.setI(self.driveISub.get())
            self.mecanum.frontLeftPIDController.setD(self.driveDSub.get())
            # Front Right Controller
            self.mecanum.frontRightPIDController.setP(self.drivePSub.get())
            self.mecanum.frontRightPIDController.setI(self.driveISub.get())
            self.mecanum.frontRightPIDController.setD(self.driveDSub.get())
            # Rear Left Controller
            self.mecanum.rearLeftPIDController.setP(self.drivePSub.get())
            self.mecanum.rearLeftPIDController.setI(self.driveISub.get())
            self.mecanum.rearLeftPIDController.setD(self.driveDSub.get())
            # Rear Right Controller
            self.mecanum.rearRightPIDController.setP(self.drivePSub.get())
            self.mecanum.rearRightPIDController.setI(self.driveISub.get())
            self.mecanum.rearRightPIDController.setD(self.driveDSub.get())

        self.mecanum.updateOdometry()
        self.frontLeftMotorEncoderNetworkTopic.set(self.frontLeftMotorEncoder.getDistance())
        self.rearLeftMotorEncoderNetworkTopic.set(self.rearLeftMotorEncoder.getDistance())
        self.frontRightMotorEncoderNetworkTopic.set(self.frontRightMotorEncoder.getDistance())
        self.rearRightMotorEncoderNetworkTopic.set(self.rearRightMotorEncoder.getDistance())
        self.field2d.setRobotPose(self.mecanum.getPose())
        wpilib.SmartDashboard.putData("Field", self.field2d)

    def resetGryoThread(self):
        time.sleep(1)
        self.gyro.reset()

    def getAutonomousCommand(self):
        return self.robotContainer.getAutonomousCommand()

    def autonomousInit(self):
        self.runAuto = False
        self.shooterOnAuto = True
        self.autoEndWristVal = False
        self.disableAutoWheels = False
        self.autoRan = False
        self.intakeEncoder.setPosition(0)
        if self.inStartingPosition:
            self.armBuiltinEncoder.setPosition(198)
            self.wristEncoder.reset()
            self.inStartingPosition = False
        self.mecanum.breakMotors()

    def autonomousPeriodic(self):
        # Note: Look here
        # https://github.com/robotpy/examples/blob/main/GyroDriveCommands/commands/turntoangle.py
        # https://docs.wpilib.org/en/stable/docs/software/pathplanning/trajectory-tutorial/characterizing-drive.html
        # https://docs.wpilib.org/en/stable/docs/software/advanced-controls/index.html

        # Get the rotation of the robot from the gyro.
        # Put Wrist Into Shooting Position
        # Start Shooter Motors
        if self.shooterOnAuto:
            self.shooter.set(0.005 * self.shooterPID.calculate(self.shooterEncoder.getVelocity(),
                                                               3500) + self.shooterVValueSub.get())
            self.shooterHelper.set(-0.005 * self.shooterHelperPID.calculate(self.shooterHelperEncoder.getVelocity(),
                                                                            3500) - self.shooterHelperVValueSub.get())
        else:
            self.shooter.set(0)
            self.shooterHelper.set(0)

        # If Wrist in position start arm movement
        if self.wristEncoder.get() <= -4400:
            self.autoArmDownStart = True

        # If Arm in position after wrist is in position shoot
        if self.autoArmDownStart and (self.armBuiltinEncoder.getPosition() <= 30):
            if self.intakeEncoder.getPosition() > 150:
                if not self.autoEndWristVal:
                    self.wrist.set(-0.004 * self.wristPID.calculate(self.wristEncoder.get(), -3000))
                self.shooterOnAuto = False
                self.runAuto = True
            else:
                self.intake.set(1)  # Spit note out of jaws
        else:
            if not self.autoEndWristVal:
                self.wrist.set(-0.004 * self.wristPID.calculate(self.wristEncoder.get(), -4500))

        if self.runAuto and (self.autoRan is False):
            self.getAutonomousCommand().schedule()
            self.autoRan = True

        if self.runAuto:
            if not self.intakeOffAuto:
                self.intake.set(-0.6)
            else:
                self.shooterOnAuto = True
                self.mecanum.rearLeftMotor.set(0.125)
                self.mecanum.rearRightMotor.set(0.125)
                self.mecanum.frontRightMotor.set(0.125)
                self.mecanum.frontLeftMotor.set(0.125)
                if self.autoEndWristVal:
                    self.wrist.set(-0.004 * self.wristPID.calculate(self.wristEncoder.get(), -4500))
                if (self.wristEncoder.get() <= -4400) and self.autoEndWristVal:
                    self.intake.set(1)
                    if self.intakeEncoder.getPosition() > 200:
                        self.mecanum.rearLeftMotor.set(-0.25)
                        self.mecanum.rearRightMotor.set(-0.25)
                        self.mecanum.frontRightMotor.set(-0.25)
                        self.mecanum.frontLeftMotor.set(-0.25)
                else:
                    self.intakeEncoder.setPosition(0)
                    self.intake.set(0)

        if self.autoArmDownStart:
            if self.armDownLimitSwitch.get() is False:
                self.arm.set(0.05)
                if self.armEncoderReseting is False:
                    self.armBuiltinEncoder.setPosition(0)
                self.armEncoderReseting = True
            elif self.armUpLimitSwitch.get() is False:
                self.arm.set(-0.125)
            else:
                self.armEncoderReseting = False
                self.arm.set(self.armPID.calculate(self.armBuiltinEncoder.getPosition(), 0))
        else:
            if self.armDownLimitSwitch.get() is False:
                self.arm.set(0.05)
                if self.armEncoderReseting is False:
                    self.armBuiltinEncoder.setPosition(0)
                self.armEncoderReseting = True
            elif self.armUpLimitSwitch.get() is False:
                self.arm.set(-0.125)
            else:
                self.armEncoderReseting = False
                self.arm.set(self.armPID.calculate(self.armBuiltinEncoder.getPosition(), 120))

    def teleopInit(self):
        # PID Variable Declaration
        self.armPID = PIDController(
            self.armPVarSub.get(),
            self.armIVarSub.get(),
            self.armDVarSub.get()
        )

        # Start Compressor
        if self.compressor.getPressure() > 60:
            self.compressor.disable()
        else:
            self.compressor.enableDigital()

        self.compressor.enableDigital()

        # Reset Encoders
        # self.frontLeftMotorEncoder.reset()
        # self.rearLeftMotorEncoder.reset()
        # self.frontRightMotorEncoder.reset()
        # self.rearRightMotorEncoder.reset()
        # TODO: Make sure that this encoder is not reset in a match (IF FMS ATTACHED)
        if self.inStartingPosition:
            self.armBuiltinEncoder.setPosition(198)
            self.inStartingPosition = False
            self.wristEncoder.reset()

        self.actuatorMode.setIdle()  # Set the Actuator mode to Idle

        self.mecanum.breakMotors()  # Make Motors Coast at start

        # self.arm.setIdleMode(self.arm.IdleMode.kBrake)  # Enable Arm Breaking

    def teleopPeriodic(self):
        """Runs the motors with Mecanum drive."""
        # Break Button (Driver 1 B)
        if self.driver1.getBButton():
            self.mecanum.coastMotors()
        else:
            self.mecanum.breakMotors()

        # Intake Button
        if self.driver2.getBButtonReleased():
            self.actuatorMode.toggleIntaking()

        # Amp Button
        if self.driver2.getAButtonReleased():
            self.actuatorMode.toggleAmping()

        # Shoot Button
        if self.driver2.getXButtonReleased():
            self.actuatorMode.toggleShooting()

        # Reset Button
        if self.driver2.getYButtonReleased():
            self.actuatorMode.toggleReset()

        # Toggle Inverted Base Controls

        if self.driver1.getLeftStickButtonReleased():
            self.stickXYToggle = not self.stickXYToggle

        if self.stickXYToggle:
            y = self.driver1.getLeftY()
            x = self.driver1.getLeftX()
            rx = -self.driver1.getRightX()
        else:
            x = self.driver1.getLeftY()
            y = -self.driver1.getLeftX()
            rx = -self.driver1.getRightX()

        Idiot_y = math.pow(y, 3)
        Idiot_x = math.pow(x, 3)
        Idiot_rx = math.pow(rx, 3)

        self.drive.driveCartesian(Idiot_x, Idiot_y, Idiot_rx, -self.gyro.getRotation2d())

        # Actuator Mode Logic
        if self.actuatorMode.Mode == self.actuatorMode.Idle:
            self.currentArmPosition = 0  # Arm Down
            self.currentWristPosition = 1  # Idle Wrist Position
            self.shooterOn = False
            if self.driver2.getRightTriggerAxis() > self.driver2.getLeftTriggerAxis():
                self.intake.set(self.driver2.getRightTriggerAxis())
            else:
                self.intake.set(-self.driver2.getLeftTriggerAxis())

            # Reset TeleAuto Vars
            self.teleautoArmUp = False
            self.teleautoWristOut = False
        elif self.actuatorMode.Mode == self.actuatorMode.Intaking:
            self.currentArmPosition = 0  # Arm Down
            self.currentWristPosition = 0  # Wrist Down
            self.shooterOn = False
            self.intake.set(-1)
        elif self.actuatorMode.Mode == self.actuatorMode.Shooting:
            self.currentArmPosition = 0  # Arm Down
            self.currentWristPosition = 1  # Idle Wrist Position
            self.shooterOn = True
            self.intake.set(0)
            if self.shooterEncoder.getVelocity() > 3000:
                self.intake.set(0.5)  # Spit note out of jaws
            # If shooter spun and intake reversed for x revolutions, set mode to idle
        elif self.actuatorMode.Mode == self.actuatorMode.Amping:
            self.currentArmPosition = 1  # Arm Up
            self.currentWristPosition = 3  # Idle Wrist Position
            self.shooterOn = False
            self.intake.set(0)
            if self.armBuiltinEncoder.getPosition() >= 110:
                self.teleautoArmUp = True
            if self.teleautoArmUp is True:
                self.currentWristPosition = 0
            # -1000 is Out of perimeter, -4000 is Safe
            if self.teleautoArmUp and self.wristEncoder.get() >= -1100:
                self.teleautoWristOut = True

            if self.driver2.getRightTriggerAxis() > self.driver2.getLeftTriggerAxis():
                self.intake.set(self.driver2.getRightTriggerAxis())
            else:
                self.intake.set(-self.driver2.getLeftTriggerAxis())

            if self.teleautoArmUp and self.teleautoWristOut:  # If Arm up and wrist position is flipped then drop note
                # TODO: Add driver 2 rumble
                pass

            # TODO: If intake/outake spins for x amount of revolutions set mode to idle
        elif self.actuatorMode.Mode == self.actuatorMode.Reset:
            self.currentArmPosition = 2
            self.currentWristPosition = 2

        # Safety Switches and Arm
        if self.armDownLimitSwitch.get() is False:
            self.arm.set(0.05)
            if self.armEncoderReseting is False:
                self.armBuiltinEncoder.setPosition(0)
            self.armEncoderReseting = True
        elif self.armUpLimitSwitch.get() is False:
            self.arm.set(-0.125)
        else:
            self.armEncoderReseting = False
            self.arm.set(self.armPID.calculate(self.armBuiltinEncoder.getPosition(), self.armPositions[self.currentArmPosition]))

        # Wrist Feedback Controller

        self.wrist.set(-0.004*self.wristPID.calculate(self.wristEncoder.get(), self.wristPositions[self.currentWristPosition]))

        # Shooter Feedback and Feedforward Controller
        if self.shooterOn:
            self.shooter.set(0.005*self.shooterPID.calculate(self.shooterEncoder.getVelocity(), 3500) + self.shooterVValueSub.get())
            self.shooterHelper.set(-0.005*self.shooterHelperPID.calculate(self.shooterHelperEncoder.getVelocity(), 3500) - self.shooterHelperVValueSub.get())
        else:
            self.shooter.set(0)
            self.shooterHelper.set(0)

        if self.driver1.getYButtonReleased():
            self.solenoidRed.set(False)
            self.solenoidBlue.set(True)
        if self.driver1.getAButtonPressed():
            self.solenoidRed.set(True)
            self.solenoidBlue.set(False)

        # Send Back Data
        self.PidgeonCompass.set(self.pidgen.getCompassHeading())
        self.GyroPub.set(self.gyro.getAngle())
        self.GryoConnected.set(self.gyro.isConnected())
        self.armEncoderValueNet.set(self.armBuiltinEncoder.getPosition())
        self.shooterRPM.set(self.shooterEncoder.getVelocity())
        self.shooterHelperRPM.set(self.shooterHelperEncoder.getVelocity())

    def disabledInit(self):

        # self.arm.setIdleMode(self.arm.IdleMode.kCoast)
        self.compressor.disable()
        self.arm.set(0)
        self.wrist.set(0)
        self.intake.set(0)
        self.shooter.set(0)
        self.shooterHelper.set(0)
        self.mecanum.rearLeftMotor.set(0)
        self.mecanum.frontLeftMotor.set(0)
        self.mecanum.rearRightMotor.set(0)
        self.mecanum.frontRightMotor.set(0)
        self.intakeOffAuto = False
        self.autoEndWristVal = False

    def disabledPeriodic(self):
        # PID Variable Declaration
        self.armPID = PIDController(
            self.armPVarSub.get(),
            self.armIVarSub.get(),
            self.armDVarSub.get()
        )
        self.shooterPID = PIDController(
            self.shooterPValueSub.get(),
            self.shooterIValueSub.get(),
            self.shooterDValueSub.get()
        )
        self.shooterHelperPID = PIDController(
            self.shooterHelperPValueSub.get(),
            self.shooterHelperIValueSub.get(),
            self.shooterHelperDValueSub.get()
        )
        self.wristPID = PIDController(
            self.wristPSub.get(),
            self.wristISub.get(),
            self.wristDSub.get()
        )
        self.currentArmPosition = 0

    def testInit(self):
        if self.inStartingPosition:
            self.armBuiltinEncoder.setPosition(198)
            self.wristEncoder.reset()
            self.inStartingPosition = False
        self.arm.setIdleMode(rev.CANSparkMax.IdleMode.kCoast)
        self.wrist.setIdleMode(rev.CANSparkMax.IdleMode.kCoast)
        self.intake.setIdleMode(rev.CANSparkMax.IdleMode.kCoast)

    def testPeriodic(self):
        # if self.armDownLimitSwitch.get() is False:
        #     self.arm.set(0.05)
        #     if self.armEncoderReseting is False:
        #         self.armBuiltinEncoder.setPosition(0)
        #     self.armEncoderReseting = True
        # elif self.armUpLimitSwitch.get() is False:
        #     self.arm.set(-0.125)
        # else:
        #     self.armEncoderReseting = False
        #     self.arm.set(self.armPID.calculate(self.armBuiltinEncoder.getPosition(), 50))
        # self.wrist.set(-0.004 * self.wristPID.calculate(self.wristEncoder.get(), -1000))
        pass

    def testExit(self):
        self.arm.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        self.wrist.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        self.intake.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)

    def intakeOffAutoFunction(self):
        self.intakeOffAuto = True

    def autoEndWristFunction(self):
        self.autoEndWristVal = True

    def ender2(self):
        self.disableAutoWheels = True


if __name__ == "__main__":
    wpilib.run(MyRobot)
