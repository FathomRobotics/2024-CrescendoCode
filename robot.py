#!/usr/bin/env python3
#
# Copyright (c) Fathom Robotics
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import math
import threading
import time

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
import wpimath.filter

from drivetrain import Drivetrain
from robotcontainer import RobotContainer
# python -m robotpy deploy --nc --skip-tests


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


class MyRobot(wpilib.TimedRobot):
    """
    This is a demo program showing how to use Mecanum control with the
    MecanumDrive class.
    """

    def robotInit(self):
        """Robot initialization function"""
        # Kinematics and Odometry
        # Locations of the wheels relative to the robot center.
        # 21.375in Wide
        # 20.5in Long

        # Drive Encoders
        self.frontLeftMotorEncoder = wpilib.Encoder(0, 1)
        self.rearLeftMotorEncoder = wpilib.Encoder(2, 3)
        self.frontRightMotorEncoder = wpilib.Encoder(4, 5)
        self.rearRightMotorEncoder = wpilib.Encoder(6, 7)

        self.mecanum = Drivetrain(
            self.frontLeftMotorEncoder,
            self.rearLeftMotorEncoder,
            self.frontRightMotorEncoder,
            self.rearRightMotorEncoder
        )

        self.robotContainer = RobotContainer(self.mecanum)
        self.autoCommand = self.robotContainer.getAutonomousCommand()
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

        self.powerDistribution = wpilib.PowerDistribution()

        # Control Devices
        # self.rearLeftMotor = phoenix5.WPI_TalonSRX(1)
        # self.rearRightMotor = phoenix5.WPI_TalonSRX(2)
        # self.frontRightMotor = phoenix5.WPI_TalonSRX(3)
        # self.frontLeftMotor = phoenix5.WPI_TalonSRX(4)
        self.pidgen = phoenix5.sensors.Pigeon2(5)
        self.intake = rev.CANSparkMax(7, type=rev.CANSparkLowLevel.MotorType.kBrushless)
        self.arm = rev.CANSparkMax(8, type=rev.CANSparkLowLevel.MotorType.kBrushless)
        self.wrist = rev.CANSparkMax(9, type=rev.CANSparkLowLevel.MotorType.kBrushless)
        self.shooter = rev.CANSparkMax(10, type=rev.CANSparkLowLevel.MotorType.kBrushless)
        self.shooterHelper = rev.CANSparkMax(21, type=rev.CANSparkLowLevel.MotorType.kBrushless)
        self.pnumaticsHub = wpilib.PneumaticHub(17)

        # Enbeded Encoders
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

        self.wristP = table.getDoubleTopic("WristP").publish()
        self.wristP.set(0.025)
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
        self.PidgeonCompass.set(self.pidgen.getCompassHeading())
        self.GryoConnected.set(False)

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
        self.wristPositions = [-1000, -4500, 0, -4200]
        self.currentWristPosition = 1
        self.wristSP.set(0)

        # Sticky Vars
        self.armEncoderReseting = False
        self.armPositionChanged = False
        self.autoArmDownStart = False
        self.teleautoArmUp = False
        self.teleautoWristOut = False
        self.inStartingPosition = True  # Assume starting in starting position

        # Actuator Mode
        self.actuatorMode = ActuatorSystemModeManager()
        self.actuatorMode.setIdle()

    def robotPeriodic(self):
        self.armDownLimitSwitchNetwork.set(self.armDownLimitSwitch.get())
        self.armUpLimitSwitchNetwork.set(self.armUpLimitSwitch.get())
        self.wristEncoderNetwork.set(self.wristEncoder.get())
        # Get my wheel positions
        # wheelPositions = wpimath.kinematics.MecanumDriveWheelPositions(
        #     self.frontLeftMotorEncoder.getDistance(), self.frontRightMotorEncoder.getDistance(),
        #     self.rearLeftMotorEncoder.getDistance(), self.rearRightMotorEncoder.getDistance()
        # )
        #
        # # Get the rotation of the robot from the gyro.
        # gyroAngle = self.gyro.getRotation2d()
        #
        # # Update the pose self.pose = odometry.update(gyroAngle, wheelPositions) self.fieldPose.set(str(self.pose))
        # if self.powerDistribution.getTotalCurrent() < self.maxCurrentDrawWhenCheckingPercentage:
        # self.BatteryPercentageEstimationTopic.set((self.powerDistribution.getVoltage() - self.minVoltage) /
        # self._maxmandiffVoltage)
        self.mecanum.updateOdometry()

    def resetGryoThread(self):
        time.sleep(1)
        self.gyro.reset()

    def autonomousInit(self):
        if self.inStartingPosition:
            self.armBuiltinEncoder.setPosition(198)
            self.wristEncoder.reset()
            self.inStartingPosition = False
        auto = pathplannerlib.auto.AutoBuilder.buildAuto("SamAuto")
        auto.schedule()

    def autonomousPeriodic(self):
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
        pass

    def teleopInit(self):
        if self.autoCommand is not None:
            self.autoCommand.cancel()

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

        self.mecanum.coastMotors()  # Make Motors Coast at start

        # self.arm.setIdleMode(self.arm.IdleMode.kBrake)  # Enable Arm Breaking

    def teleopPeriodic(self):
        """Runs the motors with Mecanum drive."""
        # Break Button (Driver 1 B)
        if self.driver1.getBButton():
            self.mechanum.breakMotors()
        else:
            self.mecanum.coastMotors()

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
        xSpeed = (
                -self.xspeedLimiter.calculate(Idiot_y)
                * Drivetrain.kMaxSpeed
        )

        # Get the y speed or sideways/strafe speed. We are inverting this because
        # we want a positive value when we pull to the left. Xbox controllers
        # return positive values when you pull to the right by default.
        ySpeed = (
                -self.yspeedLimiter.calculate(Idiot_x)
                * Drivetrain.kMaxSpeed
        )

        # Get the rate of angular rotation. We are inverting this because we want a
        # positive value when we pull to the left (remember, CCW is positive in
        # mathematics). Xbox's controllers return positive values when you pull to
        # the right by default.
        rot = (
                -self.rotLimiter.calculate(Idiot_rx)
                * Drivetrain.kMaxAngularSpeed
        )

        self.mecanum.drive(xSpeed, ySpeed, rot, True, self.getPeriod())

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
        # if self.armDownLimitSwitch.get() is False:
        #     self.arm.set(0.05)
        #     if self.armEncoderReseting is False:
        #         self.armBuiltinEncoder.setPosition(0)
        #     self.armEncoderReseting = True
        # elif self.armUpLimitSwitch.get() is False:
        #     self.arm.set(-0.125)
        # else:
        #     self.armEncoderReseting = False
        #     self.arm.set(self.armPID.calculate(self.armBuiltinEncoder.getPosition(), self.armPositions[self.currentArmPosition]))

        # Wrist Feedback Controller

        # self.wrist.set(-0.004*self.wristPID.calculate(self.wristEncoder.get(), self.wristPositions[self.currentWristPosition]))

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
        # self.frontLeftMotorEncoderNetworkTopic.set((self.frontLeftMotorEncoder.getRaw() / 10000) * 6 * math.pi)
        # self.rearLeftMotorEncoderNetworkTopic.set((self.rearLeftMotorEncoder.getRaw() / 10000) * 6 * math.pi)
        # self.frontRightMotorEncoderNetworkTopic.set((self.frontRightMotorEncoder.getRaw() / 10000) * 6 * math.pi)
        # self.rearRightMotorEncoderNetworkTopic.set((self.rearRightMotorEncoder.getRaw() / 10000) * 6 * math.pi)

        # Manual Code Graveyard
        # TODO: Implement oh shoot manual mode

        # This code was for wrist
        # if self.driver2.getAButtonReleased():
        #     if self.currentWristPosition == (len(self.wristPositions) - 1):
        #         self.currentWristPosition = 0
        #     else:
        #         self.currentWristPosition += 1
        #     self.wristSP.set(self.wristPositions[self.currentWristPosition])

        # This code used to be for shooter
        # if self.driver2.getXButtonReleased():
        #     self.shooterOn = not self.shooterOn

        # This was the manual code for shooter
        # self.shooter.set(self.driver1.getLeftTriggerAxis())
        # self.shooterHelper.set(-self.driver1.getLeftTriggerAxis())

    def disabledInit(self):
        # self.arm.setIdleMode(self.arm.IdleMode.kCoast)
        self.compressor.disable()
        if self.autoCommand is not None:
            self.autoCommand.cancel()

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


if __name__ == "__main__":
    wpilib.run(MyRobot)
