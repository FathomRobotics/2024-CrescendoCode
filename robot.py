#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
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
from wpimath.geometry import Translation2d
from wpimath.kinematics import MecanumDriveKinematics
from wpimath.kinematics import MecanumDriveWheelPositions
from wpimath.kinematics import MecanumDriveOdometry
from wpimath.geometry import Pose2d
from wpimath.geometry import Rotation2d
from wpimath.controller import PIDController
# python -m robotpy deploy --nc --skip-tests


class ActuatorSystemModeManager:
    def __init__(self):
        self.Mode = None
        self.Idle = 0
        self.Intaking = 1
        self.Shooting = 2
        self.Amping = 3

    def setModeToAmping(self):
        self.Mode = self.Amping
    
    def setModeToIdle(self):
        self.Mode = self.Idle

    def setModeToShooting(self):
        self.Mode = self.Shooting



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
        #

        # NavX Initialization
        self.gyro = navx.AHRS(wpilib.SPI.Port.kMXP)
        gyroThread = threading.Thread(target=self.resetGryoThread)
        gyroThread.run()

        # DIO Encoders
        self.frontLeftMotorEncoder = wpilib.Encoder(0, 1)
        self.rearLeftMotorEncoder = wpilib.Encoder(2, 3)
        self.frontRightMotorEncoder = wpilib.Encoder(4, 5)
        self.rearRightMotorEncoder = wpilib.Encoder(6, 7)
        self.wristEncoder = wpilib.Encoder(23, 22)

        # frontLeftLocation = Translation2d(0.2713, 0.2715)
        # frontRightLocation = Translation2d(0.2713, -0.2715)
        # backLeftLocation = Translation2d(-0.2713, 0.2715)
        # backRightLocation = Translation2d(-0.2713, -0.2715)
        #
        # # Creating my kinematics object using the wheel locations.
        # self.kinematics = wpimath.kinematics.MecanumDriveKinematics(
        #     frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation
        # )
        #
        # # Creating my odometry object from the kinematics object and the initial wheel positions.
        # # Here, our starting pose is 5 meters along the long end of the field and in the
        # # center of the field along the short end, facing the opposing alliance wall.
        # self.odometry = MecanumDriveOdometry(
        #     self.kinematics,
        #     self.gyro.getRotation2d(),
        #     wpimath.kinematics.MecanumDriveWheelPositions(
        #         self.frontLeftMotorEncoder.getDistance(), self.frontRightMotorEncoder.getDistance(),
        #         self.rearLeftMotorEncoder.getDistance(), self.rearRightMotorEncoder.getDistance()
        #     ),
        #     Pose2d(5.0, 13.5, Rotation2d())
        # )
        #
        # # Get my wheel positions
        # wheelPositions = wpimath.kinematics.MecanumDriveWheelPositions(
        #     self.frontLeftMotorEncoder.getDistance(), self.frontRightMotorEncoder.getDistance(),
        #     self.rearLeftMotorEncoder.getDistance(), self.rearRightMotorEncoder.getDistance()
        # )
        #
        # # Get the rotation of the robot from the gyro.
        # gyroAngle = self.gyro.getRotation2d()
        #
        # # Update the pose
        # self.pose = odometry.update(gyroAngle, wheelPositions)

        # Novelty Battery Percentage
        self.minVoltage = 11.9
        self.maxVoltage = 13
        self.maxCurrentDrawWhenCheckingPercentage = 0.05
        self._maxmandiffVoltage = self.maxVoltage - self.minVoltage
        if self._maxmandiffVoltage <= 0:
            wpilib.reportWarning("Min Voltage Variable is Larger or Equal to Max", printTrace=False)

        self.powerDistribution = wpilib.PowerDistribution()

        # Control Devices
        self.rearLeftMotor = phoenix5.WPI_TalonSRX(1)
        self.rearRightMotor = phoenix5.WPI_TalonSRX(2)
        self.frontRightMotor = phoenix5.WPI_TalonSRX(3)
        self.frontLeftMotor = phoenix5.WPI_TalonSRX(4)
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

        # Drive Train Configuration
        self.rearLeftMotor.setInverted(True)

        self.drive = wpilib.drive.MecanumDrive(
            self.frontLeftMotor,
            self.rearLeftMotor,
            self.frontRightMotor,
            self.rearRightMotor,
        )

        # Pneumatics Hub Devices
        self.solenoidRed = self.pnumaticsHub.makeSolenoid(0)
        self.solenoidBlue = self.pnumaticsHub.makeSolenoid(1)
        self.compressor = self.pnumaticsHub.makeCompressor()
        self.compressor.isEnabled()
        self.frontLeftMotor.setInverted(True)

        # Network Tables Initialization
        inst = ntcore.NetworkTableInstance.getDefault()
        table = inst.getTable("SmartDashboard")
        # TODO: Implement Field 2d View
        # TODO: Implement Speedometer
        # TODO: Implement warning lights (Power Usage, Arm Limits, PDH Temp)
        # TODO: Implement Shooter Indicator Light
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
        self.armPVar.set(0.02)
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
        # Arm Positions
        self.armPositions = [0, 120]
        self.currentArmPosition = 1
        self.armSPVar.set(120)

        # Wrist Positions
        self.wristPositions = [-1000, -4500]
        self.currentWristPosition = 1
        self.wristSP.set(0)

        # Sticky Vars
        self.armEncoderReseting = False
        self.armPositionChanged = False
        self.autoArmDownStart = False

    def robotPeriodic(self):
        self.armDownLimitSwitchNetwork.set(self.armDownLimitSwitch.get())
        self.armUpLimitSwitchNetwork.set(self.armUpLimitSwitch.get())
        self.wristEncoderNetwork.set(self.wristEncoder.get())
        # self.fieldPose.set(str(self.pose))
        # if self.powerDistribution.getTotalCurrent() < self.maxCurrentDrawWhenCheckingPercentage:
        #     self.BatteryPercentageEstimationTopic.set((self.powerDistribution.getVoltage() - self.minVoltage) / self._maxmandiffVoltage)

    def resetGryoThread(self):
        time.sleep(1)
        self.gyro.reset()

    def autonomousPeriodic(self):
        # Note: Look here
        # https://github.com/robotpy/examples/blob/main/GyroDriveCommands/commands/turntoangle.py
        # https://docs.wpilib.org/en/stable/docs/software/pathplanning/trajectory-tutorial/characterizing-drive.html
        # https://docs.wpilib.org/en/stable/docs/software/advanced-controls/index.html

        # # Get my wheel positions
        # wheelPositions = wpimath.kinematics.MecanumDriveWheelPositions(
        #     self.frontLeftMotorEncoder.getDistance(), self.frontRightMotorEncoder.getDistance(),
        #     self.rearLeftMotorEncoder.getDistance(), self.rearRightMotorEncoder.getDistance()
        # )

        # Get the rotation of the robot from the gyro.
        # Put Wrist Into Shooting Position
        self.wrist.set(-0.004 * self.wristPID.calculate(self.wristEncoder.get(), -4500))
        # Start Shooter Motors
        self.shooter.set(0.005 * self.shooterPID.calculate(self.shooterEncoder.getVelocity(), 3500) + self.shooterVValueSub.get())
        self.shooterHelper.set(-0.005 * self.shooterHelperPID.calculate(self.shooterHelperEncoder.getVelocity(), 3500) - self.shooterHelperVValueSub.get())

        # If Wrist in position start arm movement
        if self.wristEncoder.get() <= -4400:
            self.autoArmDownStart = True

        # If Arm in position after wrist is in position shoot
        if self.autoArmDownStart and (self.armBuiltinEncoder.getPosition() <= 30):
            self.intake.set(0.25)

        if self.autoArmDownStart:
            self.arm.set(self.armPID.calculate(self.armBuiltinEncoder.getPosition(), 0))
        else:
            self.arm.set(self.armPID.calculate(self.armBuiltinEncoder.getPosition(), 120))

    def teleopInit(self):
        self.arm.setIdleMode(self.arm.IdleMode.kBrake)
        # PID Variable Declaration
        self.armPID = PIDController(
            self.armPVarSub.get(),
            self.armIVarSub.get(),
            self.armDVarSub.get()
        )
        if self.compressor.getPressure() > 60:
            self.compressor.disable()
        else:
            self.compressor.enableDigital()
        self.compressor.enableDigital()
        self.frontLeftMotorEncoder.reset()
        self.rearLeftMotorEncoder.reset()
        self.frontRightMotorEncoder.reset()
        self.rearRightMotorEncoder.reset()
        self.armBuiltinEncoder.setPosition(198)
        self.wristEncoder.reset()
        self.frontLeftMotor.setNeutralMode(phoenix5.NeutralMode.Coast)
        self.rearLeftMotor.setNeutralMode(phoenix5.NeutralMode.Coast)
        self.frontRightMotor.setNeutralMode(phoenix5.NeutralMode.Coast)
        self.rearRightMotor.setNeutralMode(phoenix5.NeutralMode.Coast)

    def teleopPeriodic(self):
        """Runs the motors with Mecanum drive."""
        # Break Button
        if self.driver1.getBButton():
            self.frontLeftMotor.setNeutralMode(phoenix5.NeutralMode.Brake)
            self.rearLeftMotor.setNeutralMode(phoenix5.NeutralMode.Brake)
            self.frontRightMotor.setNeutralMode(phoenix5.NeutralMode.Brake)
            self.rearRightMotor.setNeutralMode(phoenix5.NeutralMode.Brake)
        else:
            self.frontLeftMotor.setNeutralMode(phoenix5.NeutralMode.Coast)
            self.rearLeftMotor.setNeutralMode(phoenix5.NeutralMode.Coast)
            self.frontRightMotor.setNeutralMode(phoenix5.NeutralMode.Coast)
            self.rearRightMotor.setNeutralMode(phoenix5.NeutralMode.Coast)
        # Set Arm Setpoint
        if self.driver2.getBButtonReleased():
            if self.currentArmPosition == (len(self.armPositions) - 1):
                self.currentArmPosition = 0
            else:
                self.currentArmPosition += 1
            self.armSPVar.set(self.armPositions[self.currentArmPosition])

        # Set Wrist Setpoint
        if self.driver2.getAButtonReleased():
            if self.currentWristPosition == (len(self.wristPositions) - 1):
                self.currentWristPosition = 0
            else:
                self.currentWristPosition += 1
            self.wristSP.set(self.wristPositions[self.currentWristPosition])

        if self.driver2.getYButtonReleased():
            if self.currentWristPosition == 0:
                self.currentWristPosition = (len(self.wristPositions) - 1)
            else:
                self.currentWristPosition -= 1
            self.wristSP.set(self.wristPositions[self.currentWristPosition])

        # Toggle Shooter

        if self.stickXYToggle:
            y = self.driver1.getLeftY()
            x = -self.driver1.getLeftX()
            rx = -self.driver1.getRightX()
        else:
            x = self.driver1.getLeftY()
            y = -self.driver1.getLeftX()
            rx = -self.driver1.getRightX()

        if self.driver1.getLeftStickButtonReleased():
            self.stickXYToggle = not self.stickXYToggle

        Idiot_y = math.pow(y, 3)
        Idiot_x = math.pow(x, 3)
        Idiot_rx = math.pow(rx, 3)

        self.drive.driveCartesian(Idiot_x, Idiot_y, Idiot_rx, -self.gyro.getRotation2d())

        if self.driver2.getRightTriggerAxis() > self.driver2.getLeftTriggerAxis():
            self.intake.set(self.driver2.getRightTriggerAxis())
        else:
            self.intake.set(-self.driver2.getLeftTriggerAxis())

        # TODO: Make shooter be controlled by Driver 2 left bumper
        if self.armDownLimitSwitch.get() is False:
            self.arm.set(0.05)
            if self.armEncoderReseting is False:
                self.armBuiltinEncoder.setPosition(0)
            self.armEncoderReseting = True
        elif self.armUpLimitSwitch.get() is False:
            self.arm.set(-0.125)
        else:
            self.armEncoderReseting = False
            self.arm.set(self.armPID.calculate(self.armBuiltinEncoder.getPosition(), self.armSPVarSub.get()))

        self.wrist.set(-0.004*self.wristPID.calculate(self.wristEncoder.get(), self.wristSPSub.get()))

        if self.driver2.getXButtonReleased():
            self.shooterOn = not self.shooterOn

        if self.shooterOn:
            self.shooter.set(0.005*self.shooterPID.calculate(self.shooterEncoder.getVelocity(), 3500) + self.shooterVValueSub.get())
            self.shooterHelper.set(-0.005*self.shooterHelperPID.calculate(self.shooterHelperEncoder.getVelocity(), 3500) - self.shooterHelperVValueSub.get())
        else:
            self.shooter.set(0)
            self.shooterHelper.set(0)

        # self.shooter.set(self.driver1.getLeftTriggerAxis())
        # self.shooterHelper.set(-self.driver1.getLeftTriggerAxis())

        self.frontLeftMotorEncoderNetworkTopic.set((self.frontLeftMotorEncoder.getRaw() / 10000) * 6 * math.pi)
        self.rearLeftMotorEncoderNetworkTopic.set((self.rearLeftMotorEncoder.getRaw() / 10000) * 6 * math.pi)
        self.frontRightMotorEncoderNetworkTopic.set((self.frontRightMotorEncoder.getRaw() / 10000) * 6 * math.pi)
        self.rearRightMotorEncoderNetworkTopic.set((self.rearRightMotorEncoder.getRaw() / 10000) * 6 * math.pi)

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
        # 3000 In middle
        # TODO: Add shooter ready

    def disabledInit(self):
        self.arm.setIdleMode(self.arm.IdleMode.kCoast)
        self.compressor.disable()

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
