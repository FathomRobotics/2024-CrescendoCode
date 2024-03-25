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


class PIDController:
    def __init__(self, sp):
        self.integralSummation = 0
        self.lastError = 0
        self.originalTime = int(time.time())
        self.setpoint = sp

    def start(self, setpoint):
        self.integralSummation = 0
        self.lastError = 0
        self.originalTime = int(time.time())
        if setpoint is not None:
            self.setpoint = setpoint

    def pidController(self, state, p, i, d):
        # Note: This is how we do it
        # https://github.com/Evium-99/FtcRobotController/blob/25dd0e3e160cc1c4ef66c7e2853a0461da9241de/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/armPIDTuner.java#L125-L131
        error = self.setpoint - state
        integralSummation = self.integralSummation + (error * (int(time.time()) - self.originalTime))
        derivative = (error - self.lastError) / (int(time.time()) - self.originalTime)
        self.lastError = error
        return (p * error) + (i * integralSummation) + (d * derivative)


class MyRobot(wpilib.TimedRobot):
    """
    This is a demo program showing how to use Mecanum control with the
    MecanumDrive class.
    """

    def robotInit(self):
        """Robot initialization function"""

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

        # Encoders
        self.frontLeftMotorEncoder = wpilib.Encoder(0, 1)
        self.rearLeftMotorEncoder = wpilib.Encoder(2, 3)
        self.frontRightMotorEncoder = wpilib.Encoder(4, 5)
        self.rearRightMotorEncoder = wpilib.Encoder(6, 7)
        self.shooterEncoder = self.shooter.getEncoder()

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

        # NavX Initialization
        self.gyro = navx.AHRS(wpilib.SPI.Port.kMXP)
        gyroThread = threading.Thread(target=self.resetGryoThread)
        gyroThread.run()

        # Network Tables Initialization
        inst = ntcore.NetworkTableInstance.getDefault()
        table = inst.getTable("SmartDashboard")
        self.GyroPub = table.getDoubleTopic("Gyro-Degrees").publish()  # Gyro Value
        self.GryoConnected = table.getBooleanTopic("NavX-Connected").publish()  # NavX Connected?
        self.PidgeonCompass = table.getDoubleTopic("Pigeon-Degrees").publish()  # Gyro Pigeon Value
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

        # PID Variable Declaration
        self.armPID = None

    def robotPeriodic(self):
        if self.powerDistribution.getTotalCurrent() < self.maxCurrentDrawWhenCheckingPercentage:
            self.BatteryPercentageEstimationTopic.set(
                (self.powerDistribution.getVoltage() - self.minVoltage) / self._maxmandiffVoltage)

    def resetGryoThread(self):
        time.sleep(1)
        self.gyro.reset()

    def autonomousPeriodic(self):
        # Note: Look here
        # https://github.com/robotpy/examples/blob/main/GyroDriveCommands/commands/turntoangle.py
        # https://docs.wpilib.org/en/stable/docs/software/pathplanning/trajectory-tutorial/characterizing-drive.html
        # https://docs.wpilib.org/en/stable/docs/software/advanced-controls/index.html
        pass

    def teleopInit(self):
        if self.compressor.getPressure() > 60:
            self.compressor.disable()
        else:
            self.compressor.enableDigital()
        self.compressor.enableDigital()
        self.frontLeftMotorEncoder.reset()
        self.rearLeftMotorEncoder.reset()
        self.frontRightMotorEncoder.reset()
        self.rearRightMotorEncoder.reset()
        self.armPID = PIDController(0)

    def disabledInit(self):
        self.compressor.disable()

    def teleopPeriodic(self):
        """Runs the motors with Mecanum drive."""
        if self.driver1.getAButtonPressed():
            self.compressor.disable()
        if self.driver1.getYButtonReleased():
            self.compressor.enableDigital()

        self.PidgeonCompass.set(self.pidgen.getCompassHeading())
        self.GyroPub.set(self.gyro.getAngle())
        self.GryoConnected.set(self.gyro.isConnected())

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

        self.arm.set(-self.driver2.getRightY())
        self.wrist.set(-((0.25 * self.driver2.getLeftY()) + (pow(self.driver2.getLeftY(), 7) * 0.75)))
        # TODO: Make shooter be controlled by Driver 2 left bumper
        self.shooter.set(self.driver1.getLeftTriggerAxis())
        self.shooterHelper.set(-self.driver1.getLeftTriggerAxis())

        self.frontLeftMotorEncoderNetworkTopic.set((self.frontLeftMotorEncoder.getRaw() / 10000) * 6 * math.pi)
        self.rearLeftMotorEncoderNetworkTopic.set((self.rearLeftMotorEncoder.getRaw() / 10000) * 6 * math.pi)
        self.frontRightMotorEncoderNetworkTopic.set((self.frontRightMotorEncoder.getRaw() / 10000) * 6 * math.pi)
        self.rearRightMotorEncoderNetworkTopic.set((self.rearRightMotorEncoder.getRaw() / 10000) * 6 * math.pi)

        if self.driver1.getYButtonReleased():
            self.solenoidRed.set(False)
            self.solenoidRed.set(True)
        if self.driver1.getAButtonPressed():
            self.solenoidRed.set(True)
            self.solenoidRed.set(False)


if __name__ == "__main__":
    wpilib.run(MyRobot)
