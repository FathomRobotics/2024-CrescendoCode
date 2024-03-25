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

        self.minVoltage = 11.9
        self.maxVoltage = 13
        self.maxCurrentDrawWhenCheckingPercentage = 0.05
        self._maxmandiffVoltage = self.maxVoltage - self.minVoltage
        if self._maxmandiffVoltage <= 0:
            wpilib.reportWarning("Min Voltage Variable is Larger or Equal to Max", printTrace=False)

        self.joystickChannel = 0
        self.joystickChannel2 = 1

        self.powerDistribution = wpilib.PowerDistribution()

        # CAN Devices
        self.rearLeftMotor = phoenix5.WPI_TalonSRX(1)
        self.rearRightMotor = phoenix5.WPI_TalonSRX(2)
        self.frontRightMotor = phoenix5.WPI_TalonSRX(3)
        self.frontLeftMotor = phoenix5.WPI_TalonSRX(4)
        self.pidgen = phoenix5.sensors.Pigeon2(5)
        # self.pnumaticsHub = wpilib.PneumaticHub(canID)

        self.intake = rev.CANSparkMax(7, type=rev.CANSparkLowLevel.MotorType.kBrushless)
        self.arm = rev.CANSparkMax(8, type=rev.CANSparkLowLevel.MotorType.kBrushless)
        self.wrist = rev.CANSparkMax(9, type=rev.CANSparkLowLevel.MotorType.kBrushless)
        self.shooter = rev.CANSparkMax(10, type=rev.CANSparkLowLevel.MotorType.kBrushless)
        self.shooterHelper = rev.CANSparkMax(21, type=rev.CANSparkLowLevel.MotorType.kBrushless)
        self.shooterEncoder = self.shooter.getEncoder()

        self.frontLeftMotorEncoder = wpilib.Encoder(0, 1)
        self.rearLeftMotorEncoder = wpilib.Encoder(2, 3)
        self.frontRightMotorEncoder = wpilib.Encoder(4, 5)
        self.rearRightMotorEncoder = wpilib.Encoder(6, 7)

        # invert the left side motors 17
        self.pnumaticsHub = wpilib.PneumaticHub(17)
        self.solinoidRed = self.pnumaticsHub.makeSolenoid(0)
        self.solinoidBlue = self.pnumaticsHub.makeSolenoid(1)
        self.compressor = self.pnumaticsHub.makeCompressor()
        self.compressor.isEnabled()
        self.frontLeftMotor.setInverted(True)

        # Gyro
        self.gyro = navx.AHRS(wpilib.SPI.Port.kMXP)
        self.gyro.isConnected()
        gyroThread = threading.Thread(target=self.resetGryoThread)
        gyroThread.run()
        inst = ntcore.NetworkTableInstance.getDefault()
        table = inst.getTable("SmartDashboard")
        self.GyroPub = table.getDoubleTopic("Gyro").publish()
        self.GryoConnected = table.getBooleanTopic("GyroConnected").publish()
        self.PidgeonCompass = table.getDoubleTopic("PidgeonCompass").publish()
        self.BatteryPercentageEstimationTopic = table.getDoubleTopic("BatteryPercentageEstimation").publish()
        self.frontLeftMotorEncoderNetworkTopic = table.getDoubleTopic("frontLeftMotorEncoder").publish()
        self.rearLeftMotorEncoderNetworkTopic = table.getDoubleTopic("rearLeftMotorEncoder").publish()
        self.frontRightMotorEncoderNetworkTopic = table.getDoubleTopic("frontRightMotorEncoder").publish()
        self.rearRightMotorEncoderNetworkTopic = table.getDoubleTopic("rearRightMotorEncoder").publish()
        self.PidgeonCompass.set(self.pidgen.getCompassHeading())
        self.GryoConnected.set(False)

        # you may need to change or remove this to match your robot
        self.rearLeftMotor.setInverted(True)

        self.drive = wpilib.drive.MecanumDrive(
            self.frontLeftMotor,
            self.rearLeftMotor,
            self.frontRightMotor,
            self.rearRightMotor,
        )
        self.f2d = wpilib.Field2d()

        # Define the Xbox Controller.
        self.driver1 = wpilib.XboxController(self.joystickChannel)
        self.driver2 = wpilib.XboxController(self.joystickChannel2)
        self.stickXYToggle = False

        # PID Stuff
        self.armPID = None

    def robotPeriodic(self):
        if self.powerDistribution.getTotalCurrent() < self.maxCurrentDrawWhenCheckingPercentage:
            self.BatteryPercentageEstimationTopic.set(
                (self.powerDistribution.getVoltage() - self.minVoltage) / self._maxmandiffVoltage)

    def resetGryoThread(self):
        time.sleep(1)
        self.gyro.reset()

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

        if self.driver1.getRightBumper():
            self.intake.set(self.driver1.getRightTriggerAxis())
        else:
            self.intake.set(-self.driver1.getRightTriggerAxis())

        self.arm.set(-self.driver2.getRightY())
        self.wrist.set(-((0.25 * self.driver2.getLeftY()) + (pow(self.driver2.getLeftY(), 7) * 0.75)))
        self.shooter.set(self.driver1.getLeftTriggerAxis())
        self.shooterHelper.set(-self.driver1.getLeftTriggerAxis())

        self.frontLeftMotorEncoderNetworkTopic.set((self.frontLeftMotorEncoder.getRaw() / 10000) * 6 * math.pi)
        self.rearLeftMotorEncoderNetworkTopic.set((self.rearLeftMotorEncoder.getRaw() / 10000) * 6 * math.pi)
        self.frontRightMotorEncoderNetworkTopic.set((self.frontRightMotorEncoder.getRaw() / 10000) * 6 * math.pi)
        self.rearRightMotorEncoderNetworkTopic.set((self.rearRightMotorEncoder.getRaw() / 10000) * 6 * math.pi)

        if self.driver1.getXButtonReleased():
            if self.solinoidRed.get():
                self.solinoidRed.set(False)
            else:
                self.solinoidRed.set(True)
        if self.driver1.getBButtonReleased():
            if self.solinoidBlue.get():
                self.solinoidBlue.set(False)
            else:
                self.solinoidBlue.set(True)

        """Alternatively, to match the driver station enumeration, you may use  ---> self.drive.driveCartesian(
            self.stick.getRawAxis(1), self.stick.getRawAxis(3), self.stick.getRawAxis(2), 0
        )"""


if __name__ == "__main__":
    wpilib.run(MyRobot)
