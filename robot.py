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


class MyRobot(wpilib.TimedRobot):
    """
    This is a demo program showing how to use Mecanum control with the
    MecanumDrive class.
    """

    # The channel on the driver station that the joystick is connected to
    joystickChannel = 0
    joystickChannel2 = 1

    def robotInit(self):
        """Robot initialization function"""

        # cscore.CameraServer.startAutomaticCapture()

        self.frontLeftMotor = phoenix5.WPI_TalonSRX(4)
        self.rearLeftMotor = phoenix5.WPI_TalonSRX(1)
        self.frontRightMotor = phoenix5.WPI_TalonSRX(3)
        self.rearRightMotor = phoenix5.WPI_TalonSRX(2)

        self.intake = rev.CANSparkMax(7, type=rev.CANSparkLowLevel.MotorType.kBrushless)
        self.arm = rev.CANSparkMax(8, type=rev.CANSparkLowLevel.MotorType.kBrushless)
        self.wrist = rev.CANSparkMax(9, type=rev.CANSparkLowLevel.MotorType.kBrushless)
        self.shooter = rev.CANSparkMax(10, type=rev.CANSparkLowLevel.MotorType.kBrushless)
        self.shooterEncoder = self.shooter.getEncoder()

        self.frontLeftMotorEncoder = wpilib.Encoder(0, 1)
        self.rearLeftMotorEncoder = wpilib.Encoder(2, 3)
        self.frontRightMotorEncoder = wpilib.Encoder(4, 5)
        self.rearRightMotorEncoder = wpilib.Encoder(6, 7)

        # invert the left side motors
        self.frontLeftMotor.setInverted(True)

        # Gyro
        self.gyro = navx.AHRS(wpilib.SPI.Port.kMXP)
        # self.gyro2 = phoenix5.Pi
        self.pidgen = phoenix5.sensors.Pigeon2(5)
        self.gyro.isConnected()
        gyroThread = threading.Thread(target=self.resetGryoThread)
        gyroThread.run()
        inst = ntcore.NetworkTableInstance.getDefault()
        table = inst.getTable("SmartDashboard")
        self.GyroPub = table.getDoubleTopic("Gyro").publish()
        self.GryoConnected = table.getBooleanTopic("GyroConnected").publish()
        self.PidgeonCompass = table.getDoubleTopic("PidgeonCompass").publish()
        self.testRPM = table.getDoubleTopic("TestRPM").publish()
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
        self.stick = wpilib.XboxController(self.joystickChannel)
        self.stick2 = wpilib.XboxController(self.joystickChannel2)

    def resetGryoThread(self):
        time.sleep(1)
        self.gyro.reset()

    def teleopInit(self):
        self.frontLeftMotorEncoder.reset()
        self.rearLeftMotorEncoder.reset()
        self.frontRightMotorEncoder.reset()
        self.rearRightMotorEncoder.reset()

    def teleopPeriodic(self):
        """Runs the motors with Mecanum drive."""
        self.PidgeonCompass.set(self.pidgen.getCompassHeading())
        self.GyroPub.set(self.gyro.getAngle())
        self.GryoConnected.set(self.gyro.isConnected())

        y = -self.stick.getLeftY()
        x = -self.stick.getLeftX()
        rx = -self.stick.getRightX()

        Idiot_y = math.pow(y, 3)
        Idiot_x = math.pow(x, 3)
        Idiot_rx = math.pow(rx, 3)
        self.testRPM.set(self.shooterEncoder.getVelocity())

        self.drive.driveCartesian(Idiot_x, Idiot_y, Idiot_rx, -self.gyro.getRotation2d())

        self.intake.set(-self.stick.getRightTriggerAxis())
        self.arm.set(self.stick2.getRightY())
        self.wrist.set(self.stick2.getLeftY())
        self.shooter.set(self.stick.getLeftTriggerAxis())

        self.frontLeftMotorEncoderNetworkTopic.set((self.frontLeftMotorEncoder.getRaw()/10000) * 6 * math.pi)
        self.rearLeftMotorEncoderNetworkTopic.set((self.rearLeftMotorEncoder.getRaw()/10000) * 6 * math.pi)
        self.frontRightMotorEncoderNetworkTopic.set((self.frontRightMotorEncoder.getRaw()/10000) * 6 * math.pi)
        self.rearRightMotorEncoderNetworkTopic.set((self.rearRightMotorEncoder.getRaw()/10000) * 6 * math.pi)

        """Alternatively, to match the driver station enumeration, you may use  ---> self.drive.driveCartesian(
            self.stick.getRawAxis(1), self.stick.getRawAxis(3), self.stick.getRawAxis(2), 0
        )"""


if __name__ == "__main__":
    wpilib.run(MyRobot)
