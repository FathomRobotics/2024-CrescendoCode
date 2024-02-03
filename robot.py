#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import threading
import time

import cscore
import navx
import ntcore
import phoenix5
import wpilib
import wpilib.drive
import wpimath.geometry


class MyRobot(wpilib.TimedRobot):
    """
    This is a demo program showing how to use Mecanum control with the
    MecanumDrive class.
    """

    # The channel on the driver station that the joystick is connected to
    joystickChannel = 0

    def robotInit(self):
        """Robot initialization function"""

        cscore.CameraServer.startAutomaticCapture()

        self.frontLeftMotor = phoenix5.WPI_TalonSRX(1)
        self.rearLeftMotor = phoenix5.WPI_TalonSRX(3)
        self.frontRightMotor = phoenix5.WPI_TalonSRX(2)
        self.rearRightMotor = phoenix5.WPI_TalonSRX(4)

        # invert the left side motors
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

    def resetGryoThread(self):
        time.sleep(1)
        self.gyro.reset()

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        """Runs the motors with Mecanum drive."""
        self.GyroPub.set(self.gyro.getAngle())
        self.GryoConnected.set(self.gyro.isConnected())

        y = -self.stick.getLeftY()
        x = -self.stick.getLeftX()
        rx = -self.stick.getRightX()

        self.drive.driveCartesian(x, y, rx, -self.gyro.getRotation2d())

        """Alternatively, to match the driver station enumeration, you may use  ---> self.drive.driveCartesian(
            self.stick.getRawAxis(1), self.stick.getRawAxis(3), self.stick.getRawAxis(2), 0
        )"""


if __name__ == "__main__":
    wpilib.run(MyRobot)
