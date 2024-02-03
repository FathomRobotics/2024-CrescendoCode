#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import phoenix5
import wpilib


class MyRobot(wpilib.TimedRobot):
    """
    This is a demo program showing how to use Mecanum control with the
    MecanumDrive class.
    """

    # The channel on the driver station that the joystick is connected to
    joystickChannel = 0

    def robotInit(self):
        """Robot initialization function"""
        self.frontLeftMotor = phoenix5.TalonSRX(1)
        self.rearLeftMotor = phoenix5.TalonSRX(3)
        self.frontRightMotor = phoenix5.TalonSRX(2)
        self.rearRightMotor = phoenix5.TalonSRX(4)

        # invert the left side motors
        self.frontLeftMotor.setInverted(True)

        # you may need to change or remove this to match your robot
        self.rearLeftMotor.setInverted(True)

        # self.drive = MecanumDrive(
        #     self.frontLeftMotor,
        #     self.rearLeftMotor,
        #     self.frontRightMotor,
        #     self.rearRightMotor,
        # )
        # Define the Xbox Controller.
        self.stick = wpilib.XboxController(self.joystickChannel)

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        """Runs the motors with Mecanum drive."""
        # Use the joystick X axis for lateral movement, Y axis for forward movement, and Z axis for rotation.
        # This sample does not use field-oriented drive, so the gyro input is set to zero.
        # This Stick configuration is created by K.E. on our team.  Left stick Y axis is speed, Left Stick X axis is strafe, and Right Stick Y axis is turn.
        y = -self.stick.getLeftY()
        x = self.stick.getLeftX()
        rx = -self.stick.getRightX()

        divisor = max(abs(y) + abs(x) + abs(rx), 1)

        frontLeftPower = (y + x + rx) / divisor
        backLeftPower = (y - x + rx) / divisor
        frontRightPower = (y - x - rx) / divisor
        backRightPower = (y + x - rx) / divisor

        self.frontLeftMotor.set(mode=phoenix5.ControlMode.PercentOutput, value=frontLeftPower)
        self.rearLeftMotor.set(mode=phoenix5.ControlMode.PercentOutput, value=backLeftPower)
        self.frontRightMotor.set(mode=phoenix5.ControlMode.PercentOutput, value=frontRightPower)
        self.rearRightMotor.set(mode=phoenix5.ControlMode.PercentOutput, value=backRightPower)

        """Alternatively, to match the driver station enumeration, you may use  ---> self.drive.driveCartesian(
            self.stick.getRawAxis(1), self.stick.getRawAxis(3), self.stick.getRawAxis(2), 0
        )"""


if __name__ == "__main__":
    wpilib.run(MyRobot)
