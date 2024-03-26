#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import wpilib.drive
import wpimath.controller
import wpimath.geometry
from wpimath.kinematics import ChassisSpeeds
import wpimath.units

import math


class Drivetrain:
    """Represents a differential drive style drivetrain."""

    kMaxSpeed = 3.0  # 3 meters per second
    kMaxAngularSpeed = math.pi  # 1/2 rotation per second

    def __init__(self):
        self.rearLeftMotor = phoenix5.WPI_TalonSRX(1)
        self.rearRightMotor = phoenix5.WPI_TalonSRX(2)
        self.frontRightMotor = phoenix5.WPI_TalonSRX(3)
        self.frontLeftMotor = phoenix5.WPI_TalonSRX(4)

        self.frontLeftMotorEncoder = wpilib.Encoder(0, 1)
        self.rearLeftMotorEncoder = wpilib.Encoder(2, 3)
        self.frontRightMotorEncoder = wpilib.Encoder(4, 5)
        self.rearRightMotorEncoder = wpilib.Encoder(6, 7)

        frontLeftLocation = Translation2d(0.2713, 0.2715)
        frontRightLocation = Translation2d(0.2713, -0.2715)
        rearLeftLocation = Translation2d(-0.2713, 0.2715)
        rearRightLocation = Translation2d(-0.2713, -0.2715)

        self.frontLeftPIDController = wpimath.controller.PIDController(1, 0, 0)
        self.frontRightPIDController = wpimath.controller.PIDController(1, 0, 0)
        self.rearLeftPIDController = wpimath.controller.PIDController(1, 0, 0)
        self.rearRightPIDController = wpimath.controller.PIDController(1, 0, 0)

        self.gyro = phoenix5.sensors.Pigeon2(5)

        self.kinematics = wpimath.kinematics.MecanumDriveKinematics(
            frontLeftLocation, frontRightLocation, rearLeftLocation, rearRightLocation
        )

        self.odometry = wpimath.kinematics.MecanumDriveOdometry(
            self.kinematics, self.gyro.getRotation2d(), self.getCurrentDistances()
        )

        # Gains are for example purposes only - must be determined for your own robot!
        self.feedforward = wpimath.controller.SimpleMotorFeedforwardMeters(1, 1)

        self.gyro.reset()

        # We need to invert one side of the drivetrain so that positive voltages
        # result in both sides moving forward. Depending on how your robot's
        # gearbox is constructed, you might have to invert the left side instead.
        self.frontRightMotor.setInverted(True)
        self.rearRightMotor.setInverted(True)

    def getCurrentState(self) -> wpimath.kinematics.MecanumDriveWheelSpeeds:
        """Returns the current state of the drivetrain."""
        return wpimath.kinematics.MecanumDriveWheelSpeeds(
            self.frontLeftEncoder.getRate(),
            self.frontRightEncoder.getRate(),
            self.rearLeftEncoder.getRate(),
            self.rearRightEncoder.getRate(),
        )

    def getCurrentDistances(self) -> wpimath.kinematics.MecanumDriveWheelPositions:
        """Returns the current distances measured by the drivetrain."""
        pos = wpimath.kinematics.MecanumDriveWheelPositions()

        pos.frontLeft = self.frontLeftEncoder.getDistance()
        pos.frontRight = self.frontRightEncoder.getDistance()
        pos.rearLeft = self.rearLeftEncoder.getDistance()
        pos.rearRight = self.rearRightEncoder.getDistance()

        return pos

    def setSpeeds(self, speeds: wpimath.kinematics.MecanumDriveWheelSpeeds):
        """Sets the desired speeds for each wheel."""
        frontLeftFeedforward = self.feedforward.calculate(speeds.frontLeft)
        frontRightFeedforward = self.feedforward.calculate(speeds.frontRight)
        rearLeftFeedforward = self.feedforward.calculate(speeds.rearLeft)
        rearRightFeedforward = self.feedforward.calculate(speeds.rearRight)

        frontLeftOutput = self.frontLeftPIDController.calculate(
            self.frontLeftEncoder.getRate(), speeds.frontLeft
        )
        frontRightOutput = self.frontRightPIDController.calculate(
            self.frontRightEncoder.getRate(), speeds.frontRight
        )
        rearLeftOutput = self.frontLeftPIDController.calculate(
            self.rearLeftEncoder.getRate(), speeds.rearLeft
        )
        rearRightOutput = self.frontRightPIDController.calculate(
            self.rearRightEncoder.getRate(), speeds.rearRight
        )

        self.frontLeftMotor.setVoltage(frontLeftOutput + frontLeftFeedforward)
        self.frontRightMotor.setVoltage(frontRightOutput + frontRightFeedforward)
        self.rearLeftMotor.setVoltage(rearLeftOutput + rearLeftFeedforward)
        self.rearRightMotor.setVoltage(rearRightOutput + rearRightFeedforward)

    def drive(
        self,
        xSpeed: float,
        ySpeed: float,
        rot: float,
        fieldRelative: bool,
        periodSeconds: float,
    ):
        """Method to drive the robot using joystick info."""
        mecanumDriveWheelSpeeds = self.kinematics.toWheelSpeeds(
            ChassisSpeeds.discretize(
                (
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, self.gyro.getRotation2d()
                    )
                    if fieldRelative
                    else ChassisSpeeds(xSpeed, ySpeed, rot)
                ),
                periodSeconds,
            )
        )
        mecanumDriveWheelSpeeds.desaturate(self.kMaxSpeed)
        self.setSpeeds(mecanumDriveWheelSpeeds)

    def updateOdometry(self):
        """Updates the field-relative position."""
        self.odometry.update(self.gyro.getRotation2d(), self.getCurrentDistances())