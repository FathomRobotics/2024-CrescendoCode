#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import wpilib.drive
import navx
import wpimath.controller
import wpimath.geometry
from wpimath.kinematics import ChassisSpeeds
import wpimath.units
import phoenix5
import phoenix5.sensors
from wpimath.geometry import Translation2d
from wpimath.kinematics import MecanumDriveKinematics
from wpimath.kinematics import MecanumDriveWheelPositions
from wpimath.kinematics import MecanumDriveOdometry
from wpimath.geometry import Pose2d
from wpimath.geometry import Rotation2d
from wpimath.controller import PIDController

import math


class Drivetrain:
    """Represents a differential drive style drivetrain."""

    def resetGryoThread(self):
        time.sleep(1)
        self.gyro.reset()

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

        self.gyro = navx.AHRS(wpilib.SPI.Port.kMXP)
        gyroThread = threading.Thread(target=self.resetGryoThread)
        gyroThread.run()

        self.kinematics = wpimath.kinematics.MecanumDriveKinematics(
            frontLeftLocation, frontRightLocation, rearLeftLocation, rearRightLocation
        )

        self.odometry = wpimath.kinematics.MecanumDriveOdometry(
            self.kinematics, self.gyro.getRawGyro(), self.getCurrentDistances()
        )

        # Gains are for example purposes only - must be determined for your own robot!
        self.feedforward = wpimath.controller.SimpleMotorFeedforwardMeters(1, 1)

        self.gyro.reset()

        # We need to invert one side of the drivetrain so that positive voltages
        # result in both sides moving forward. Depending on how your robot's
        # gearbox is constructed, you might have to invert the left side instead.
        self.rearLeftMotor.setInverted(True)

    def getCurrentState(self) -> wpimath.kinematics.MecanumDriveWheelSpeeds:
        """Returns the current state of the drivetrain."""
        return wpimath.kinematics.MecanumDriveWheelSpeeds(
            self.frontLeftMotorEncoder.getRate(),
            self.frontRightMotorEncoder.getRate(),
            self.rearLeftMotorEncoder.getRate(),
            self.rearRightMotorEncoder.getRate(),
        )

    def getCurrentDistances(self) -> wpimath.kinematics.MecanumDriveWheelPositions:
        """Returns the current distances measured by the drivetrain."""
        pos = wpimath.kinematics.MecanumDriveWheelPositions()

        pos.frontLeft = self.frontLeftMotorEncoder.getDistance()
        pos.frontRight = self.frontRightMotorEncoder.getDistance()
        pos.rearLeft = self.rearLeftMotorEncoder.getDistance()
        pos.rearRight = self.rearRightMotorEncoder.getDistance()

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

    def coastMotors(self):
        self.frontLeftMotor.setNeutralMode(phoenix5.NeutralMode.Coast)
        self.rearLeftMotor.setNeutralMode(phoenix5.NeutralMode.Coast)
        self.frontRightMotor.setNeutralMode(phoenix5.NeutralMode.Coast)
        self.rearRightMotor.setNeutralMode(phoenix5.NeutralMode.Coast)

    def breakMotors(self):
        self.frontLeftMotor.setNeutralMode(phoenix5.NeutralMode.Brake)
        self.rearLeftMotor.setNeutralMode(phoenix5.NeutralMode.Brake)
        self.frontRightMotor.setNeutralMode(phoenix5.NeutralMode.Brake)
        self.rearRightMotor.setNeutralMode(phoenix5.NeutralMode.Brake)
