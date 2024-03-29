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
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import HolonomicPathFollowerConfig, ReplanningConfig, PIDConstants
from wpilib import DriverStation
import threading
import time

import math


class Drivetrain:
    """Represents a differential drive style drivetrain."""

    kMaxSpeed = 3.0  # 3 meters per second
    kMaxAngularSpeed = math.pi*2  # 1/2 rotation per second

    def resetGryoThread(self):
        time.sleep(1)
        self.gyro.reset()

    def __init__(self):
        self.field = wpilib.Field2d()
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

        # kP = 0.005 and kI 0.0001 Bad
        kP = 0.0001
        kI = 0
        kD = 0
        self.frontLeftPIDController = wpimath.controller.PIDController(kP, kI, kD)
        self.frontRightPIDController = wpimath.controller.PIDController(kP, kI, kD)
        self.rearLeftPIDController = wpimath.controller.PIDController(kP, kI, kD)
        self.rearRightPIDController = wpimath.controller.PIDController(kP, kI, kD)

        self.gyro = navx.AHRS(wpilib.SPI.Port.kMXP)
        gyroThread = threading.Thread(target=self.resetGryoThread)
        gyroThread.run()

        self.kinematics = wpimath.kinematics.MecanumDriveKinematics(
            frontLeftLocation, frontRightLocation, rearLeftLocation, rearRightLocation
        )

        self.odometry = wpimath.kinematics.MecanumDriveOdometry(
            self.kinematics, self.gyro.getRotation2d(), self.getCurrentDistances()
        )

        # Gains are for example purposes only - must be determined for your own robot!
        self.feedforward = wpimath.controller.SimpleMotorFeedforwardMeters(1.5, 1.5)

        self.gyro.reset()

        # We need to invert one side of the drivetrain so that positive voltages
        # result in both sides moving forward. Depending on how your robot's
        # gearbox is constructed, you might have to invert the left side instead.
        self.rearRightMotor.setInverted(True)
        self.frontRightMotor.setInverted(True)

        AutoBuilder.configureHolonomic(
            self.getPose,  # Robot pose supplier
            self.resetPose,  # Method to reset odometry (will be called if your auto has a starting pose)
            self.getRobotRelativeSpeeds,  # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            self.driveRobotRelative,  # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            HolonomicPathFollowerConfig(  # HolonomicPathFollowerConfig, this should likely live in your Constants class
                PIDConstants(kP, 0.0, 0.0),  # Translation PID constants
                PIDConstants(kP, 0.0, 0.0),  # Rotation PID constants
                self.kMaxSpeed,  # Max module speed, in m/s
                self.kMaxAngularSpeed,  # Drive base radius in meters. Distance from robot center to furthest module.
                ReplanningConfig()  # Default path replanning config. See the API for the options here
            ),
            self.shouldFlipPath,  # Supplier to control path flipping based on alliance color
            self  # Reference to this subsystem to set requirements
        )

    def getCurrentState(self) -> wpimath.kinematics.MecanumDriveWheelSpeeds:
        """Returns the current state of the drivetrain."""
        return wpimath.kinematics.MecanumDriveWheelSpeeds(
            self.frontLeftMotorEncoder.getRate(),
            self.frontRightMotorEncoder.getRate(),
            self.rearLeftMotorEncoder.getRate(),
            self.rearRightMotorEncoder.getRate(),
        )

    def getPose(self):
        return self.odometry.getPose()

    def resetPose(self, pose):
        # TODO: https://github.com/mjansen4857/pathplanner/blob/d0e12f59430b869bde70180cb709e175b762fc67/examples/java/src/main/java/frc/robot/subsystems/SwerveSubsystem.java#L92
        self.odometry.resetPosition(self.gyro.getRotation2d(), ?self.getPositions(), pose)

    def getRobotRelativeSpeeds(self):
        return self.getCurrentState()

    def getCurrentDistances(self) -> wpimath.kinematics.MecanumDriveWheelPositions:
        """Returns the current distances measured by the drivetrain."""
        pos = wpimath.kinematics.MecanumDriveWheelPositions()

        pos.frontLeft = self.frontLeftMotorEncoder.getDistance()
        pos.frontRight = self.frontRightMotorEncoder.getDistance()
        pos.rearLeft = self.rearLeftMotorEncoder.getDistance()
        pos.rearRight = self.rearRightMotorEncoder.getDistance()

        return pos

    def driveRobotRelative(self, speeds):
        self.setSpeeds(speeds=speeds)

    def shouldFlipPath(self):
        # Boolean supplier that controls when the path will be mirrored for the red alliance
        # This will flip the path being followed to the red side of the field.
        # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed

    def setSpeeds(self, speeds: wpimath.kinematics.MecanumDriveWheelSpeeds):
        """Sets the desired speeds for each wheel."""
        frontLeftFeedforward = self.feedforward.calculate(speeds.frontLeft)
        frontRightFeedforward = self.feedforward.calculate(speeds.frontRight)
        rearLeftFeedforward = self.feedforward.calculate(speeds.rearLeft)
        rearRightFeedforward = self.feedforward.calculate(speeds.rearRight)

        frontLeftOutput = self.frontLeftPIDController.calculate(
            self.frontLeftMotorEncoder.getRate(), speeds.frontLeft
        )
        frontRightOutput = self.frontRightPIDController.calculate(
            self.frontRightMotorEncoder.getRate(), speeds.frontRight
        )
        rearLeftOutput = self.frontLeftPIDController.calculate(
            self.rearLeftMotorEncoder.getRate(), speeds.rearLeft
        )
        rearRightOutput = self.frontRightPIDController.calculate(
            self.rearRightMotorEncoder.getRate(), speeds.rearRight
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
