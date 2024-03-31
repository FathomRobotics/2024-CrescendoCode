#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import commands2
import pathplannerlib.logging
import pathplannerlib.path
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
    kMaxAngularSpeed = math.pi * 3  # 1/2 rotation per second

    def resetGryoThread(self):
        time.sleep(1)
        self.gyro.reset()

    def __init__(self, frontLeftMotorEncoder, rearLeftMotorEncoder, frontRightMotorEncoder, rearRightMotorEncoder):
        self.pathInit = pathplannerlib.path.PathPlannerPath.fromPathFile('Simple1')
        self.field = wpilib.Field2d()
        self.rearLeftMotor = phoenix5.WPI_TalonSRX(1)
        self.rearRightMotor = phoenix5.WPI_TalonSRX(2)
        self.frontRightMotor = phoenix5.WPI_TalonSRX(3)
        self.frontLeftMotor = phoenix5.WPI_TalonSRX(4)

        self.frontLeftMotorEncoder = frontLeftMotorEncoder
        self.rearLeftMotorEncoder = rearLeftMotorEncoder
        self.frontRightMotorEncoder = frontRightMotorEncoder
        self.rearRightMotorEncoder = rearRightMotorEncoder

        frontLeftLocation = Translation2d(0.2713, 0.2715)
        frontRightLocation = Translation2d(0.2713, -0.2715)
        rearLeftLocation = Translation2d(-0.2713, 0.2715)
        rearRightLocation = Translation2d(-0.2713, -0.2715)

        # kP = 0.005 and kI 0.0001 Bad
        self.kP = 3
        self.kI = 0
        self.kD = 0
        self.feedforwardValue = 0
        self.frontLeftPIDController = wpimath.controller.PIDController(self.kP, self.kI, self.kD)
        self.frontRightPIDController = wpimath.controller.PIDController(self.kP, self.kI, self.kD)
        self.rearLeftPIDController = wpimath.controller.PIDController(self.kP, self.kI, self.kD)
        self.rearRightPIDController = wpimath.controller.PIDController(self.kP, self.kI, self.kD)

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
        self.feedforward = wpimath.controller.SimpleMotorFeedforwardMeters(self.feedforwardValue, self.feedforwardValue)

        self.gyro.reset()

        # We need to invert one side of the drivetrain so that positive voltages
        # result in both sides moving forward. Depending on how your robot's
        # gearbox is constructed, you might have to invert the left side instead.
        self.rearLeftMotor.setInverted(True)
        self.frontLeftMotor.setInverted(True)
        self.rearRightMotor.setInverted(False)
        self.frontRightMotor.setInverted(False)

        AutoBuilder.configureHolonomic(
            self.getPose,  # Robot pose supplier
            self.resetPose,  # Method to reset odometry (will be called if your auto has a starting pose)
            self.getCurrentSpeeds,  # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            self.driveRobotRelative,  # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            HolonomicPathFollowerConfig(  # HolonomicPathFollowerConfig, this should likely live in your Constants class
                PIDConstants(self.kP, 0.0, 0.0),  # Translation PID constants
                PIDConstants(0.25, 0.1, 0.0),  # Rotation PID constants
                self.kMaxSpeed,  # Max module speed, in m/s
                0.381,  # Drive base radius in meters. Distance from robot center to furthest module.
                ReplanningConfig()  # Default path replanning config. See the API for the options here
            ),
            self.shouldFlipPath,  # Supplier to control path flipping based on alliance color
            self  # Reference to this subsystem to set requirements
        )

    def followSam(self):
        return pathplannerlib.auto.FollowPathHolonomic(
            self.pathInit,
            self.getPose,
            self.getCurrentSpeeds,
            self.driveRobotRelative,
            HolonomicPathFollowerConfig(  # HolonomicPathFollowerConfig, this should likely live in your Constants class
                PIDConstants(1.2, 0.0, 0.0),  # Translation PID constants
                PIDConstants(0.7, 0.0, 0.0),  # Rotation PID constants
                2,  # Max module speed, in m/s
                0.381,  # Drive base radius in meters. Distance from robot center to furthest module.
                ReplanningConfig()  # Default path replanning config. See the API for the options here
            ),
            self.shouldFlipPath,  # Supplier to control path flipping based on alliance color
            self
        ).andThen()

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

    def stopRobot(self):
        self.frontLeftPIDController.setPID(0, 0, 0)
        self.frontRightPIDController.setPID(0, 0, 0)
        self.rearLeftPIDController.setPID(0, 0, 0)
        self.rearRightPIDController.setPID(0, 0, 0)

    def resetPose(self, pose):
        self.odometry.resetPosition(self.gyro.getRotation2d(), self.getCurrentDistances(), pose)

    def getCurrentSpeeds(self):
        return self.kinematics.toChassisSpeeds(self.getCurrentState())

    def getCurrentDistances(self) -> wpimath.kinematics.MecanumDriveWheelPositions:
        """Returns the current distances measured by the drivetrain."""
        pos = wpimath.kinematics.MecanumDriveWheelPositions()

        pos.frontLeft = self.frontLeftMotorEncoder.getDistance()
        pos.frontRight = self.frontRightMotorEncoder.getDistance()
        pos.rearLeft = self.rearLeftMotorEncoder.getDistance()
        pos.rearRight = self.rearRightMotorEncoder.getDistance()

        return pos

    def driveRobotRelative(self, speeds):
        self.setSpeeds(self.kinematics.toWheelSpeeds(speeds))

    def shouldFlipPath(self):
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

    def periodic(self):
        pass

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
