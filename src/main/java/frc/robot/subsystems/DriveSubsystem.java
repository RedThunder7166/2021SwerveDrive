// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

  //Robot swerve modules

  private final SwerveModule m_frontLeft =
  new SwerveModule(
      DriveConstants.kFrontLeftDriveMotorPort,
      DriveConstants.kFrontLeftTurningMotorPort,
      DriveConstants.kFrontLeftDriveEncoderPorts,
      DriveConstants.kFrontLeftTurningEncoderPorts,
      DriveConstants.kFrontLeftDriveEncoderReversed,
      DriveConstants.kFrontLeftTurningEncoderReversed);

private final SwerveModule m_rearLeft =
  new SwerveModule(
      DriveConstants.kRearLeftDriveMotorPort,
      DriveConstants.kRearLeftTurningMotorPort,
      DriveConstants.kRearLeftDriveEncoderPorts,
      DriveConstants.kRearLeftTurningEncoderPorts,
      DriveConstants.kRearLeftDriveEncoderReversed,
      DriveConstants.kRearLeftTurningEncoderReversed);

private final SwerveModule m_frontRight =
  new SwerveModule(
      DriveConstants.kFrontRightDriveMotorPort,
      DriveConstants.kFrontRightTurningMotorPort,
      DriveConstants.kFrontRightDriveEncoderPorts,
      DriveConstants.kFrontRightTurningEncoderPorts,
      DriveConstants.kFrontRightDriveEncoderReversed,
      DriveConstants.kFrontRightTurningEncoderReversed);

private final SwerveModule m_rearRight =
  new SwerveModule(
      DriveConstants.kRearRightDriveMotorPort,
      DriveConstants.kRearRightTurningMotorPort,
      DriveConstants.kRearRightDriveEncoderPorts,
      DriveConstants.kRearRightTurningEncoderPorts,
      DriveConstants.kRearRightDriveEncoderReversed,
      DriveConstants.kRearRightTurningEncoderReversed);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
