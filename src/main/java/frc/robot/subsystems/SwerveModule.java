// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {

  private final TalonFX m_driveMotor;
  private final TalonFX m_turningMotor;

  private final CANCoder m_driveEncoder;
  //Driving encoder uses the motor port.
  // e.g. testMotor.getSelectedSensorPosition();

  private final PIDController m_drivePIDController = 
    new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);
  
  //Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPidController = 
    new ProfiledPIDController(
      ModuleConstants.kPModuleTurningController,
      0,
      0,
      new TrapezoidProfile.Constraints(
          ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
          ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));


  /** Creates a new SwerveModule. */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int driveEncoderPorts,
      int turningEncoderPorts) {
    
    m_driveMotor = new TalonFX(driveMotorChannel);
    m_turningMotor = new TalonFX(turningMotorChannel);
    
    m_turningMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 1, 0);

    this.m_driveEncoder = new CANCoder(driveEncoderPorts);
    
  }

  //Returns the current state of the module

  public SwerveModuleState getState(){
    double m_speedMetersPerSecond = 
      (m_driveEncoder.getVelocity() * (ModuleConstants.kWheelCircumferenceMeters / 360)) / ModuleConstants.kDriveGearRatio;

    double m_rotationDegrees =  
      ModuleConstants.kTurningPositiontoDegrees * m_turningMotor.getSelectedSensorPosition();

    return new SwerveModuleState(m_speedMetersPerSecond, new Rotation2d(m_rotationDegrees));
  }

  public void setDesiredState(SwerveModuleState desiredState){

    double m_speedMetersPerSecond = 
      (m_driveEncoder.getVelocity() * (ModuleConstants.kWheelCircumferenceMeters / 360)) / ModuleConstants.kDriveGearRatio;
    double m_rotationDegrees =  
      ModuleConstants.kTurningPositiontoDegrees * m_turningMotor.getSelectedSensorPosition();

    //Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = 
      SwerveModuleState.optimize(desiredState, new Rotation2d(m_rotationDegrees));

    //Calculate the drive output from the drive PID controller
    final double driveOutput =
      m_drivePIDController.calculate(m_speedMetersPerSecond, state.speedMetersPerSecond);

    final var turnOutput = 
      m_turningPidController.calculate(m_rotationDegrees, state.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller
    m_driveMotor.set(ControlMode.PercentOutput, driveOutput);
    m_turningMotor.set(ControlMode.PercentOutput, turnOutput);
  }

  public void resetEncoders() {
    //m_turningMotor
    m_driveEncoder
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
