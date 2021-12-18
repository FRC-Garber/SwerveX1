// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;


import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DRIVE;
import frc.robot.Constants.SWERVE;

public class DrivelineSubsystem extends SubsystemBase {
  private final SwerveModule m_frontLeft = new SwerveModule(1,2,3);
  private final SwerveModule m_frontRight = new SwerveModule(4,5,6);
  private final SwerveModule m_backLeft = new SwerveModule(7,8,9);
  private final SwerveModule m_backRight = new SwerveModule(10,11,12);
  
  // The gyro sensor
  private final AHRS m_gyro = new AHRS(Port.kMXP);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(DRIVE.kDriveKinematics, m_gyro.getRotation2d());

  /** Creates a new Drivesubsystem. */
  public DrivelineSubsystem() {
    m_frontLeft.setSteerAbsoluteOffset(SWERVE.kFLAbsoluteOffsetInDegrees);
    m_frontRight.setSteerAbsoluteOffset(SWERVE.kFRAbsoluteOffsetInDegrees);
    m_backLeft.setSteerAbsoluteOffset(SWERVE.kBLAbsoluteOffsetInDegrees);
    m_backRight.setSteerAbsoluteOffset(SWERVE.kBRAbsoluteOffsetInDegrees);

  }

  // The periodic method is called first on every pass of the schedular for a subsystem. Then the commands are executed
  // It is recommended that only non-essential functionality goes here like updating the dashboard.
  @Override
  public void periodic() {
    // Update the robots position on the field. This is used for Autonomous.
    m_odometry.update(       
      m_gyro.getRotation2d(),
      m_frontLeft.getState(),
      m_backLeft.getState(),
      m_frontRight.getState(),
      m_backRight.getState());
  }
  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }
    /**
   * Method to drive the robot using joystick info.
   * Inputs are +/- 1 from the joystick
   * Values are converted to needed swereve module rates
   *
   * @param _xSpeed Speed of the robot in the x direction (forward).
   * @param _ySpeed Speed of the robot in the y direction (sideways).
   * @param _rot Angular rate of the robot.
   * @param _fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double _xSpeed, double _ySpeed, double _rot, boolean _fieldRelative) {
    // Convert joystick values of +/- 1 to Meters/Sec and Rad/Sec
    double xSpeed = _ySpeed * DRIVE.kMaxSpeedMetersPerSecond;
    double ySpeed = -_xSpeed * DRIVE.kMaxSpeedMetersPerSecond;
    double rot = -_rot * DRIVE.kMaxAngularRateRadPerSecond;

    // Calculate the swerve module states
    SwerveModuleState[] swerveModuleStates = DRIVE.kDriveKinematics.toSwerveModuleStates(_fieldRelative 
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    // Normalize the wheel speeds                
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, DRIVE.kMaxSpeedMetersPerSecond);
    // Set the desired state of each swerve module with the new calculated states.
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }
}
