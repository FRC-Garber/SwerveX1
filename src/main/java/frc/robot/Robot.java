// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DRIVE;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  int m_virtualQuad = 1;
  int m_previousDesiredQuad = 1;
  int[][] m_desiredQuadDirIndexArray= {{0,2,1,3},{1,0,3,2},{2,3,0,1},{3,1,2,0}};
  int[][] m_virtualQuadArray = {{1,2,3,4,-1,-2,-3,-4},{2,3,4,1,1,-1,-2,-3},{-1,1,2,3,-2,-3,-4,-1},{-2,-1,1,2,2,1,-1,-2}};
  Joystick stick = new Joystick(0);
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    //ChassisSpeeds speeds = new ChassisSpeeds(stick.getX(), -stick.getY(), 0);
    var swerveModuleStates = DRIVE.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(-stick.getY(), -stick.getX(), stick.getRawAxis(2)));
    SmartDashboard.putNumber("SMS[0].angle.deg", swerveModuleStates[0].angle.getDegrees());
    SmartDashboard.putNumber("SMS[1].angle.deg", swerveModuleStates[1].angle.getDegrees());
    SmartDashboard.putNumber("SMS[2].angle.deg", swerveModuleStates[2].angle.getDegrees());
    SmartDashboard.putNumber("SMS[3].angle.deg", swerveModuleStates[3].angle.getDegrees());
    int quad0 = calcAngleQuadrant(swerveModuleStates[0].angle.getDegrees());
    int quad1 = calcAngleQuadrant(swerveModuleStates[1].angle.getDegrees());
    int quad2 = calcAngleQuadrant(swerveModuleStates[2].angle.getDegrees());
    int quad3 = calcAngleQuadrant(swerveModuleStates[3].angle.getDegrees());
    SmartDashboard.putNumber("m_virtualQuad0", quad0);
    SmartDashboard.putNumber("m_virtualQuad1", quad1);
    SmartDashboard.putNumber("m_virtualQuad2", quad2);
    SmartDashboard.putNumber("m_virtualQuad3", quad3);

    // Generate new SwerveModuleStates with the new angle based on the quadrant. 
    double newAng;
    if(quad0 > 2){
      newAng = Math.toRadians(180 + (180 + swerveModuleStates[0].angle.getDegrees()) );
    }else if(quad0 < -2){
      newAng = Math.toRadians(-180 + (-180 + swerveModuleStates[0].angle.getDegrees()) );
    }else {
      newAng = swerveModuleStates[0].angle.getRadians();
    }
   // double newAng = quad0 > 2 ? Math.toRadians(180 + (180 + swerveModuleStates[0].angle.getDegrees()) ) : swerveModuleStates[0].angle.getRadians();
    SwerveModuleState sms_0 = new SwerveModuleState(swerveModuleStates[0].speedMetersPerSecond,new Rotation2d(newAng));
    SwerveModuleState sms_0_Optimized = SwerveModuleState.optimize(sms_0, swerveModuleStates[0].angle);
    SmartDashboard.putNumber("SMSOp_0Ang", sms_0_Optimized.angle.getDegrees());
    SmartDashboard.putNumber("SMSOp_0Spd", sms_0_Optimized.speedMetersPerSecond);
    
    SwerveModuleState sms_1 = new SwerveModuleState(swerveModuleStates[1].speedMetersPerSecond,new Rotation2d(newAng));
    SwerveModuleState sms_1_Optimized = SwerveModuleState.optimize(sms_1, swerveModuleStates[1].angle);
    SmartDashboard.putNumber("SMSOp_1Ang", sms_1_Optimized.angle.getDegrees());
    SmartDashboard.putNumber("SMSOp_1Spd", sms_1_Optimized.speedMetersPerSecond);

    SwerveModuleState sms_2 = new SwerveModuleState(swerveModuleStates[2].speedMetersPerSecond,new Rotation2d(newAng));
    SwerveModuleState sms_2_Optimized = SwerveModuleState.optimize(sms_2, swerveModuleStates[2].angle);
    SmartDashboard.putNumber("SMSOp_2Ang", sms_2_Optimized.angle.getDegrees());
    SmartDashboard.putNumber("SMSOp_2Spd", sms_2_Optimized.speedMetersPerSecond);
    SwerveModuleState sms_3 = new SwerveModuleState(swerveModuleStates[3].speedMetersPerSecond,new Rotation2d(newAng));
    SwerveModuleState sms_3_Optimized = SwerveModuleState.optimize(sms_3, swerveModuleStates[3].angle);
    SmartDashboard.putNumber("SMSOp_3Ang", sms_3_Optimized.angle.getDegrees());
    SmartDashboard.putNumber("SMSOp_3Spd", sms_3_Optimized.speedMetersPerSecond);
  }
  public int calcQuad(double _desiredAngle){
    int desiredQuad = 1;
    if(_desiredAngle >=0 && _desiredAngle < 90){
      desiredQuad = 1;
  }else if(_desiredAngle >=90 && _desiredAngle < 180){
      desiredQuad = 2;
  }else if(_desiredAngle <0 && _desiredAngle > -90){
      desiredQuad = -1;
  }else if(_desiredAngle <-90 && _desiredAngle > -180){
      desiredQuad = -2;
  }
    return desiredQuad;
  }
  public int calcAngleQuadrant(double _desiredAngle){
    int desiredQuad = calcQuad(_desiredAngle);
    SmartDashboard.putNumber("m_previousDesiredQuad", m_previousDesiredQuad);

    SmartDashboard.putNumber("desiredQuad", desiredQuad);
    int desiredQuadDirectionIndex = Math.abs(desiredQuad - 1);
    int previousDesiredQuadDirectionIndex = Math.abs(m_previousDesiredQuad - 1);
    int desiredQuadDirection = m_desiredQuadDirIndexArray[desiredQuadDirectionIndex][previousDesiredQuadDirectionIndex];
    int virtualQuadIndex = m_virtualQuad < 0 ? Math.abs(m_virtualQuad - 3) : Math.abs(m_virtualQuad -1);
    m_virtualQuad = m_virtualQuadArray[desiredQuadDirection][virtualQuadIndex];
    m_previousDesiredQuad = desiredQuad;
    return m_virtualQuad;
}

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
