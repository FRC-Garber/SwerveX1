// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.OI;

public class DrivelineDefaultCommand extends CommandBase {
  SlewRateLimiter xSRL = new SlewRateLimiter(0.5);
  SlewRateLimiter ySRL = new SlewRateLimiter(0.5);
  SlewRateLimiter twistSRL = new SlewRateLimiter(0.5);
  /** Creates a new DrivelineDefaultCommand. */
  public DrivelineDefaultCommand() {
    addRequirements(RobotContainer.drivelineSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Read Joystick values for X, Y and rotation
    double x = RobotContainer.driverController.getRawAxis(1);
    double y = -RobotContainer.driverController.getRawAxis(2);
    double twist = RobotContainer.driverController.getRawAxis(3);

    // Limit the X,Y Rotation values so minor changes do not make motors move
    x = (Math.abs(x) < OI.kDeadband) ? x : 0; 
    y = (Math.abs(y) < OI.kDeadband) ? y : 0; 
    twist = (Math.abs(twist) < OI.kDeadband) ? twist : 0; 

    // Slew Rate Limit X,Y,Rotation values so drastic changes do not happen
    x = xSRL.calculate(x);
    y = ySRL.calculate(y);
    twist = twistSRL.calculate(twist);
                  //  X,   Y, Twist, getMotorAngle, getMotorAngle, Speed, speedDirection, FRAng, FRSpd,FLAng, FLSpd,BLAng, BLSpd,BRAng, BRSpd}
  

   for(int i = 0; i < 2; i++){
    RobotContainer.x = RobotContainer.inData[i][0];
    RobotContainer.y = RobotContainer.inData[i][1];
    RobotContainer.twist = RobotContainer.inData[i][2];
    RobotContainer.getMotorAngle = RobotContainer.inData[i][3];
    RobotContainer.drivelineSubsystem.drive(RobotContainer.x, RobotContainer.y, RobotContainer.twist, false);
   }
  
    // Call the DrivelineSubsystem drive method with the X, Y, Rotation values
    RobotContainer.drivelineSubsystem.drive(x, y, twist, false);
     
    // Given: X, Y, Twist come fron Joystick
    // Desired Angle is in  0
    //                  90      -90
    //                     +180- 
    // Issue is a +/- 180 transition causes a disconnect.
    // Given: Motor actual angle will read continuous past 180.
    // Given: Motor actual setAngle will be continuous
    // Optimize method does not take into account the discontinuity
    // 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
