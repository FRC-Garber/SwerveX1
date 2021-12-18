// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class  Constants {
    public static final class DRIVE{
        // Distance between centers of right and left wheels on robot
        public static final double kTrackWidth = 0.56;
        // Distance between front and back wheels on robot
        public static final double kWheelBase = 0.56;
        public static final double kWheelTrackBaseCircumference = 2*Math.PI*(Math.sqrt(Math.pow((kTrackWidth/2),2) + Math.pow((kWheelBase/2),2)));
        // Converts units/100ms to wheel m/sec
        public static final double kDriveVelRatio = 503.6815;
        public static final double kMaxSpeedMetersPerSecond = 4.324;
        public static final double kMaxAngularRateRadPerSecond = (1/kWheelTrackBaseCircumference/2*Math.PI)*kMaxSpeedMetersPerSecond;
        // frontLeft, frontRight, rearLeft, rearRight
        public static final SwerveDriveKinematics kDriveKinematics =
            new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),  
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
    }
    public static final class SWERVE{
        public static final double kPModuleDriveController = 1;
        public static final double kPModuleSteerController = 1;
        public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
        public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;
        public static final double kSteerEncoderCountsPerRev = 2048.0;
        public static final double kSteerCountsPerRadian = kSteerEncoderCountsPerRev / 2 * Math.PI;
        public static final double kFLAbsoluteOffsetInDegrees = 45;
        public static final double kFRAbsoluteOffsetInDegrees = 45;
        public static final double kBLAbsoluteOffsetInDegrees = 45;
        public static final double kBRAbsoluteOffsetInDegrees = 45;
    }
    public static final class OI{
        public static final double kDeadband = 0.1;
    }
}
