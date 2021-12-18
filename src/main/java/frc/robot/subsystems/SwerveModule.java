package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.ISwerveModule;

import frc.robot.Constants.DRIVE;
import frc.robot.Constants.SWERVE;

/** Add your docs here. */
public class SwerveModule implements ISwerveModule{

    private final WPI_TalonFX m_steerMotor;
    private final WPI_TalonFX m_driveMotor;
    private final CANCoder m_steerEncoder;
    private double m_steerEncoderAbsoluteOffset;
    private int[][] m_desiredQuadDirIndexArray = { { 0, 2, 1, 3 }, { 1, 0, 3, 2 }, { 2, 3, 0, 1 }, { 3, 1, 2, 0 } };
    private int[][] m_virtualQuadArray = { { 1, 2, 3, 4, -1, -2, -3, -4 }, { 2, 3, 4, 1, 1, -1, -2, -3 },
                                           { -1, 1, 2, 3, -2, -3, -4, -1 }, { -2, -1, 1, 2, 2, 1, -1, -2 } };
    private int m_virtualQuad = 1;
    private int m_previousDesiredQuad = 1;

    // Drive PID controller is a simple PID to calculate the error and then command the motor.
    // How can this work with the command getting lower as the closer it gets for a velocity.
    // This should be controlled by the TalonFX as .set(TalonFXControlMode.Velocity, targetVelocity_UnitsPer100ms);

    private final PIDController m_drivePIDController = 
        new PIDController(SWERVE.kPModuleDriveController, 0, 0);

    // Using a TrapezoidProfile PIDController to allow for smooth turning
    // This must be done by the RoboRIO because the encoder is connected to it.
    // CANIfier/CANCoder used so the encoder can be on the CAN bus and possibly used by the FalconFX
    private final ProfiledPIDController m_steeringPIDController =
      new ProfiledPIDController(
        SWERVE.kPModuleSteerController,
          0,
          0,
          new TrapezoidProfile.Constraints(
            SWERVE.kMaxModuleAngularSpeedRadiansPerSecond,
            SWERVE.kMaxModuleAngularAccelerationRadiansPerSecondSquared));
    public SwerveModule(int _steerCANID, int _driveCANID, int _steerCANCoderCANID){
        m_steerMotor = new WPI_TalonFX(_steerCANID);
        m_driveMotor = new WPI_TalonFX(_driveCANID);
        m_steerEncoder = new CANCoder(_steerCANCoderCANID);
        m_steerEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition, 100);
       // m_steerEncoder.set
    }
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocityInMPS(), new Rotation2d(getSteerAngleInRad()));
    }
    public double getSteerAngleInRad(){
        return m_steerEncoder.getPosition() * Math.PI/180.0;
    }
    public Rotation2d getRotation2d(){
        return new  Rotation2d(getSteerAngleInRad());
    }
    public double getDriveVelocityInMPS(){
        return m_driveMotor.getSelectedSensorVelocity(0) * DRIVE.kDriveVelRatio;
    }

    // Calculate the states and command the motors.
    public void setDesiredState(SwerveModuleState _desiredState){
        calcNewAngle(_desiredState);
        SwerveModuleState swerveModuleState = SwerveModuleState.optimize(_desiredState, getRotation2d());

        // // Calculate the PID for steering
         var steerOutput = m_steeringPIDController.calculate(getSteerAngleInRad(), swerveModuleState.angle.getRadians());
        // // Set the power of the steering motor
        m_steerMotor.set(ControlMode.PercentOutput, steerOutput);
        
        // // Calculate the PID for driving
        var driveOutput = m_drivePIDController.calculate(getDriveVelocityInMPS(), swerveModuleState.speedMetersPerSecond);
        driveOutput = driveOutput / DRIVE.kMaxSpeedMetersPerSecond;
        // // Set the power of the driving motor
         m_driveMotor.set(ControlMode.PercentOutput, driveOutput);


    }
    public void resetEncoders(){
        m_steerEncoder.setPosition(0);
        m_driveMotor.setSelectedSensorPosition(0);
    }
    private void calcNewAngle(SwerveModuleState _state){
        double newAng;
        int quad = 0;

        quad = calcAngleQuadrant(_state.angle.getDegrees());
        if(quad > 2){ newAng = Math.toRadians(180 + (180 + _state.angle.getDegrees()) );}else
        if(quad < -2){ newAng = Math.toRadians(-180 + (-180 + _state.angle.getDegrees()) );}
        else {newAng = _state.angle.getRadians();}

        _state.angle = new Rotation2d(newAng);
    }
    private int calcQuad(double _desiredAngle) {
        int desiredQuad = 1;
        if (_desiredAngle >= 0 && _desiredAngle < 90) { desiredQuad = 1; } else 
        if (_desiredAngle >= 90 && _desiredAngle < 180) { desiredQuad = 2; } else 
        if (_desiredAngle < 0 && _desiredAngle > -90) { desiredQuad = -1; } else 
        if (_desiredAngle < -90 && _desiredAngle > -180) { desiredQuad = -2; }
        return desiredQuad;
    }

    private int calcAngleQuadrant(double _desiredAngle) {
        int desiredQuad = calcQuad(_desiredAngle);
        int desiredQuadDirectionIndex = Math.abs(desiredQuad - 1);
        int previousDesiredQuadDirectionIndex = Math.abs(m_previousDesiredQuad - 1);
        int desiredQuadDirection = m_desiredQuadDirIndexArray[desiredQuadDirectionIndex][previousDesiredQuadDirectionIndex];
        int virtualQuadIndex = m_virtualQuad < 0 ? Math.abs(m_virtualQuad - 3) : Math.abs(m_virtualQuad - 1);
        m_virtualQuad = m_virtualQuadArray[desiredQuadDirection][virtualQuadIndex];
        m_previousDesiredQuad = desiredQuad;
        return m_virtualQuad;
    }
    public void setSteerAbsoluteOffset(double _angle){
        m_steerEncoderAbsoluteOffset = _angle;
    }
    /**
     * The CANEncoder has 4096 counts per revolution. The gearing ot the steer motor is 1:1.
     * Therefore the Degrees/Count = 360/4096 or Counts/Degree = 4096/360
     * 
     */
    private void initializeEncoderOffset(){
        double angleSteerAbs = m_steerEncoder.getAbsolutePosition();
        double angleSteerEnc = m_steerEncoder.getPosition();
       // m_steerEncoder.setPosition(m_steerEncoderAbsoluteOffset -)
    }
    

}
