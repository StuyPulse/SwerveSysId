package com.stuypulse.robot.subsystems.module;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve.Encoder;
import com.stuypulse.robot.subsystems.SwerveModule;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.math.Angle;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MaxSwerveModuleImpl extends SwerveModule {

    private interface Turn {
        double kP = 3.5;
        double kI = 0.0;
        double kD = 0.0;
    }

    // module data
    private String id;
    private Translation2d location;
    private Rotation2d angleOffset;

    // turn
    private CANSparkMax turnMotor;
    private SparkMaxAbsoluteEncoder absoluteEncoder;

    // drive
    private CANSparkMax driveMotor;
    private RelativeEncoder driveEncoder;

    // controller
    private AngleController turnPID;

    private double voltage;

    public MaxSwerveModuleImpl(String id, Translation2d location, int turnCANId, Rotation2d angleOffset, int driveCANId) {

        // module data
        this.id = id;
        this.location = location;
        this.angleOffset = angleOffset;

        // turn 
        turnMotor = new CANSparkMax(turnCANId, MotorType.kBrushless);
        turnPID = new AnglePIDController(Turn.kP, Turn.kI, Turn.kD);
        absoluteEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);
        configureTurnMotor(angleOffset);
        
        // drive
        driveMotor = new CANSparkMax(driveCANId, MotorType.kBrushless);
        configureDriveMotor();
    }   

    private void configureTurnMotor(Rotation2d angleOffset) {
        turnMotor.restoreFactoryDefaults();
        
        absoluteEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);
        absoluteEncoder.setPositionConversionFactor(Encoder.Turn.POSITION_CONVERSION);
        absoluteEncoder.setVelocityConversionFactor(Encoder.Turn.VELOCITY_CONVERSION);
        absoluteEncoder.setZeroOffset(angleOffset.getRotations());
        absoluteEncoder.setInverted(true);

        turnMotor.enableVoltageCompensation(12.0);

        Motors.TURN.config(turnMotor);
    }

    private void configureDriveMotor() {
        driveMotor.restoreFactoryDefaults();
        
        driveEncoder = driveMotor.getEncoder();
        // driveEncoder.setPositionConversionFactor(Encoder.Drive.POSITION_CONVERSION);
        // driveEncoder.setVelocityConversionFactor(Encoder.Drive.VELOCITY_CONVERSION);
        
        driveMotor.enableVoltageCompensation(12.0);
        Motors.DRIVE.config(driveMotor);
        driveEncoder.setPosition(0);
    }

    @Override
    public Translation2d getLocation() {
        return location;
    }

    @Override
    public double getVelocity() {
        return driveEncoder.getVelocity();
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity() * Settings.Swerve.Encoder.Drive.WHEEL_CIRCUMFERENCE, getRotation2d());
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDistance(), getRotation2d());
    }

    private Rotation2d getAbsolutePosition() {
        return Rotation2d.fromRotations(absoluteEncoder.getPosition());
    }

    private Rotation2d getRotation2d() {
        return getAbsolutePosition().minus(angleOffset);
    }

    @Override
    public double getDistance() {
        return driveEncoder.getPosition();
    }

    @Override
    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }

    public void periodic() {
        turnMotor.set(turnPID.update(Angle.kZero, Angle.fromRotation2d(getRotation2d())));
        driveMotor.setVoltage(voltage);

        SmartDashboard.putNumber(id + "/Angle", getRotation2d().getDegrees());
        SmartDashboard.putNumber(id + "/Absolute Angle", getAbsolutePosition().getDegrees());

        SmartDashboard.putNumber(id + "/Velocity", getVelocity());

        SmartDashboard.putNumber(id + "/Drive Voltage", voltage);
    }

}
