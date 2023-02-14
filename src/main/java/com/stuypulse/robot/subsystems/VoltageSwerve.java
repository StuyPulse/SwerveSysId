package com.stuypulse.robot.subsystems;

import static com.stuypulse.robot.constants.Motors.Modules.*;

import java.util.Arrays;

import com.kauailabs.navx.frc.AHRS;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.module.VoltageSwerveModule;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.util.AngleVelocity;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VoltageSwerve extends SubsystemBase {
		
	// Subsystems
	private final SwerveModule[] modules = new SwerveModule[] {
		makeModule(FrontRight.ID, FrontRight.TURN_PORT, FrontRight.DRIVE_PORT,
				FrontRight.ABSOLUTE_OFFSET, FrontRight.MODULE_OFFSET),
		makeModule(FrontLeft.ID, FrontLeft.TURN_PORT, FrontLeft.DRIVE_PORT,
				FrontLeft.ABSOLUTE_OFFSET, FrontLeft.MODULE_OFFSET),
		makeModule(BackLeft.ID, BackLeft.TURN_PORT, BackLeft.DRIVE_PORT,
				BackLeft.ABSOLUTE_OFFSET, BackLeft.MODULE_OFFSET),
		makeModule(BackRight.ID, BackRight.TURN_PORT, BackRight.DRIVE_PORT,
				BackRight.ABSOLUTE_OFFSET, BackRight.MODULE_OFFSET)
	};

	private final SwerveDriveKinematics kinematics;
	private final SwerveDriveOdometry odometry;

	// Sensors
	private final AHRS gyro;
	private final AngleVelocity angleFilter;
	private double angularVelocity;

	private double leftVoltage, rightVoltage;

	private Field2d field;

	public VoltageSwerve() {
		gyro = new AHRS(SPI.Port.kMXP);

		kinematics = new SwerveDriveKinematics(getModuleLocations());
		odometry = new SwerveDriveOdometry(kinematics, getRotation2d(), getModulePositions());

		angleFilter = new AngleVelocity();

		field = new Field2d();
		SmartDashboard.putData(field);
	}

	private Translation2d[] getModuleLocations() {
		return Arrays.stream(modules).map(x -> x.getLocation()).toArray(Translation2d[]::new);
	}

	private SwerveModuleState[] getModuleStates() {
		return Arrays.stream(modules).map(x -> x.getState()).toArray(SwerveModuleState[]::new);
	}

	private SwerveModulePosition[] getModulePositions() {
		return Arrays.stream(modules).map(x -> x.getPosition()).toArray(SwerveModulePosition[]::new);
	}

	public void setLeftVoltage(double voltage) {
		leftVoltage = voltage;
	}

	public double getLeftVoltage() {
		return leftVoltage;
	}

	// NOTE: might be incorrect if encoders don't both start at zero
	public double getLeftPosition() {
		return (modules[1].getDistance() + modules[2].getDistance()) / 2.0;
	}

	public double getLeftVelocity() {
		return (modules[1].getVelocity() + modules[2].getVelocity()) / 2.0;
	}

	public void setRightVoltage(double voltage) {
		rightVoltage = voltage;
	}

	public double getRightVoltage() {
		return rightVoltage;
	}

	// NOTE: might be incorrect if encoders don't both start at zero
	public double getRightPosition() {
		return (modules[0].getDistance() + modules[3].getDistance()) / 2.0;
	}

	public double getRightVelocity() {
		return (modules[0].getVelocity() + modules[3].getVelocity()) / 2.0;
	}

	// return ccw+ angle
	public Rotation2d getRotation2d() {
		return gyro.getRotation2d();
	}

	public double getAngularVelocity() {
		if (RobotBase.isReal())
			return Math.toRadians(gyro.getRate());
		else
			return angularVelocity;
	}


	@Override
	public void periodic() {
		odometry.update(getRotation2d(), getModulePositions());

		modules[0].setVoltage(rightVoltage);
		modules[1].setVoltage(leftVoltage);
		modules[2].setVoltage(leftVoltage);
		modules[3].setVoltage(rightVoltage);

		field.setRobotPose(odometry.getPoseMeters());

		SmartDashboard.putNumber("Swerve/Left Voltage", leftVoltage);
		SmartDashboard.putNumber("Swerve/Right Voltage", leftVoltage);
		SmartDashboard.putNumber("Swerve/Left Pos (rotations)", getLeftPosition());
		SmartDashboard.putNumber("Swerve/Left Vel (rotations per s)", getLeftVelocity());
		SmartDashboard.putNumber("Swerve/Right Pos (rotations)", getRightPosition());
		SmartDashboard.putNumber("Swerve/Right Vel (rotations per s)", getRightVelocity());
	}

	@Override
	public void simulationPeriodic() {
		var speeds = kinematics.toChassisSpeeds(getModuleStates());

        gyro.setAngleAdjustment(gyro.getAngle() - Math.toDegrees(speeds.omegaRadiansPerSecond * Settings.dT));

		angularVelocity = angleFilter.get(Angle.fromRotation2d(getRotation2d()));
	}

}
