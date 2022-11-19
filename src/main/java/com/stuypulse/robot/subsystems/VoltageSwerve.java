package com.stuypulse.robot.subsystems;

import static com.stuypulse.robot.constants.Motors.Modules.*;

import com.kauailabs.navx.frc.AHRS;
import com.stuypulse.robot.subsystems.module.VoltageSwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VoltageSwerve extends SubsystemBase {
		
	// Subsystems
	private final SwerveModule[] modules = new SwerveModule[] {
		makeModule(FrontRight.ID, FrontRight.TURN_PORT, FrontRight.DRIVE_PORT,
				FrontRight.ENCODER_PORT, FrontRight.ABSOLUTE_OFFSET, FrontRight.MODULE_OFFSET),
		makeModule(FrontLeft.ID, FrontLeft.TURN_PORT, FrontLeft.DRIVE_PORT,
				FrontLeft.ENCODER_PORT, FrontLeft.ABSOLUTE_OFFSET, FrontLeft.MODULE_OFFSET),
		makeModule(BackLeft.ID, BackLeft.TURN_PORT, BackLeft.DRIVE_PORT,
				BackLeft.ENCODER_PORT, BackLeft.ABSOLUTE_OFFSET, BackLeft.MODULE_OFFSET),
		makeModule(BackRight.ID, BackRight.TURN_PORT, BackRight.DRIVE_PORT,
				BackRight.ENCODER_PORT, BackRight.ABSOLUTE_OFFSET, BackRight.MODULE_OFFSET)
	};

	// Sensors
	private final AHRS gyro;

	private double leftVoltage, rightVoltage;

	public VoltageSwerve() {
		gyro = new AHRS(SPI.Port.kMXP);
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


	@Override
	public void periodic() {
		modules[0].setVoltage(rightVoltage);
		modules[1].setVoltage(leftVoltage);
		modules[2].setVoltage(leftVoltage);
		modules[3].setVoltage(rightVoltage);

		SmartDashboard.putNumber("Swerve/Left Voltage", leftVoltage);
		SmartDashboard.putNumber("Swerve/Right Voltage", leftVoltage);
		SmartDashboard.putNumber("Swerve/Left Pos", getLeftPosition());
		SmartDashboard.putNumber("Swerve/Left Vel", getLeftVelocity());
		SmartDashboard.putNumber("Swerve/Right Pos", getRightPosition());
		SmartDashboard.putNumber("Swerve/right Vel", getRightVelocity());
	}

}
