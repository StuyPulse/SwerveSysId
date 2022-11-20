package com.stuypulse.robot.commands;

import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Quasistatic extends CommandBase {
	
	private final RobotContainer robot;
	private final double voltDelta;

	private double volts;

	public Quasistatic(RobotContainer robot, double rampRate) {
		this.robot = robot;
		voltDelta = rampRate * Settings.dT;

		volts = 0;
	}

	@Override
	public void initialize() {
		robot.logger.clear();
	}

	@Override
	public void execute() {
		robot.swerve.setLeftVoltage(volts);
		robot.swerve.setRightVoltage(volts);

		volts += voltDelta;
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		robot.swerve.setLeftVoltage(0.0);
		robot.swerve.setRightVoltage(0.0);

		robot.logger.publish("slow", voltDelta > 0);
	}
	
}
