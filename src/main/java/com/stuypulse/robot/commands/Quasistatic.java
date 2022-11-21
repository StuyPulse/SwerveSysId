package com.stuypulse.robot.commands;

import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Quasistatic extends CommandBase {
	
	private final RobotContainer robot;

	private Number rampRate;

	private double volts;

	public Quasistatic(RobotContainer robot, Number rampRate) {
		this.robot = robot;
		this.rampRate = rampRate;
	}

	@Override
	public void initialize() {
		robot.logger.clear();

		volts = 0;
	}

	@Override
	public void execute() {
		robot.swerve.setLeftVoltage(volts);
		robot.swerve.setRightVoltage(volts);

		volts += rampRate.doubleValue() * Settings.dT;
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		robot.swerve.setLeftVoltage(0.0);
		robot.swerve.setRightVoltage(0.0);

		robot.logger.publish("slow", rampRate.doubleValue() > 0);
	}
	
}
