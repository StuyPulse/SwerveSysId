package com.stuypulse.robot.commands;

import com.stuypulse.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Dynamic extends CommandBase {

	private final RobotContainer robot;
	private final double volts;

	public Dynamic(RobotContainer robot, double volts) {
		this.robot = robot;
		this.volts = volts;
	}

	@Override
	public void initialize() {
		robot.logger.clear();
	}

	@Override
	public void execute() {
		robot.swerve.setLeftVoltage(volts);
		robot.swerve.setRightVoltage(volts);
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		robot.swerve.setLeftVoltage(0.0);
		robot.swerve.setRightVoltage(0.0);

		robot.logger.publish();
	}

}
