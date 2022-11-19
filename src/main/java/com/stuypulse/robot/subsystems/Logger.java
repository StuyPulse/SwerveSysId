package com.stuypulse.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.stuypulse.robot.commands.Quasistatic;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Logger extends SubsystemBase {

	private static final int LEN_VALUES = 36000;

	private final List<Double> values;
	private final VoltageSwerve swerve;

	public Logger(VoltageSwerve swerve) {
		values = new ArrayList<>(LEN_VALUES);
		this.swerve = swerve;
	}

	public void clear() {
		values.clear();
	}

	public void publish(Command test, boolean forwards) {
		String str = (test instanceof Quasistatic) ? "slow" : "fast";

		str += forwards ? "-forwards" : "-backwards";

		SmartDashboard.putNumberArray(str, values.toArray(new Double[values.size()]));
	}

	@Override
	public void periodic() {
		values.add(Timer.getFPGATimestamp());
		values.add(swerve.getLeftVoltage());
		values.add(swerve.getRightVoltage());
		values.add(swerve.getLeftPosition());
		values.add(swerve.getRightPosition());
		values.add(swerve.getLeftVelocity());
		values.add(swerve.getRightVelocity());
		values.add(swerve.getRotation2d().getDegrees() / 360.0);
		// fix this
		values.add(0.0);
	}
}
