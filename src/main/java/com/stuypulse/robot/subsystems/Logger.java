package com.stuypulse.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Logger extends SubsystemBase {

	// 200 samples/s * 20 seconds * 9 doubles per sample
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

	public void publish(String test, boolean forwards) {
		String path = test + (forwards ? "-forward" : "-backward");
		String data = "";

		for (int i = 0; i < values.size(); i += 9) {
			if (i != 0)
				data += ",";

			data += "[";

			for (int j = 0; j < 9; j++) {
				if (j != 0)
					data += ",";

				data += values.get(i + j).toString();
			}

			data += "]";
		}

		SmartDashboard.putString(path, data);
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
		values.add(swerve.getAngularVelocity() / Math.PI / 2.0);
	}
}
