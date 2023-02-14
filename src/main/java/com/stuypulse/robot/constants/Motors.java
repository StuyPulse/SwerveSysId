/************************ PROJECT PHIL ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.revrobotics.CANSparkMax.IdleMode;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings.Chassis;
import com.stuypulse.robot.subsystems.SwerveModule;
import com.stuypulse.robot.subsystems.module.SimVoltageSwerveModule;
import com.stuypulse.robot.subsystems.module.VoltageSwerveModule;
import com.stuypulse.robot.util.SparkMaxConfig;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.SmartAngle;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/*-
 * File containing all of the configurations that different motors require.
 *
 * Such configurations include:
 *  - If it is Inverted
 *  - The Idle Mode of the Motor
 *  - The Current Limit
 *  - The Open Loop Ramp Rate
 */
public interface Motors {
	SparkMaxConfig DRIVE = new SparkMaxConfig(false, IdleMode.kBrake, 40, 0);
	SparkMaxConfig TURN  = new SparkMaxConfig(false, IdleMode.kBrake, 20, 0);

	public interface Modules {
		public interface FrontRight {
			String ID = "Front Right";
			int DRIVE_PORT = 10;
			int TURN_PORT = 11;
			Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(0);
			Translation2d MODULE_OFFSET = new Translation2d(Chassis.WIDTH * +0.5, Chassis.HEIGHT * -0.5);
		}
	
		public interface FrontLeft {
			String ID = "Front Left";
			int DRIVE_PORT = 12;
			int TURN_PORT = 13;
			Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(0);
			Translation2d MODULE_OFFSET = new Translation2d(Chassis.WIDTH * +0.5, Chassis.HEIGHT * +0.5);
		}
	
		public interface BackLeft {
			String ID = "Back Left";
			int DRIVE_PORT = 14;
			int TURN_PORT = 15;
			Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(0);
			Translation2d MODULE_OFFSET = new Translation2d(Chassis.WIDTH * -0.5, Chassis.HEIGHT * +0.5);
		}
	
		public interface BackRight {
			String ID = "Back Right";
			int DRIVE_PORT = 16;
			int TURN_PORT = 17;
			Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(0);
			Translation2d MODULE_OFFSET = new Translation2d(Chassis.WIDTH * -0.5, Chassis.HEIGHT * -0.5);
		}
	
		public static SwerveModule makeModule(String id, int turnId, int driveId,
				Rotation2d absoluteOffset, Translation2d moduleOffset) {
			return Robot.isReal()
				? new VoltageSwerveModule(id, moduleOffset, turnId, absoluteOffset, driveId)
				: new SimVoltageSwerveModule(id, moduleOffset);
		}
	}
}
