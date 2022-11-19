/************************ PROJECT PHIL ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.revrobotics.CANSparkMax.IdleMode;
import com.stuypulse.robot.constants.Settings.Chassis;
import com.stuypulse.robot.subsystems.SwerveModule;
import com.stuypulse.robot.subsystems.module.SimVoltageSwerveModule;
import com.stuypulse.robot.subsystems.module.VoltageSwerveModule;
import com.stuypulse.robot.util.SparkMaxConfig;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.SmartAngle;

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
	SparkMaxConfig TURN_CONFIG = new SparkMaxConfig(false, IdleMode.kBrake, 60, 0.0);
	SparkMaxConfig DRIVE_CONFIG = new SparkMaxConfig(false, IdleMode.kBrake, 60, 0.0);

	public interface Modules {
		public interface FrontRight {
			String ID = "Front Right";
			int DRIVE_PORT = 3;
			int TURN_PORT = 4;
			int ENCODER_PORT = 1;
			SmartAngle ABSOLUTE_OFFSET = new SmartAngle(ID + "/Absolute Offset", Angle.fromDegrees(143));
			Translation2d MODULE_OFFSET = new Translation2d(Chassis.WIDTH * +0.5, Chassis.HEIGHT * -0.5);
		}
	
		public interface FrontLeft {
			String ID = "Front Left";
			int DRIVE_PORT = 1;
			int TURN_PORT = 2;
			int ENCODER_PORT = 3;
			SmartAngle ABSOLUTE_OFFSET = new SmartAngle(ID + "/Absolute Offset", Angle.fromDegrees(36));
			Translation2d MODULE_OFFSET = new Translation2d(Chassis.WIDTH * +0.5, Chassis.HEIGHT * +0.5);
		}
	
		public interface BackLeft {
			String ID = "Back Left";
			int DRIVE_PORT = 5;
			int TURN_PORT = 6;
			int ENCODER_PORT = 2;
			SmartAngle ABSOLUTE_OFFSET = new SmartAngle(ID + "/Absolute Offset", Angle.fromDegrees(-80.5));
			Translation2d MODULE_OFFSET = new Translation2d(Chassis.WIDTH * -0.5, Chassis.HEIGHT * +0.5);
		}
	
		public interface BackRight {
			String ID = "Back Right";
			int DRIVE_PORT = 7;
			int TURN_PORT = 8;
			int ENCODER_PORT = 0;
			SmartAngle ABSOLUTE_OFFSET = new SmartAngle(ID + "/Absolute Offset", Angle.fromDegrees(142.3));
			Translation2d MODULE_OFFSET = new Translation2d(Chassis.WIDTH * -0.5, Chassis.HEIGHT * -0.5);
		}
	
		public static SwerveModule makeModule(String id, int turnId, int driveId, int encoderPort,
				SmartAngle absoluteOffset, Translation2d moduleOffset) {
			// return new VoltageSwerveModule(id, moduleOffset, turnId, encoderPort, absoluteOffset, driveId);
			return new SimVoltageSwerveModule(id, moduleOffset);
		}
	}
}
