/************************ PROJECT PHIL ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.util.Units;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
    int UPDATE_RATE = 200;
	double dT = 1.0 / UPDATE_RATE;
	
    public interface Chassis {
        double WIDTH = Units.inchesToMeters(29.0);
        double HEIGHT = Units.inchesToMeters(29.0);
        double MAX_SPEED = Units.feetToMeters(14.0);
    }
	
	public interface Encoder {
		public interface Drive {
			double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
			double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

			public interface Stages {
				// input / output
				double FIRST = 16.0 / 48.0;
				double SECOND = 28.0 / 16.0;
				double THIRD = 15.0 / 60.0;
			}

			double GEAR_RATIO = Stages.FIRST * Stages.SECOND * Stages.THIRD;

			// double POSITION_CONVERSION = WHEEL_CIRCUMFERENCE * GEAR_RATIO;
			double POSITION_CONVERSION = GEAR_RATIO;
			double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
		}
	}
}
