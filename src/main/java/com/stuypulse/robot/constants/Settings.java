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
        double WIDTH = Units.inchesToMeters(26.504);
        double HEIGHT = Units.inchesToMeters(20.508);
    }
	
	public interface Encoder {
		public interface Drive {
			double WHEEL_DIAMETER = Units.inchesToMeters(3);
			double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
			double GEAR_RATIO = 1.0 / 4.71;
			
			double POSITION_CONVERSION = WHEEL_CIRCUMFERENCE * GEAR_RATIO;
			double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
		}

		public interface Turn {
			double POSITION_CONVERSION = 1;
			double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;

			double MIN_PID_INPUT = 0;
			double MAX_PID_INPUT = POSITION_CONVERSION;
		}
	}
}
