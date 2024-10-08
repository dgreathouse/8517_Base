// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.subsystems.SwerveModule;

/** Add your docs here. */
public class g {
    public static class CAN_IDS_ROBORIO {
        public static final String NAME = "rio";
        
    }

    public static class CAN_IDS_CANIVORE {
        public static final String NAME = "CANivore";
        public static final int PIGEON2 = 5;
    }

    public static class CV {
        public static final double DEGREES_TO_RADIANS = 0.017453292519943295;
        public static final double RADIANS_TO_DEGREES = 57.29577951308232;
        public static final double INCHES_TO_METERS = 0.0254;
    }

    public static class ROBOT {
        public static volatile double AngleActual_deg = 0;
        public static volatile double AngleTarget_deg = 0;
        public static final double PERIOD = 0.02;
        public static final double BATTERY_MAX_VOLTS = 12.8;
        public static final int PD_CANID = 1; // Power Distribution, Rev or CTRE

    }

    public static class OI {
        // Driver controller
        public static final int DRIVER_CONTROLLER_PORT = 0;

        // Operator controller
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }

    public static class SWERVE {
        public static SwerveModule[] Modules = new SwerveModule[g.SWERVE.MODULE.COUNT];
        public static class MODULE {
            public static final int COUNT = 4;
            
            public static SwerveModulePosition[] Positions = new SwerveModulePosition[COUNT];
            public static SwerveModuleConstants[] Constants = new SwerveModuleConstants[COUNT];
            public static class DRIVE {

            }

            public static class STEER {

            }
        }

        public static class CHASSIS {

        }

    }



    public static class DRIVETRAIN {

    }
    
}
