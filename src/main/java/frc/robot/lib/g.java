// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SwerveModule;

/** Add your docs here. */
public class g {
    public static class CAN_IDS_ROBORIO {
        public static final String NAME = "rio";

    }

    public static class CAN_IDS_CANIVORE {
        public static final String NAME = "CANivore";
        public static final int PIGEON2 = 5;
        public static final double UPDATE_FREQ_HZ = 250;
    }

    public static class CV {
        public static final double DEGREES_TO_RADIANS = 0.017453292519943295;
        public static final double RADIANS_TO_DEGREES = 57.29577951308232;
        public static final double INCHES_TO_METERS = 0.0254;
    }

    public static class ROBOT {
        public static volatile double AngleActual_deg = 0;
        public static volatile double AngleTarget_deg = 0;
        public static volatile Rotation2d RobotActualAngle = new Rotation2d();
        public static final double PERIOD = 0.02;
        public static final double BATTERY_MAX_VOLTS = 12.8;
        public static final int PD_CANID = 1; // Power Distribution, Rev or CTRE
        public static Pose2d Pose = new Pose2d();
        public static final Drivetrain Drive = new Drivetrain();
        public static PowerDistribution Power = new PowerDistribution();

    }
    public static class DASHBOARD{
        public static Set<IUpdateDashboard> updates = new HashSet<>();
        public static ShuffleboardTab ShuffleBoardTab_Match = Shuffleboard.getTab("Match");
    }
    public static class OI {
        // Driver controller
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final CommandPS5Controller driveController = new CommandPS5Controller(DRIVER_CONTROLLER_PORT);
        public static final double ANGLE_TARGET_DEADBAND = 0.8;

        public static Trigger DRIVER_DRIVE_MODE_FIELDCENTRIC = driveController.povLeft();
        public static Trigger DRIVER_DRIVE_MODE_ROBOTCENTRIC = driveController.povRight();
        public static Trigger DRIVER_DRIVE_MODE_ANGLEFIELDCENTRIC = driveController.povDown();
        public static Trigger DRIVER_DRIVE_MODE_ROTATEFIELDCENTRIC = driveController.povUp();

        public static Trigger DRIVER_DRIVE_MODE_SPEED_HI = driveController.R1();
        public static Trigger DRIVER_DRIVE_MODE_SPEED_LOW = driveController.L1();

        public static Trigger DRIVER_RESET_YAW = driveController.create();

        // Operator controller
        public static final int OPERATOR_CONTROLLER_PORT = 1;
        public static final CommandPS5Controller operatorController = new CommandPS5Controller(OPERATOR_CONTROLLER_PORT);
    }

    public static class SWERVE {
        public static final int Count = 3;
        public static SwerveModule[] Modules = new SwerveModule[Count];
        public static volatile SwerveModulePosition[] Positions = new SwerveModulePosition[Count];
        //public static SwerveModuleConstants[] Constants = new SwerveModuleConstants[Count];

        public static class MODULE {

            public static class DRIVE {
                public static double PID_kp = 1.0;
                public static double PID_ki = 0.0;
                public static double PID_kv = 2.8256;
                public static double PID_ks = 0.0;
                private static final double MOTOR_PINION_TEETH = 12.0;
                private static final double GEAR_1_TEETH = 32.0;
                private static final double GEAR_2_DRIVE_TEETH = 28.0;
                private static final double GEAR_2_DRIVEN_TEETH = 18.0;
                private static final double GEAR_BEVEL_DRIVE_TEETH = 15.0;
                private static final double GEAR_BEVEL_DRIVEN_TEETH = 45.0;
                public static final double GEAR_RATIO = (GEAR_1_TEETH / MOTOR_PINION_TEETH)
                        * (GEAR_2_DRIVEN_TEETH / GEAR_2_DRIVE_TEETH)
                        * (GEAR_BEVEL_DRIVEN_TEETH / GEAR_BEVEL_DRIVE_TEETH);
                public static final double WHEEL_DIAMETER_m = .1015; // .10287
                private static final double WHEEL_CIRCUMFERENCE_m = Math.PI * WHEEL_DIAMETER_m;
                public static final double WHEEL_MotRotPerMeter = GEAR_RATIO / WHEEL_CIRCUMFERENCE_m;
                private static final double MOTOR_MAX_VELOCITY_RotPerMin = 5800.0;
                private static final double MOTOR_MAX_VELOCITY_RotPerSec = MOTOR_MAX_VELOCITY_RotPerMin / 60.0;
                private static final double WHEEL_MAX_VELOCITY_RotPerSec = MOTOR_MAX_VELOCITY_RotPerSec / GEAR_RATIO;
                private static final double MOTOR_PEAK_EFFICIENCY_percent = 100;
                public static final double MAX_VELOCITY_MeterPerSec = WHEEL_CIRCUMFERENCE_m
                        * WHEEL_MAX_VELOCITY_RotPerSec * MOTOR_PEAK_EFFICIENCY_percent / 100.0;
                // 10T = 3.63 Mps = 11.9 FtPSec,
                // 11T 3.994 Mps = 13.1 FtPSec,
                // 12T 4.35 Mps = 14.27 FtPSec, 91% Eff
                public static final double MAX_ANGULAR_VELOCITY_RadianPerSec = MAX_VELOCITY_MeterPerSec
                        * (1 / CHASSIS.WHEEL_BASE_MeterPerRad);
            }

            public static class STEER {
                public static double PID_kp = 0.1;
                public static double PID_ki = 0.0;
                private static final double MOTOR_PINION_TEETH = 8.0;
                private static final double MOTOR_DRIVE_GEAR_TEETH = 24.0;
                private static final double GEAR_1_DRIVE_TEETH = 14.0;
                private static final double GEAR_1_DRIVEN_TEETH = 72.0;
                private static final double CANCODER_GEAR_RATIO = 1.0;
                public static final double GEAR_RATIO = 1
                        / ((MOTOR_PINION_TEETH / MOTOR_DRIVE_GEAR_TEETH) * (GEAR_1_DRIVE_TEETH / GEAR_1_DRIVEN_TEETH));
                public static final double GEAR_RATIO_TO_CANCODER = GEAR_RATIO * CANCODER_GEAR_RATIO;
            }
        }
    }
    public static class CHASSIS {
        public static final double WHEEL_BASE_Y_m = 0.47738;
        public static final double WHEEL_BASE_X_m = 0.47851;
        private static final double WHEEL_BASE_XY_AVG_m = (WHEEL_BASE_Y_m + WHEEL_BASE_X_m) / 2.0;
        private static final double WHEEL_BASE_CIRCUMFERENCE_m = Math.PI * WHEEL_BASE_XY_AVG_m;
        private static final double WHEEL_BASE_MeterPerRad = WHEEL_BASE_CIRCUMFERENCE_m / (2 * Math.PI);
    }
    public static class DRIVETRAIN {
        public static final double TURN_KP = 10;
        public static final double TURN_KI = 0.0;
        public static final double TURN_KD = 0.0;
        public static DriveMode driveMode = DriveMode.ANGLE_FIELD_CENTRIC;
        public static double driveSpeedMultiplier = 1.0;
        public static final double ROTATE_FIELDCENTRIC_SPEED_RadPSec = 10.0;
    }

}
