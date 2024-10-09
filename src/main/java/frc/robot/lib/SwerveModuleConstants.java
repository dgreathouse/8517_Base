// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

/** Add your docs here. */
public class SwerveModuleConstants {
        public String NAME = "";
        public String CAN_NETWORK_ID = g.CAN_IDS_ROBORIO.NAME;
        public int DRIVE_CAN_ID;
        public boolean DRIVE_IS_REVERSED = false;
        public int STEER_CAN_ID;
        public boolean STEER_IS_REVERSED = false;
        public int CANCODER_ID;
        public double CANCODER_OFFSET_ROT;
        public double LOCATION_X_METER;
        public double LOCATION_Y_METER;

        public SwerveModuleConstants(
                String _name,
                int _driveCanID, boolean _driveIsReversed,
                int _steerCanID, boolean _steerIsReversed,
                int _canCoderID, double _canCoderOffset_rot,
                double _locationX_meter, double _locationY_meter) {
            NAME = _name;
            DRIVE_CAN_ID = _driveCanID;
            DRIVE_IS_REVERSED = _driveIsReversed;
            STEER_CAN_ID = _steerCanID;
            STEER_IS_REVERSED = _steerIsReversed;
            CANCODER_ID = _canCoderID;
            CANCODER_OFFSET_ROT = _canCoderOffset_rot;
            LOCATION_X_METER = _locationX_meter;
            LOCATION_Y_METER = _locationY_meter;
        }

    }
