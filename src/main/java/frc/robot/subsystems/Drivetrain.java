// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.DriveMode;
import frc.robot.lib.IUpdateDashboard;
import frc.robot.lib.SwerveModuleConstants;
import frc.robot.lib.g;

public class Drivetrain extends SubsystemBase implements IUpdateDashboard {
  Pigeon2 m_pigeon2;
  SwerveDriveKinematics m_kinematics;
  SwerveDriveOdometry m_odometry;
  Field2d m_field;
  OdometryThread m_odometryThread;
  private PIDController m_turnPID = new PIDController(g.DRIVETRAIN.TURN_KP, g.DRIVETRAIN.TURN_KI, g.DRIVETRAIN.TURN_KD);

  /** Creates a new Drivetrain. */
  public Drivetrain() {

    initialize();
  }

  @SuppressWarnings("unused")
  public void initialize() {
    // Define the IMU/Gyro called Pigeon2 that is on the CANIvore Bus network
    m_pigeon2 = new Pigeon2(g.CAN_IDS_CANIVORE.PIGEON2, g.CAN_IDS_CANIVORE.NAME);
    // Define the swerve module constants for each module.
    // TODO: Change this if g.SWERVE.Count changes from 3 to 4.
    g.SWERVE.Modules[0] = new SwerveModule(new SwerveModuleConstants(
        "FL",
        12, false,
        22, true,
        2, 0.2234,
        g.CHASSIS.WHEEL_BASE_X_m /2.0, g.CHASSIS.WHEEL_BASE_Y_m / 2.0));
    g.SWERVE.Modules[1] = new SwerveModule(new SwerveModuleConstants(
        "FR",
        13, true,
        23, true,
        3, 0.03637,
        g.CHASSIS.WHEEL_BASE_X_m /2.0, -g.CHASSIS.WHEEL_BASE_Y_m / 2.0));
    g.SWERVE.Modules[2] = new SwerveModule(new SwerveModuleConstants(
        "BR",
        11, false,
        21, true,
        1, -0.2324,
        -g.CHASSIS.WHEEL_BASE_X_m /2.0, 0.0));
    if(g.SWERVE.Count == 4){        
      g.SWERVE.Modules[3] = new SwerveModule(new SwerveModuleConstants(
          "BL",
          13, false,
          23, false,
          4, 0,
          0, 0));
    }
    for (int i = 0; i < g.SWERVE.Count; i++) {
      g.SWERVE.Positions[i] = new SwerveModulePosition();
    }
    updatePositions();
    m_kinematics = new SwerveDriveKinematics(g.SWERVE.Modules[0].m_location, g.SWERVE.Modules[1].m_location,
        g.SWERVE.Modules[2].m_location, g.SWERVE.Modules[3].m_location);
    m_odometry = new SwerveDriveOdometry(m_kinematics, getRobotAngle(), g.SWERVE.Positions);
    m_field = new Field2d();

    m_turnPID.enableContinuousInput(-Math.PI, Math.PI);
    m_turnPID.setTolerance(Math.toRadians(.1), 1);

    m_odometryThread = new OdometryThread();
    m_odometryThread.start();

    g.DASHBOARD.updates.add(this);
  }

  public void updatePositions() {
    for (int i = 0; i < g.SWERVE.Count; i++) {
      g.SWERVE.Positions[i] = g.SWERVE.Modules[i].updatePosition();
    }
  }

  public Rotation2d getRobotAngle() {
    return m_pigeon2.getRotation2d();
  }

  public void updateDashboard() {

  }

  public void driveRobotCentric(ChassisSpeeds _speeds) {
    var swerveStates = m_kinematics.toSwerveModuleStates(_speeds);
    setSwerveModules(swerveStates, true, true);
  }

  public void driveFieldCentric(ChassisSpeeds _speeds) {
    var robotCentricSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(_speeds, getRobotAngle());
    var swerveStates = m_kinematics.toSwerveModuleStates(robotCentricSpeeds);
    setSwerveModules(swerveStates, true, true);
  }

  public void driveAngleFieldCentric(double _xSpeed, double _ySpeed, boolean _enableSteer, boolean _enableDrive) {
    double rotationalSpeed = m_turnPID.calculate(getRobotAngle().getRadians(), Math.toRadians(g.ROBOT.AngleTarget_deg));
    rotationalSpeed = MathUtil.applyDeadband(rotationalSpeed, 0.01);
    var robotCentricSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(_xSpeed, _ySpeed, rotationalSpeed, getRobotAngle());
    var swerveStates = m_kinematics.toSwerveModuleStates(robotCentricSpeeds);
    setSwerveModules(swerveStates, _enableSteer, _enableDrive);
  }

  public void drivePolarFieldCentric(double _driveAngle_deg, double _speed_mps, boolean _enableSteer,
      boolean _enableDrive) {
    double y = Math.sin(Units.degreesToRadians(_driveAngle_deg)) * _speed_mps;
    double x = Math.cos(Units.degreesToRadians(_driveAngle_deg)) * _speed_mps;
    driveAngleFieldCentric(x, y, _enableSteer, _enableDrive);
  }

  public void setSwerveModules(SwerveModuleState[] _states, boolean _enableSteer, boolean _enableDrive) {
    for (int i = 0; i < g.SWERVE.Count; i++) {
      g.SWERVE.Modules[i].setDesiredState(_states[i], _enableSteer, _enableDrive);
    }
  }


  public void fastStop() {
    switch(g.SWERVE.Count){
      case 3:
        g.SWERVE.Modules[0].setDesiredState(new SwerveModuleState(0.0,new Rotation2d(Math.toRadians(45))), true, true); // FL
        g.SWERVE.Modules[1].setDesiredState(new SwerveModuleState(0.0,new Rotation2d(Math.toRadians(-45))), true, true); // FR
        g.SWERVE.Modules[2].setDesiredState(new SwerveModuleState(0.0,new Rotation2d(Math.toRadians(90))), true, true); // BR
        break;
      case 4:
        g.SWERVE.Modules[0].setDesiredState(new SwerveModuleState(0.0,new Rotation2d(Math.toRadians(45))), true, true); // FL
        g.SWERVE.Modules[1].setDesiredState(new SwerveModuleState(0.0,new Rotation2d(Math.toRadians(-45))), true, true); // FR
        g.SWERVE.Modules[2].setDesiredState(new SwerveModuleState(0.0,new Rotation2d(Math.toRadians(135))), true, true); // BR
        g.SWERVE.Modules[3].setDesiredState(new SwerveModuleState(0.0,new Rotation2d(Math.toRadians(-135))), true, true); // BL
        break;
    }
  }

  public void resetYaw() {
    m_pigeon2.setYaw(0.0);
  }

  public void setDriveMode(DriveMode _driveMode) {
    g.DRIVETRAIN.driveMode = _driveMode;
  }
  public boolean isRotateAtTarget(){
    return m_turnPID.atSetpoint();
  }
  public void setDriveSpeedMultiplier(double _val) {
    g.DRIVETRAIN.driveSpeedMultiplier = _val;
  }

  public void setAngleTarget() {
    double x = g.OI.driveController.getLeftX();
    double y = g.OI.driveController.getLeftY();
    double hyp = Math.hypot(x, y);
    double actualAngle = 0.0;
    if (Math.abs(hyp) > g.OI.ANGLE_TARGET_DEADBAND) {
      if (actualAngle >= -22.5 && actualAngle <= 22.5) { // North
        g.ROBOT.AngleTarget_deg = 0.0;
      } else if (actualAngle >= -67.5 && actualAngle < -22.5) { // North East
        g.ROBOT.AngleTarget_deg = -45.0;
      } else if (actualAngle >= -112.5 && actualAngle < -67.5) { // East
        g.ROBOT.AngleTarget_deg = -90.0;
      } else if (actualAngle >= -157.5 && actualAngle < -112.5) { // South East
        g.ROBOT.AngleTarget_deg = -135.0;
      } else if ((actualAngle >= 157.5 && actualAngle <= 180.0) || (actualAngle <= -157.5 && actualAngle > -179.99)) { // South
        g.ROBOT.AngleTarget_deg = 180.0;
      } else if (actualAngle <= 67.5 && actualAngle > 22.5) { // North West
        g.ROBOT.AngleTarget_deg = 45.0;
      } else if (actualAngle <= 112.5 && actualAngle > 67.5) { // West
        g.ROBOT.AngleTarget_deg = 90.0;
      } else if (actualAngle <= 157.5 && actualAngle > 112.5) { // South West
        g.ROBOT.AngleTarget_deg = 135.0;
      }
    }
  }

  public void setAngleTarget(double _angle_deg) {
    g.ROBOT.AngleTarget_deg = _angle_deg;
  }

  @Override
  public void periodic() {
    g.ROBOT.AngleActual_deg = getRobotAngle().getDegrees();
    setAngleTarget();
    // This method will be called once per scheduler run
  }

  /* Perform swerve module updates in a separate thread to minimize latency */
  private class OdometryThread extends Thread {
    public OdometryThread() {
      super();
    }

    @Override
    public void run() {

      /* Run as fast as possible, our signals will control the timing */
      while (true) {
        /* Now update odometry */
        for (int i = 0; i < g.SWERVE.Count; i++) {
          g.SWERVE.Positions[0] = g.SWERVE.Modules[0].updatePosition();
          g.SWERVE.Positions[1] = g.SWERVE.Modules[1].updatePosition();
          g.SWERVE.Positions[2] = g.SWERVE.Modules[2].updatePosition();
          g.SWERVE.Positions[3] = g.SWERVE.Modules[3].updatePosition();
        }
        g.ROBOT.Pose = m_odometry.update(getRobotAngle(), g.SWERVE.Positions);
        m_field.setRobotPose(g.ROBOT.Pose);
        try {
          Thread.sleep(5);
        } catch (InterruptedException e) {
          System.out.println(e.getMessage());
        }
      }
    }
  }
}
