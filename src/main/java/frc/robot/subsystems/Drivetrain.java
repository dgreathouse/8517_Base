// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
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
  StatusSignal<Double> m_yaw;
  StatusSignal<Double> m_angularVelocityZ;
  
  private PIDController m_turnPID = new PIDController(g.DRIVETRAIN.TURN_KP, g.DRIVETRAIN.TURN_KI, g.DRIVETRAIN.TURN_KD);
  /** Creates a new Drivetrain. */
  
  public Drivetrain() {
    initialize();
  }
  
  @SuppressWarnings("unused")
    public void initialize() {
    // Define the IMU/Gyro called Pigeon2 that is on the CANIvore Bus network
    m_pigeon2 = new Pigeon2(g.CAN_IDS_CANIVORE.PIGEON2, g.CAN_IDS_CANIVORE.NAME);
    m_yaw = m_pigeon2.getYaw();
    m_yaw.setUpdateFrequency(g.CAN_IDS_CANIVORE.UPDATE_FREQ_HZ);
    m_angularVelocityZ = m_pigeon2.getAngularVelocityZDevice();
    m_angularVelocityZ.setUpdateFrequency(g.CAN_IDS_CANIVORE.UPDATE_FREQ_HZ);
    
    // Define the swerve module constants for each module.
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
    // TODO: Adjust Kinematics for the number of swerve modules
    m_kinematics = new SwerveDriveKinematics(g.SWERVE.Modules[0].m_location, g.SWERVE.Modules[1].m_location, g.SWERVE.Modules[2].m_location);
    m_odometry = new SwerveDriveOdometry(m_kinematics, g.ROBOT.RobotActualAngle, g.SWERVE.Positions);
    m_field = new Field2d();

    m_turnPID.enableContinuousInput(-Math.PI, Math.PI);
    m_turnPID.setTolerance(Math.toRadians(.1), 1);

    // Setup StatusSignals
    

    m_odometryThread = new OdometryThread();
    m_odometryThread.start();

    g.DASHBOARD.updates.add(this);
  }

  public void updatePositions() {
    for (int i = 0; i < g.SWERVE.Count; i++) {
      g.SWERVE.Positions[i] = g.SWERVE.Modules[i].updatePosition();
    }
  }



  /**
   * Drive in a robot centric mode
   * @param _speeds The ChassisSpeeds contains the X,Y direction and robot rotational speed
   */
  public void driveRobotCentric(ChassisSpeeds _speeds) {
    var swerveStates = m_kinematics.toSwerveModuleStates(_speeds);
    setSwerveModules(swerveStates, true, true);
  }

   /**
   * Drive in a field centric mode. The X,Y are changed to compensate for the Field orientation
   * @param _speeds The ChassisSpeeds contains the X,Y direction and robot rotational speed
   */
  public void driveFieldCentric(ChassisSpeeds _speeds) {
    var robotCentricSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(_speeds, g.ROBOT.RobotActualAngle);
    var swerveStates = m_kinematics.toSwerveModuleStates(robotCentricSpeeds);
    setSwerveModules(swerveStates, true, true);
  }
  /**
   * Drive in a angle field centric mode. The X,Y are changed to compensate for the Field orientation.
   * The rotational speed is changed to PID to the requested angle from g.ROBOT.AngleTarget_deg
   * @param _xSpeeds The requested X speed in MPS
   * @param _ySpeeds The requested Y speed in MPS
   * @param _enableSteer enable the Steer motor
   * @param _enableDrive enable the Drive motor
   */
  public void driveAngleFieldCentric(double _xSpeed, double _ySpeed, boolean _enableSteer, boolean _enableDrive) {
    double rotationalSpeed = m_turnPID.calculate(g.ROBOT.RobotActualAngle.getRadians(), Math.toRadians(g.ROBOT.AngleTarget_deg));
    rotationalSpeed = MathUtil.applyDeadband(rotationalSpeed, 0.01);
    var robotCentricSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(_xSpeed, _ySpeed, rotationalSpeed, g.ROBOT.RobotActualAngle);
    var swerveStates = m_kinematics.toSwerveModuleStates(robotCentricSpeeds);
    setSwerveModules(swerveStates, _enableSteer, _enableDrive);
  }
  /**
   * Drive in Polar Field Centric. This will drive at a specified angle, speed and maintain the robot
   * heading based on g.ROBOT.AngleTarget_deg. This method is used in autonomous when we only want 
   * to specify the speed and angle to drive at.
   * @param _driveAngle_deg The angle to drive at
   * @param _speed_mps The speed in MPS to drive at
   * @param _enableSteer enable the Steer Motor
   * @param _enableDrive enable the Drive Motor
   */
  public void drivePolarFieldCentric(double _driveAngle_deg, double _speed_mps, boolean _enableSteer,
      boolean _enableDrive) {
    double y = Math.sin(Units.degreesToRadians(_driveAngle_deg)) * _speed_mps;
    double x = Math.cos(Units.degreesToRadians(_driveAngle_deg)) * _speed_mps;
    driveAngleFieldCentric(x, y, _enableSteer, _enableDrive);
  }
  /**
   * This is meant to cause the drive train to do a fast stop by changing the wheels to ~45 degrees. 
   * This can be used to stop fast or prevent the robot from being pushed.
   */
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
  
  /**
   * The only method that sets the Swerve Module states to drive the motors
   * @param _states An array of module speeds and angles developed from the drive modes
   * @param _enableSteer enable the Steer Motor
   * @param _enableDrive enable the Drive Motor
   */
  public void setSwerveModules(SwerveModuleState[] _states, boolean _enableSteer, boolean _enableDrive) {
    for (int i = 0; i < g.SWERVE.Count; i++) {
      g.SWERVE.Modules[i].setDesiredState(_states[i], _enableSteer, _enableDrive);
    }
  }

  /**
   * Reset the GYRO/IMU to zero heading.
   * 
   */
  public void resetYaw() {
    m_pigeon2.setYaw(0.0);
  }

  /**
   * Set the current drive mode
   * @param _driveMode The drive mode the default command should use
   */
  public void setDriveMode(DriveMode _driveMode) {
    g.DRIVETRAIN.driveMode = _driveMode;
  }

  /**
   * Is the robot rotational on target for g.ROBOT.AngleTarget_deg
   * @return if the rotation of the robot is on target
   */
  public boolean isRotateAtTarget(){
    return m_turnPID.atSetpoint();
  }

  /**
   * Set the drive speed multiplier for the amount of voltage to apply to the drive motor.
   * A value of 1.0 is max speed. A value of 0 will disable the drive motor.
   * @param _val
   */
  public void setDriveSpeedMultiplier(double _val) {
    g.DRIVETRAIN.driveSpeedMultiplier = _val;
  }

  /**
   * Use the drive controller Left X,Y to set 1 of 8 discrete robot target angles if the X and Y are greater than a preset value
   */
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

  /**
   * Directly set the g.ROBOT.AngleTarget_deg to a angle. Generally this is used in autonomous or by a button.
   * @param _angle_deg The target angle the robot should PID to in AngleFieldCentric Mode.
   */
  public void setAngleTarget(double _angle_deg) {
    g.ROBOT.AngleTarget_deg = _angle_deg;
  }
  private Rotation2d getRobotAngle() {
    return new Rotation2d(g.ROBOT.AngleActual_deg);
  }
  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    setAngleTarget();
    
  }

  /* Perform swerve module updates in a separate thread to minimize latency */
  private class OdometryThread extends Thread {
    public OdometryThread() {
      super();
    }

    @Override
    public void run() {
       /* If using the CANIvore the signals should be set to update at a fast rate.
       * Each swerve module signal and the Pigeon are set to update at ~250Hz = 4ms
       * The Robot actual angle is Latency Compensated.
       * There is no blocking in this approach or waiting for all signals. 
       * There may be a sync issue but when it comes to distances traveled it is less than an inch.
       * Since the Yaw is Latenceny compensated the only issue left is Yaw vs Swerve positions or Pose
       * Pose is updated at the same rate in this thread at the same rate as the updated signals.
       * 
       */
      while (true) {
        /* Now update odometry */
        for (int i = 0; i < g.SWERVE.Count; i++) {
          g.SWERVE.Positions[i] = g.SWERVE.Modules[i].updatePosition();
        }
        g.ROBOT.AngleActual_deg = StatusSignal.getLatencyCompensatedValue(m_yaw, m_angularVelocityZ);
        g.ROBOT.RobotActualAngle = getRobotAngle();
        g.ROBOT.Pose = m_odometry.update(g.ROBOT.RobotActualAngle, g.SWERVE.Positions);
        m_field.setRobotPose(g.ROBOT.Pose);
        try {
          Thread.sleep(5);
        } catch (InterruptedException e) {
          System.out.println(e.getMessage());
        }
      }
    }
  }
    /**
   * Called by separate thread to put stuff to the dashboard at a slower rate than the main periodic
   */
  public void updateDashboard() {
    
  }
}
