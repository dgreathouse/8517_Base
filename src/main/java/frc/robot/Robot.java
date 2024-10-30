// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.autoCommandGroups.AutoDoNothing;
import frc.robot.commands.DrivetrainDefaultCommand;
import frc.robot.commands.IntakeDefaultCommand;
import frc.robot.commands.ShooterDefaultCommand;
import frc.robot.lib.DriveMode;
import frc.robot.lib.IUpdateDashboard;
import frc.robot.lib.g;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  /* -------- Old RobotContainer Stuff (Begin)------------- */
  private DrivetrainDefaultCommand m_drivetrainDefaultCommand = new DrivetrainDefaultCommand(g.ROBOT.Drive);
  private ShooterDefaultCommand m_shooterDefaultCommand = new ShooterDefaultCommand(g.ROBOT.Shooter);
  private IntakeDefaultCommand m_intakeDefaultCommand = new IntakeDefaultCommand(g.ROBOT.Intake);
  private SendableChooser<Command> autoChooser = new SendableChooser<>();
  private Notifier m_telemetry;
  /* -------- Old RobotContainer Stuff (Begin)------------- */

  public Robot() {
    super(g.ROBOT.LOOP_RATE_SEC); // Set the loop rate. Typical is 0.02 or 20ms. We are trying 10ms.
    LiveWindow.disableAllTelemetry();
    /* -------- Old RobotContainer Stuff (Begin)------------- */
    g.ROBOT.Drive.setDefaultCommand(m_drivetrainDefaultCommand);
    g.ROBOT.Intake.setDefaultCommand(m_intakeDefaultCommand);
    g.ROBOT.Shooter.setDefaultCommand(m_shooterDefaultCommand);
    configureBindings();

    autoChooser.setDefaultOption("Do Nothing", new AutoDoNothing());
    SmartDashboard.putData("Autonomous Play", autoChooser);

    m_telemetry = new Notifier(this::updateDashboard);
    m_telemetry.startPeriodic(0.1);

    /* -------- Old RobotContainer Stuff (End)------------- */
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {

  }

  /**
   * This function is called every {@link g.ROBOT.LOOP_RATE_SEC}, no matter the
   * mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = getAutonomousCommand();

    // schedule the autonomous command
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }

  /* -------- Old RobotContainer Stuff (Begin)------------- */
  private void updateDashboard() {
    for (IUpdateDashboard updates : g.DASHBOARD.updates) {
      updates.updateDashboard();
    }
  }

  private void configureBindings() {
    g.OI.DRIVER_RESET_YAW.onTrue(new InstantCommand(g.ROBOT.Drive::resetYaw, g.ROBOT.Drive));

    g.OI.DRIVER_DRIVE_MODE_ANGLEFIELDCENTRIC
        .onTrue(new InstantCommand(() -> g.ROBOT.Drive.setDriveMode(DriveMode.ANGLE_FIELD_CENTRIC), g.ROBOT.Drive));
    g.OI.DRIVER_DRIVE_MODE_FIELDCENTRIC
        .onTrue(new InstantCommand(() -> g.ROBOT.Drive.setDriveMode(DriveMode.FIELD_CENTRIC), g.ROBOT.Drive));
    g.OI.DRIVER_DRIVE_MODE_ROBOTCENTRIC
        .onTrue(new InstantCommand(() -> g.ROBOT.Drive.setDriveMode(DriveMode.ROBOT_CENTRIC), g.ROBOT.Drive));
    g.OI.DRIVER_DRIVE_MODE_ROTATEFIELDCENTRIC
        .onTrue(new InstantCommand(() -> g.ROBOT.Drive.setDriveMode(DriveMode.ROTATE_FIELD_CENTRIC), g.ROBOT.Drive));

    g.OI.DRIVER_DRIVE_MODE_SPEED_HI.onTrue(new InstantCommand(() -> g.ROBOT.Drive.setDriveSpeedMultiplier(1.0)));
    g.OI.DRIVER_DRIVE_MODE_SPEED_HI.onTrue(new InstantCommand(() -> g.ROBOT.Drive.setDriveSpeedMultiplier(0.5)));

    g.OI.DRIVER_TOGGLE_DRIVETRAIN_ENABLE.onTrue(new InstantCommand(() -> g.ROBOT.Drive.toggleMotorsEnable()));

    g.OI.DRIVER_SHOOTER_HIGH.onTrue(new InstantCommand(() -> g.ROBOT.Shooter.setShooterFeedLong()));
    g.OI.DRIVER_SHOOTER_LOW.onTrue(new InstantCommand(() -> g.ROBOT.Shooter.setShooterFeedShort()));
    g.OI.DRIVER_SHOOTER_OFF.onTrue(new InstantCommand(() -> g.ROBOT.Shooter.setShooterOff()));

    g.OI.DRIVER_SHOOTER_FLIPPER_SHOOT.onTrue(new InstantCommand(() -> g.ROBOT.Shooter.setFlipperExtended()));
    g.OI.DRIVER_SHOOTER_FLIPPER_BACK.onTrue(new InstantCommand(() -> g.ROBOT.Shooter.setFlippersRetracted()));


  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
  /* -------- Old RobotContainer Stuff (End)------------- */
}
