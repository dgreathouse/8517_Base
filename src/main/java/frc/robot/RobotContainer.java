package frc.robot;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.autoCommandGroups.AutoDoNothing;
import frc.robot.commands.DrivetrainDefaultCommand;
import frc.robot.lib.DriveMode;
import frc.robot.lib.g;

/**
 */
public class RobotContainer {
  
  private final DrivetrainDefaultCommand m_drivetrainDefaultCommand = new DrivetrainDefaultCommand(g.ROBOT.Drive);
  private SendableChooser<Command> autoChooser = new SendableChooser<>();
  private Notifier m_telemetry;
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    g.ROBOT.Drive.setDefaultCommand(m_drivetrainDefaultCommand);
    configureBindings();
    autoChooser.setDefaultOption("Do Nothing", new AutoDoNothing());

    m_telemetry = new Notifier(this::updateDashboard);
    m_telemetry.startPeriodic(0.1);

    configShuffleBoard();
  }
  
  private void updateDashboard(){
    while (g.DASHBOARD.updates.iterator().hasNext()) {
      g.DASHBOARD.updates.iterator().next().updateDashboard();
    }
  }

  private void configureBindings() {
    g.OI.DRIVER_RESET_YAW.onTrue(new InstantCommand(g.ROBOT.Drive::resetYaw, g.ROBOT.Drive));
    
    g.OI.DRIVER_DRIVE_MODE_ANGLEFIELDCENTRIC.onTrue(new InstantCommand(() -> g.ROBOT.Drive.setDriveMode(DriveMode.ANGLE_FIELD_CENTRIC), g.ROBOT.Drive));
    g.OI.DRIVER_DRIVE_MODE_FIELDCENTRIC.onTrue(new InstantCommand(() -> g.ROBOT.Drive.setDriveMode(DriveMode.FIELD_CENTRIC), g.ROBOT.Drive));
    g.OI.DRIVER_DRIVE_MODE_ROBOTCENTRIC.onTrue(new InstantCommand(() -> g.ROBOT.Drive.setDriveMode(DriveMode.ROBOT_CENTRIC), g.ROBOT.Drive));
    g.OI.DRIVER_DRIVE_MODE_ROTATEFIELDCENTRIC.onTrue(new InstantCommand(() -> g.ROBOT.Drive.setDriveMode(DriveMode.ROTATE_FIELD_CENTRIC), g.ROBOT.Drive));

    g.OI.DRIVER_DRIVE_MODE_SPEED_HI.onTrue(new InstantCommand(() -> g.ROBOT.Drive.setDriveSpeedMultiplier(1.0)));
    g.OI.DRIVER_DRIVE_MODE_SPEED_HI.onTrue(new InstantCommand(() -> g.ROBOT.Drive.setDriveSpeedMultiplier(0.5)));

  }

  private void configShuffleBoard(){
    Shuffleboard.selectTab("Match");
    g.DASHBOARD.ShuffleBoardTab_Match.add("Autonomous Play",autoChooser).withPosition(5,0).withSize(3, 2);
    g.DASHBOARD.ShuffleBoardTab_Match.add("Drive State",g.DRIVETRAIN.driveMode.toString()).withPosition(14, 0).withSize(3,2);
    g.DASHBOARD.ShuffleBoardTab_Match.add("Battery Volts", g.ROBOT.Power.getVoltage()).withPosition(5, 2).withSize(12,3).withWidget(BuiltInWidgets.kGraph);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
