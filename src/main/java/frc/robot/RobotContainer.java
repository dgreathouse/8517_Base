package frc.robot;

import java.util.Iterator;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.autoCommandGroups.AutoDoNothing;
import frc.robot.commands.DrivetrainDefaultCommand;
import frc.robot.lib.DriveMode;
import frc.robot.lib.IUpdateDashboard;
import frc.robot.lib.g;
import frc.robot.subsystems.Drivetrain;


/**
 */
public class RobotContainer {
  public static final Drivetrain m_drivetrain = new Drivetrain();
  private final DrivetrainDefaultCommand m_drivetrainDefaultCommand = new DrivetrainDefaultCommand(m_drivetrain);
  private SendableChooser<Command> autoChooser = new SendableChooser<>();
  public static ShuffleboardTab matchTab = Shuffleboard.getTab("Match");
  public static PowerDistribution m_pd = new PowerDistribution();
  private Notifier m_telemetry;
  private Iterator<IUpdateDashboard> m_iteratorUpdate = g.DASHBOARD.updates.iterator();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_drivetrain.setDefaultCommand(m_drivetrainDefaultCommand);
    configureBindings();
    autoChooser.setDefaultOption("Do Nothing", new AutoDoNothing());

    m_telemetry = new Notifier(this::updateDashboard);
    m_telemetry.startPeriodic(0.1);

    configShuffleBoard();
    
  }
  private void updateDashboard(){
    while (m_iteratorUpdate.hasNext()) {
      m_iteratorUpdate.next().updateDashboard();
    }
  }
  /**
   */
  private void configureBindings() {
    g.OI.DRIVER_RESET_YAW.onTrue(new InstantCommand(m_drivetrain::resetYaw, m_drivetrain));
    
    g.OI.DRIVER_DRIVE_MODE_ANGLEFIELDCENTRIC.onTrue(new InstantCommand(() -> m_drivetrain.setDriveMode(DriveMode.ANGLE_FIELD_CENTRIC), m_drivetrain));
    g.OI.DRIVER_DRIVE_MODE_FIELDCENTRIC.onTrue(new InstantCommand(() -> m_drivetrain.setDriveMode(DriveMode.FIELD_CENTRIC), m_drivetrain));
    g.OI.DRIVER_DRIVE_MODE_ROBOTCENTRIC.onTrue(new InstantCommand(() -> m_drivetrain.setDriveMode(DriveMode.ROBOT_CENTRIC), m_drivetrain));
    g.OI.DRIVER_DRIVE_MODE_ROTATEFIELDCENTRIC.onTrue(new InstantCommand(() -> m_drivetrain.setDriveMode(DriveMode.ROTATE_FIELD_CENTRIC), m_drivetrain));

    g.OI.DRIVER_DRIVE_MODE_SPEED_HI.onTrue(new InstantCommand(() -> m_drivetrain.setDriveSpeedMultiplier(1.0)));
    g.OI.DRIVER_DRIVE_MODE_SPEED_HI.onTrue(new InstantCommand(() -> m_drivetrain.setDriveSpeedMultiplier(0.5)));

  }
  private void configShuffleBoard(){
    Shuffleboard.selectTab("Match");
    matchTab.add("Autonomous Play",autoChooser).withPosition(5,0).withSize(3, 2);
    matchTab.add("Drive State",g.DRIVETRAIN.driveMode.toString()).withPosition(14, 0).withSize(3,2);
    matchTab.add("Battery Volts", m_pd.getVoltage()).withPosition(5, 2).withSize(12,3).withWidget(BuiltInWidgets.kGraph);
  }
  /**
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
