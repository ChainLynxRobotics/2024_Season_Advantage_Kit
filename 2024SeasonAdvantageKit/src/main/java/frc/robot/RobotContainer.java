// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Bindings;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSparkMax;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOSparkMax;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Climber climber;
  private final Indexer indexer;
  private final Intake intake;
  private final Shooter shooter;
  private final Superstructure superstructure;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final Joystick opStick = new Joystick(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(false),
                new ModuleIOSparkMax(0),
                new ModuleIOSparkMax(1),
                new ModuleIOSparkMax(2),
                new ModuleIOSparkMax(3));
        climber =
            new Climber(
                new ClimberIOSparkMax(Ports.kClimberLeaderID, Ports.kClimberFollowerID));
        indexer = 
            new Indexer(
                new IndexerIOSparkMax(Ports.kIndexerMotorId, Ports.kLinebreakSensorId));
        intake = 
            new Intake(
                new IntakeIOSparkMax(Ports.kIntakeMotorID));
        shooter =
            new Shooter(
                new ShooterIOSparkMax(Ports.kTopFlywheelMotorId, Ports.kBottomFlywheelMotorId, Ports.kPivotMotorLeaderId, Ports.kPivotMotorFollowerId));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        System.out.println("in simulation mode");
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        climber = 
            new Climber(new ClimberIO() {});
        indexer = 
            new Indexer(new IndexerIOSim());
        intake = 
            new Intake(new IntakeIOSim());
        shooter =
            new Shooter(new ShooterIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        climber =
            new Climber(
                new ClimberIO() {});
        indexer = 
            new Indexer(
                new IndexerIO() {});
        intake = 
            new Intake(
                new IntakeIO() {});
        shooter = 
            new Shooter(
                new ShooterIO() {});
        break;
    }
    superstructure = new Superstructure(climber, indexer, intake, shooter);
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());


    autoChooser.addDefaultOption("default drive", AutoBuilder.buildAuto("Example Auto"));
    // Set up SysId routines
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    //drive.setDefaultCommand(DriveCommands.rotateCommand(drive, () -> -controller.getLeftX()));
    drive.setDefaultCommand(DriveCommands.joystickDrive(
      drive,
      () -> MathUtil.applyDeadband(-controller.getLeftY(), DriveConstants.driveDeadband), 
      () -> MathUtil.applyDeadband(-controller.getLeftX(), DriveConstants.driveDeadband), 
      () -> MathUtil.applyDeadband(-controller.getRightX(), DriveConstants.driveDeadband)));

    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
    
    new Trigger(() -> opStick.getRawButton(Bindings.kIntakeNoteButtonID))
        .whileTrue(superstructure.intake(false))
        .onFalse(superstructure.stopIntake());
    new Trigger(() -> opStick.getRawButton(Bindings.kReverseIntakeButtonID))
        .whileTrue(superstructure.intake(true))
        .onFalse(superstructure.stopIntake());

    new Trigger(() -> opStick.getRawButton(Bindings.kShoot))
        .whileTrue(superstructure.indexPiece(false))
        .onFalse(superstructure.stopIndexer());
    new Trigger(() -> opStick.getRawButton(Bindings.kShootReverse))
        .whileTrue(superstructure.indexPiece(true))
        .onFalse(superstructure.stopIndexer());

    new Trigger(() -> opStick.getRawButton(Bindings.kFlywheelSpeaker))
        .whileTrue(superstructure.runFlywheels(ShooterConstants.kFlywheelDefaultRPM, false));

    new Trigger(() -> opStick.getRawButton(Bindings.kBothClimbersUp))
        .whileTrue(superstructure.climb(false, false, false))
        .onFalse(superstructure.stopClimb());
    new Trigger(() -> opStick.getRawButton(Bindings.kBothClimbersDown))
        .whileTrue(superstructure.climb(true, false, false))
        .onFalse(superstructure.stopClimb());
    new Trigger(() -> opStick.getRawButton(Bindings.kLeftClimberUp))
        .whileTrue(superstructure.climb(false, true, false))
        .onFalse(superstructure.stopClimb());
    new Trigger(() -> opStick.getRawButton(Bindings.kLeftClimberDown))
        .whileTrue(superstructure.climb(true, true, false))
        .onFalse(superstructure.stopClimb());
    new Trigger(() -> opStick.getRawButton(Bindings.kRightClimberUp))
        .whileTrue(superstructure.climb(false, false, true))
        .onFalse(superstructure.stopClimb());
    new Trigger(() -> opStick.getRawButton(Bindings.kRightClimberDown))
        .whileTrue(superstructure.climb(true, false, true))
        .onFalse(superstructure.stopClimb());

    new Trigger(() -> opStick.getRawButton(Bindings.kStowShooter))
        .whileTrue(superstructure.movePivot(MutableMeasure.ofBaseUnits(0.0, Units.Rotations)));
    new Trigger(() -> opStick.getRawButton(Bindings.kAimSpeaker))
        .whileTrue(superstructure.movePivot(PivotConstants.kSpeakerAngle));

    new Trigger(() -> opStick.getRawButton(Bindings.kScoreSpeaker))
        .onTrue(superstructure.scoreSpeaker(PivotConstants.kSpeakerAngle, ShooterConstants.kFlywheelDefaultRPM));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
