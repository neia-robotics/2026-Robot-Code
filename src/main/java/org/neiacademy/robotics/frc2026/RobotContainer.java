// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.neiacademy.robotics.frc2026;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.neiacademy.robotics.frc2026.Constants.*;
import org.neiacademy.robotics.frc2026.commands.DriveCommands;
import org.neiacademy.robotics.frc2026.commands.ManualPivotBumperCommand;
import org.neiacademy.robotics.frc2026.generated.TunerConstants;
import org.neiacademy.robotics.frc2026.subsystems.Superstructure;
import org.neiacademy.robotics.frc2026.subsystems.drive.Drive;
import org.neiacademy.robotics.frc2026.subsystems.drive.GyroIO;
import org.neiacademy.robotics.frc2026.subsystems.drive.GyroIOPigeon2;
import org.neiacademy.robotics.frc2026.subsystems.drive.ModuleIO;
import org.neiacademy.robotics.frc2026.subsystems.drive.ModuleIOSim;
import org.neiacademy.robotics.frc2026.subsystems.drive.ModuleIOTalonFX;
import org.neiacademy.robotics.frc2026.subsystems.hood.Hood;
import org.neiacademy.robotics.frc2026.subsystems.hood.HoodIO;
import org.neiacademy.robotics.frc2026.subsystems.hood.HoodIOLinearActuator;
import org.neiacademy.robotics.frc2026.subsystems.intakedeploy.IntakeDeploy;
import org.neiacademy.robotics.frc2026.subsystems.intakedeploy.IntakeDeployIO;
import org.neiacademy.robotics.frc2026.subsystems.intakedeploy.IntakeDeployIOTalonFX;
import org.neiacademy.robotics.frc2026.subsystems.intakeroller.IntakeRoller;
import org.neiacademy.robotics.frc2026.subsystems.intakeroller.IntakeRollerIO;
import org.neiacademy.robotics.frc2026.subsystems.intakeroller.IntakeRollerIOTalonFX;
import org.neiacademy.robotics.frc2026.subsystems.loader.Loader;
import org.neiacademy.robotics.frc2026.subsystems.loader.LoaderIO;
import org.neiacademy.robotics.frc2026.subsystems.loader.LoaderIOTalonFX;
import org.neiacademy.robotics.frc2026.subsystems.shooter.Shooter;
import org.neiacademy.robotics.frc2026.subsystems.shooter.ShooterIO;
import org.neiacademy.robotics.frc2026.subsystems.shooter.ShooterIOTalonFX;
import org.neiacademy.robotics.frc2026.subsystems.spindexer.Spindexer;
import org.neiacademy.robotics.frc2026.subsystems.spindexer.SpindexerIO;
import org.neiacademy.robotics.frc2026.subsystems.spindexer.SpindexerIOTalonFX;
import org.neiacademy.robotics.frc2026.subsystems.vision.Vision;
import org.neiacademy.robotics.frc2026.subsystems.vision.VisionConstants;
import org.neiacademy.robotics.frc2026.subsystems.vision.VisionIO;
import org.neiacademy.robotics.frc2026.subsystems.vision.VisionIOPhotonVision;
import org.neiacademy.robotics.frc2026.subsystems.vision.VisionIOPhotonVisionSim;
import org.neiacademy.robotics.frc2026.subsystems.vision.VisionIOTusk;
import org.neiacademy.robotics.frc2026.util.AllianceFlipUtil;
import org.neiacademy.robotics.frc2026.util.AutoMirroringUtil;
import org.neiacademy.robotics.frc2026.util.ControllerAlertRumble;
import strix.tusk.TuskSource;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Spindexer spindexer;
  private final IntakeDeploy intakeDeploy;
  private final IntakeRoller intakeRoller;
  private final Loader loader;
  private final Shooter leftShooter;
  private final Shooter rightShooter;
  private final Hood hood;
  private final Superstructure superstructure;

  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.kWarning);
  private final Alert operatorDisconnected =
      new Alert("Operator controller disconnected (port 1).", AlertType.kWarning);
  private final Alert noAutoAlert = new Alert("Select an auto routine!!!", AlertType.kError);

  private Command noAuto = Commands.none();

  private Trigger inAllianceZone;

  // Controller
  private final CommandXboxController driverCon = new CommandXboxController(0);
  private final CommandXboxController operatorCon = new CommandXboxController(1);
  private final ControllerAlertRumble controllerAlertRumble =
      new ControllerAlertRumble(driverCon, operatorCon);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    RobotController.setBrownoutVoltage(6.0);

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                VisionConstants.useTusk
                    ? new VisionIOTusk(new TuskSource(), drive::samplePoseAt, drive::getPose)
                    : new VisionIOPhotonVision(
                        VisionConstants.camera0Name, VisionConstants.robotToCamera0));

        spindexer = new Spindexer(new SpindexerIOTalonFX());
        intakeDeploy = new IntakeDeploy(new IntakeDeployIOTalonFX());
        intakeRoller = new IntakeRoller(new IntakeRollerIOTalonFX());
        loader = new Loader(new LoaderIOTalonFX());
        leftShooter =
            new Shooter(
                new ShooterIOTalonFX(
                    true,
                    Constants.Shooter.LEFT_SHOOTER_LEADER_ID,
                    Constants.Shooter.LEFT_SHOOTER_FOLLOWER_ID,
                    true),
                true);
        rightShooter =
            new Shooter(
                new ShooterIOTalonFX(
                    false,
                    Constants.Shooter.RIGHT_SHOOTER_LEADER_ID,
                    Constants.Shooter.RIGHT_SHOOTER_FOLLOWER_ID,
                    false),
                false);

        hood = new Hood(new HoodIOLinearActuator());

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        // With useTusk, vision comes from whatever publishes /Vision/observations to the sim's NT
        // server (a real Owl or a fake one), same code path as the real robot.
        vision =
            new Vision(
                drive::addVisionMeasurement,
                VisionConstants.useTusk
                    ? new VisionIOTusk(new TuskSource(), drive::samplePoseAt, drive::getPose)
                    : new VisionIOPhotonVisionSim(
                        VisionConstants.camera0Name,
                        VisionConstants.robotToCamera0,
                        drive::getPose));

        spindexer = new Spindexer(new SpindexerIO() {});
        intakeDeploy = new IntakeDeploy(new IntakeDeployIO() {});
        intakeRoller = new IntakeRoller(new IntakeRollerIO() {});
        loader = new Loader(new LoaderIO() {});
        leftShooter = new Shooter(new ShooterIO() {}, true);
        rightShooter = new Shooter(new ShooterIO() {}, false);
        hood = new Hood(new HoodIO() {});

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
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});
        spindexer = new Spindexer(new SpindexerIO() {});
        intakeDeploy = new IntakeDeploy(new IntakeDeployIO() {});
        intakeRoller = new IntakeRoller(new IntakeRollerIO() {});
        loader = new Loader(new LoaderIO() {});
        leftShooter = new Shooter(new ShooterIO() {}, true);
        rightShooter = new Shooter(new ShooterIO() {}, false);
        hood = new Hood(new HoodIO() {});

        break;
    }

    superstructure =
        new Superstructure(
            drive, spindexer, intakeDeploy, intakeRoller, loader, leftShooter, rightShooter, hood);

    inAllianceZone =
        new Trigger(
            () -> {
              Pose2d robotPose = AllianceFlipUtil.apply(drive.getPose());
              return (robotPose.getX() <= FieldConstants.LinesVertical.allianceZone + 0.40);
            });

    // manual shoot (close hub, center only auto)
    NamedCommands.registerCommand("shoot", superstructure.shootCommand().withTimeout(5));
    NamedCommands.registerCommand(
        "spinShooterFlywheels",
        Commands.parallel(
                leftShooter.runTrackedVelocityCommand(Presets.Shooter.CLOSE_HUB_SPEED),
                rightShooter.runTrackedVelocityCommand(Presets.Shooter.CLOSE_HUB_SPEED))
            .withTimeout(5));

    NamedCommands.registerCommand("autoShoot", superstructure.autoShoot());
    NamedCommands.registerCommand(
        "intakeRoller", intakeRoller.runVoltageCommand(Presets.Intake.INTAKE_VOLTS).withTimeout(5));
    NamedCommands.registerCommand("intakeDeploy", superstructure.deployIntake().withTimeout(0.75));
    NamedCommands.registerCommand(
        "intakeRetract", superstructure.retractIntake().withTimeout(0.75));
    NamedCommands.registerCommand(
        "runSpindexer", spindexer.runVoltageCommand(Presets.Spindexer.FEED_VOLTS));
    NamedCommands.registerCommand(
        "hubAimAndShoot",
        Commands.run(
                () ->
                    inAllianceZone
                        .whileTrue(
                            superstructure.hubAimCommand(
                                () -> -driverCon.getLeftY(), () -> -driverCon.getLeftX()))
                        .and(leftShooter::atSetpoint)
                        .and(rightShooter::atSetpoint)
                        .and(DriveCommands::atAngleSetpoint)
                        .whileTrue(superstructure.shootCommand())
                        .onFalse(superstructure.endShootCommand()))
            .withTimeout(4));

    NamedCommands.registerCommand(
        "closeHubShoot",
        Commands.run(
            () -> {
              spindexer.runVoltageCommand(Presets.Spindexer.FEED_VOLTS);
              loader.runVoltageCommand(Presets.Loader.FEED_VOLTS);
              leftShooter.runVelocityCommand(Presets.Shooter.CLOSE_HUB_SPEED);
              rightShooter.runVelocityCommand(Presets.Shooter.CLOSE_HUB_SPEED);
            }));

    NamedCommands.registerCommand(
        "toggleIntakeDeploy", new RepeatCommand(superstructure.toggleIntake()).withTimeout(6));

    NamedCommands.registerCommand(
        "xLock", new ParallelCommandGroup(Commands.run(drive::stopWithX, drive)));

    NamedCommands.registerCommand(
        "forceShoot",
        new ParallelCommandGroup(
                spindexer.runVoltageCommand(Presets.Spindexer.FEED_VOLTS),
                loader.runVoltageCommand(Presets.Loader.FEED_VOLTS))
            .withTimeout(0.5));

    NamedCommands.registerCommand(
        "unjam",
        new ParallelCommandGroup(
                intakeRoller.runVoltageCommand(Presets.Intake.EXHAUST_VOLTS),
                loader.runVoltageCommand(Presets.Loader.EXHAUST_VOLTS),
                spindexer.runVoltageCommand(Presets.Spindexer.EXHAUST_VOLTS))
            .withTimeout(0.25));

    NamedCommands.registerCommand(
        "autoEndShootCommand", new ParallelCommandGroup(superstructure.autoEndShootCommand()));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    autoChooser.addDefaultOption("No Auto!", noAuto);

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
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
    // auto mirrors ANY pathplanner auto that begins with right/left (case insensitive)
    AutoMirroringUtil.addMirroredAutos(autoChooser);

    SmartDashboard.putData(
        "RunEverythingForTuning",
        new ParallelCommandGroup(
            loader.runVoltageCommand(Presets.Loader.TUNING_VOLTS),
            spindexer.runVoltageCommand(Presets.Spindexer.TUNING_VOLTS),
            intakeRoller.runVoltageCommand(Presets.Intake.TUNING_VOLTS),
            intakeDeploy.runTrackedPositionCommand(
                () -> Units.degreesToRotations(Presets.Intake.TUNING_ANGLE_DEG.getAsDouble())),
            leftShooter.runTrackedVelocityCommand(Presets.Shooter.TUNING_SPEED),
            rightShooter.runTrackedVelocityCommand(Presets.Shooter.TUNING_SPEED)));

    SmartDashboard.putData("IncreaseSpeedSetpoint", superstructure.increaseDriveSpeedCommand());

    SmartDashboard.putData("DecreaseSpeedSetpoint", superstructure.decreaseDriveSpeedCommand());

    SmartDashboard.putBoolean("ManualMode", Constants.manualMode);
    SmartDashboard.putBoolean("TuningMode", Constants.tuningMode);
    SmartDashboard.putBoolean("ConstantFlywheelsMode", Constants.constantFlywheelsMode);
    SmartDashboard.putBoolean("FixedShooterMode", Constants.fixedShooterMode);

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverCon.getLeftY() * superstructure.getCurrentDriveSpeed().translationScale,
            () -> -driverCon.getLeftX() * superstructure.getCurrentDriveSpeed().translationScale,
            () -> -driverCon.getRightX() * superstructure.getCurrentDriveSpeed().rotationScale));

    // Switch to X pattern when X button is pressed
    driverCon
        .x()
        .and(driverCon.rightTrigger().negate())
        .whileTrue(Commands.run(drive::stopWithX, drive));

    // force shoot even if the tolerances aren't being met
    driverCon
        .y()
        .whileTrue(
            new ParallelCommandGroup(
                spindexer.runVoltageCommand(Presets.Spindexer.FEED_VOLTS),
                loader.runVoltageCommand(Presets.Loader.FEED_VOLTS)));

    driverCon
        .b()
        .whileTrue(
            new ParallelCommandGroup(
                intakeRoller.runVoltageCommand(Presets.Intake.EXHAUST_VOLTS),
                loader.runVoltageCommand(Presets.Loader.EXHAUST_VOLTS),
                spindexer.runVoltageCommand(Presets.Spindexer.EXHAUST_VOLTS)));

    // Lock to a parallel angle to shove balls
    driverCon
        .a()
        .and(driverCon.rightTrigger().negate())
        .and(driverCon.x().negate())
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverCon.getLeftY(),
                () -> -driverCon.getLeftX(),
                () -> DriveCommands.closestNormalAngle(drive.getPose()),
                () -> new Rotation2d(0, 0)));

    driverCon
        .rightTrigger()
        .and(inAllianceZone)
        .whileTrue(
            superstructure.hubAimCommand(
                () -> -driverCon.getLeftY(),
                () -> -driverCon.getLeftX(),
                () -> -operatorCon.getRightX(),
                driverCon.x()))
        .and(leftShooter::atSetpoint)
        .and(rightShooter::atSetpoint)
        .and(DriveCommands::atAngleSetpoint)
        .and(hood::atSetpoint)
        .debounce(0.04)
        .whileTrue(superstructure.shootCommand())
        .onFalse(superstructure.endShootCommand());
    // shuttle
    driverCon
        .rightTrigger()
        .and(inAllianceZone.negate())
        .whileTrue(
            superstructure.shuttleAimCommand(
                () -> -driverCon.getLeftY(),
                () -> -driverCon.getLeftX(),
                () -> -operatorCon.getRightX(),
                driverCon.x()))
        .and(leftShooter::atSetpoint)
        .and(rightShooter::atSetpoint)
        .and(DriveCommands::atAngleSetpoint)
        .and(hood::atSetpoint)
        .debounce(0.04)
        .whileTrue(superstructure.shootCommand())
        .onFalse(superstructure.endShootCommand());

    // fall back
    driverCon
        .rightBumper()
        .whileTrue(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    leftShooter.runVelocityCommand(Presets.Shooter.CLOSE_HUB_SPEED),
                    rightShooter.runVelocityCommand(Presets.Shooter.CLOSE_HUB_SPEED)),
                new ParallelCommandGroup(
                    spindexer.runVoltageCommand(Presets.Spindexer.FEED_VOLTS),
                    loader.runVoltageCommand(Presets.Loader.FEED_VOLTS),
                    leftShooter.runVelocityCommand(Presets.Shooter.CLOSE_HUB_SPEED),
                    rightShooter.runVelocityCommand(Presets.Shooter.CLOSE_HUB_SPEED))))
        .onFalse(superstructure.endShootCommand());

    // force shoot fall back even if flywheels aren't fully spun up
    driverCon
        .povUp()
        .whileTrue(
            new ParallelCommandGroup(
                leftShooter.runVelocityCommand(Presets.Shooter.CLOSE_HUB_SPEED),
                rightShooter.runVelocityCommand(Presets.Shooter.CLOSE_HUB_SPEED),
                spindexer.runVoltageCommand(Presets.Spindexer.FEED_VOLTS),
                loader.runVoltageCommand(Presets.Loader.FEED_VOLTS)));

    // Reset gyro to 0 when povdown button is pressed
    driverCon
        .povDown()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
    driverCon
        .start()
        .whileTrue(
            new ParallelCommandGroup(
                leftShooter.runTrackedVelocityCommand(Presets.Shooter.HALF_SHUTTLE_SPEED),
                rightShooter.runTrackedVelocityCommand(Presets.Shooter.HALF_SHUTTLE_SPEED)))
        .and(leftShooter::atSetpoint)
        .and(rightShooter::atSetpoint)
        .whileTrue(superstructure.shootCommand())
        .onFalse(superstructure.endShootCommand());

    driverCon
        .back()
        .whileTrue(
            new ParallelCommandGroup(
                leftShooter.runTrackedVelocityCommand(Presets.Shooter.FULL_SHUTTLE_SPEED),
                rightShooter.runTrackedVelocityCommand(Presets.Shooter.FULL_SHUTTLE_SPEED)))
        .and(leftShooter::atSetpoint)
        .and(rightShooter::atSetpoint)
        .whileTrue(superstructure.shootCommand())
        .onFalse(superstructure.endShootCommand());

    // Tune shot
    driverCon
        .povLeft()
        .whileTrue(
            new ParallelCommandGroup(
                leftShooter.runTrackedVelocityCommand(Presets.Shooter.TUNING_SPEED),
                rightShooter.runTrackedVelocityCommand(Presets.Shooter.TUNING_SPEED),
                hood.runTrackedPositionCommand(Presets.Hood.TUNING_POSITION),
                spindexer.runVoltageCommand(Presets.Spindexer.FEED_VOLTS),
                loader.runVoltageCommand(Presets.Loader.FEED_VOLTS)));

    driverCon.leftTrigger().onTrue(superstructure.deployIntake());
    driverCon.leftTrigger().whileTrue(intakeRoller.runVoltageCommand(Presets.Intake.INTAKE_VOLTS));

    driverCon.leftBumper().onTrue(superstructure.retractIntake());

    operatorCon.a().whileTrue(intakeRoller.runVoltageCommand(Presets.Intake.EXHAUST_VOLTS));
    operatorCon.b().whileTrue(superstructure.holdBallHandlingStoppedCommand());
    operatorCon.x().debounce(0.5).onTrue(superstructure.resetShooterSpeedAdjustmentCommand());
    operatorCon
        .y()
        .whileTrue(
            new ParallelCommandGroup(
                spindexer.runVoltageCommand(Presets.Spindexer.EXHAUST_VOLTS),
                loader.runVoltageCommand(Presets.Loader.EXHAUST_VOLTS)));

    // A bumper press used for manual pivot must never also change shooter speed.
    operatorCon
        .leftBumper()
        .whileTrue(
            new ManualPivotBumperCommand(
                operatorCon.leftBumper(),
                operatorCon.leftStick(),
                () -> {
                  if (inAllianceZone.getAsBoolean()) {
                    CommandScheduler.getInstance()
                        .schedule(superstructure.fudgeShooterSpeedShoot(-1));
                  } else {
                    CommandScheduler.getInstance()
                        .schedule(superstructure.fudgeShooterSpeedShuttle(-1));
                  }
                }));
    Trigger manualPivot = operatorCon.leftBumper().and(operatorCon.leftStick());
    // Continue commanding the held target so the software motion profile can finish.
    manualPivot.onTrue(
        intakeDeploy.manualPositionCommand(
            () -> manualPivot.getAsBoolean() ? operatorCon.getLeftY() : 0.0));

    // fall back
    operatorCon
        .povUp()
        .whileTrue(
            new ParallelCommandGroup(
                leftShooter.runTrackedVelocityCommand(Presets.Shooter.CLOSE_HUB_SPEED),
                rightShooter.runTrackedVelocityCommand(Presets.Shooter.CLOSE_HUB_SPEED)))
        .and(leftShooter::atSetpoint)
        .and(rightShooter::atSetpoint)
        .whileTrue(superstructure.shootCommand())
        .onFalse(superstructure.endShootCommand());

    operatorCon
        .povRight()
        .whileTrue(
            new ParallelCommandGroup(
                leftShooter.runTrackedVelocityCommand(Presets.Shooter.TRENCH_SPEED),
                rightShooter.runTrackedVelocityCommand(Presets.Shooter.TRENCH_SPEED)))
        .and(leftShooter::atSetpoint)
        .and(rightShooter::atSetpoint)
        .whileTrue(superstructure.shootCommand())
        .onFalse(superstructure.endShootCommand());

    operatorCon
        .povLeft()
        .whileTrue(
            new ParallelCommandGroup(
                leftShooter.runTrackedVelocityCommand(Presets.Shooter.CORNER_SPEED),
                rightShooter.runTrackedVelocityCommand(Presets.Shooter.CORNER_SPEED)))
        .and(leftShooter::atSetpoint)
        .and(rightShooter::atSetpoint)
        .whileTrue(superstructure.shootCommand())
        .onFalse(superstructure.endShootCommand());

    operatorCon
        .povDown()
        .whileTrue(
            new ParallelCommandGroup(
                leftShooter.runTrackedVelocityCommand(Presets.Shooter.TOWER_SPEED),
                rightShooter.runTrackedVelocityCommand(Presets.Shooter.TOWER_SPEED)))
        .and(leftShooter::atSetpoint)
        .and(rightShooter::atSetpoint)
        .whileTrue(superstructure.shootCommand())
        .onFalse(superstructure.endShootCommand());

    // force shoot even if the tolerances aren't being met
    operatorCon.rightBumper().and(inAllianceZone).onTrue(superstructure.fudgeShooterSpeedShoot(1));

    operatorCon
        .rightBumper()
        .and(inAllianceZone.negate())
        .onTrue(superstructure.fudgeShooterSpeedShuttle(1));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void updateAlerts() {
    // Controller disconnected alerts
    driverDisconnected.set(!DriverStation.isJoystickConnected(driverCon.getHID().getPort()));
    operatorDisconnected.set(!DriverStation.isJoystickConnected(operatorCon.getHID().getPort()));

    // Auto alert
    noAutoAlert.set(
        DriverStation.isAutonomous() && !DriverStation.isEnabled() && autoChooser.get() == noAuto);

    controllerAlertRumble.periodic();
  }
}
