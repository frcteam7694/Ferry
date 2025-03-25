package frc.robot;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;
import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.SwerveModuleTrajectoryState;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.*;
import frc.robot.commands.elevator.ElevateCommand;
import frc.robot.commands.elevator.ElevatorCommands;
import frc.robot.commands.elevator.ManuallyElevateCommand;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOSpark;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.dropper.Dropper;
import frc.robot.subsystems.dropper.DropperIOSim;
import frc.robot.subsystems.dropper.DropperIOSpark;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOSpark;
import frc.robot.subsystems.forklift.Forklift;
import frc.robot.subsystems.forklift.ForkliftIOSim;
import frc.robot.subsystems.forklift.ForkliftIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import java.io.IOException;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;
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
  private final Vision vision;
  private final Elevator elevator;
  private final Dropper dropper;
  private final Forklift forklift;
  private final Climber climber;

  // Controller
  public static final CommandXboxController driverController = new CommandXboxController(0);
  public static final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        vision =
            Constants.currentRobot == Constants.Robots.Ferry
                ? new Vision(
                    drive::getRotation,
                    drive::addVisionMeasurement,
                    new VisionIOPhotonVision(pv0c0, pv0c0Pos),
                    new VisionIOPhotonVision(pv0c1, pv0c1Pos))
                : new Vision(drive::getRotation, drive::addVisionMeasurement);
        elevator =
            Constants.currentRobot == Constants.Robots.Ferry
                ? new Elevator(new ElevatorIOSpark())
                : new Elevator(new ElevatorIOSim());
        dropper =
            Constants.currentRobot == Constants.Robots.Ferry
                ? new Dropper(new DropperIOSpark())
                : new Dropper(new DropperIOSim());
        forklift =
            Constants.currentRobot == Constants.Robots.Ferry
                ? new Forklift(new ForkliftIOSpark())
                : new Forklift(new ForkliftIOSim());
        climber = new Climber(new ClimberIOSpark());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        vision = new Vision(drive::getRotation, drive::addVisionMeasurement);
        elevator = new Elevator(new ElevatorIOSim());
        dropper = new Dropper(new DropperIOSim());
        forklift = new Forklift(new ForkliftIOSim());
        climber = new Climber(new ClimberIOSim());
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
        vision = new Vision(drive::getRotation, drive::addVisionMeasurement);
        elevator = new Elevator(new ElevatorIOSim());
        dropper = new Dropper(new DropperIOSim());
        forklift = new Forklift(new ForkliftIOSim());
        climber = new Climber(new ClimberIOSim());
        break;
    }

    // Named commands
    NamedCommands.registerCommand("DropTrough", AutoCommands.dropL1(elevator, dropper));
    NamedCommands.registerCommand("DropLow", AutoCommands.dropL2(elevator, dropper));
    NamedCommands.registerCommand("DropMid", AutoCommands.dropL3(elevator, dropper));
    NamedCommands.registerCommand("DropHigh", AutoCommands.dropL4(elevator, dropper));
    NamedCommands.registerCommand("Align", DriveCommands.home(drive));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

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

    // Configure the button bindings
    configureButtonBindings();
    elevator.zeroEncoder();
    forklift.zeroEncoder();

    if (Robot.isReal()) {
      var camera1 = CameraServer.startAutomaticCapture();
      camera1.setResolution(212, 160);
      camera1.setFPS(25);
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            driverController.leftTrigger(),
            driverController.rightTrigger(),
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    // Lock to 0° when A button is held
    driverController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> Rotation2d.kZero));
    driverController
        .y()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> Rotation2d.k180deg));

    // Switch to X pattern when X button is pressed
    driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    driverController.povLeft().onTrue(DriveCommands.home(drive));
    driverController
        .povRight()
        .onTrue(DriveCommands.homeWhileHolding(drive, driverController.povRight()));

    // Reset gyro to 0° when B button is pressed
    driverController.b().onTrue(Commands.runOnce(drive::resetGyro, drive).ignoringDisable(true));

    // Elevator
    elevator.setDefaultCommand(ElevatorCommands.drive(elevator, operatorController));

    operatorController
        .back()
        .onTrue(
            new ManuallyElevateCommand(
                elevator,
                () ->
                    (operatorController.getLeftTriggerAxis()
                        - operatorController.getRightTriggerAxis()),
                operatorController.back()));

    operatorController.a().onTrue(ElevateCommand.create(elevator, level0));
    operatorController.leftBumper().onTrue(ElevateCommand.create(elevator, level1));
    operatorController.x().onTrue(ElevateCommand.create(elevator, level2));
    operatorController.b().onTrue(ElevateCommand.create(elevator, level3));
    operatorController.y().onTrue(ElevateCommand.create(elevator, level4)); // more but whatever
    operatorController.start().onTrue(ElevatorCommands.resetEncoder(elevator));

    // Dropper
    operatorController.rightBumper().onTrue(DropperCommands.drop(dropper));
    operatorController.leftStick().onTrue(DropperCommands.halfDrop(dropper));

    // Forklift
    operatorController.povUp().onTrue(ForkliftCommands.driveFor(forklift, -1, .8));
    operatorController
        .povLeft()
        .onTrue(ForkliftCommands.drive(forklift, -.75))
        .onFalse(ForkliftCommands.drive(forklift, 0));
    operatorController
        .povRight()
        .onTrue(ForkliftCommands.drive(forklift, -.75))
        .onFalse(ForkliftCommands.drive(forklift, 0));
    operatorController.povDown().onTrue(ForkliftCommands.driveFor(forklift, 1, .8));

    // Climber
    climber.setDefaultCommand(
        ClimberCommands.drive(climber, operatorController.rightStick(), operatorController));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void checkModules()
      throws IOException, ParseException, NoSuchFieldException, IllegalAccessException {
    var auto = getAutonomousCommand();
    if (auto == null) return;
    if (!(auto instanceof PathPlannerAuto)) return;
    var ppAuto =
        PathPlannerAuto.getPathGroupFromAutoFile(autoChooser.getSendableChooser().getSelected());
    if (ppAuto.isEmpty()) return;
    var path = ppAuto.get(0);
    var traj = path.getIdealTrajectory(DriveConstants.ppConfig);
    if (traj.isEmpty()) return;
    this.checkState(traj.get());
  }

  private void checkState(PathPlannerTrajectory traj)
      throws NoSuchFieldException, IllegalAccessException {
    var initialState = traj.getInitialState();
    var clazz = initialState.getClass();
    var field = clazz.getDeclaredField("moduleStates");
    field.setAccessible(true);
    var modules = (SwerveModuleTrajectoryState[]) field.get(initialState);
    var wpiModules =
        new SwerveModuleState[] {
          new SwerveModuleState(modules[0].speedMetersPerSecond, modules[0].angle),
          new SwerveModuleState(modules[1].speedMetersPerSecond, modules[1].angle),
          new SwerveModuleState(modules[2].speedMetersPerSecond, modules[2].angle),
          new SwerveModuleState(modules[3].speedMetersPerSecond, modules[3].angle)
        };
    Logger.recordOutput("StartingAutoWheels", wpiModules);
  }
}
