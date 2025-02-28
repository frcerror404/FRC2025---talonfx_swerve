// Copyright 2021-2025 FRC 6328
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

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.vision.VisionConstants.*;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Mode;
import frc.robot.commands.ClawAngleAvoidElevator;
import frc.robot.commands.ClawAngleHome;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeAlgae;
import frc.robot.commands.IntakeCoral;
import frc.robot.commands.PositionAlgaeL2;
import frc.robot.commands.PositionAlgaeL3;
import frc.robot.commands.PositionL2;
import frc.robot.commands.PositionL3;
import frc.robot.commands.PositionL4;
import frc.robot.commands.StopIntake;
import frc.robot.commands.StowHome;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.ClawIOSim;
import frc.robot.subsystems.claw.ClawIOTalonFX;
import frc.robot.subsystems.clawAngle.ClawAngle;
import frc.robot.subsystems.clawAngle.ClawAngleIOSim;
import frc.robot.subsystems.clawAngle.ClawAngleIOTalonFX;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.CanDef;
import frc.robot.util.CanDef.CanBus;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.ReefPositionsUtil;
import frc.robot.util.ReefPositionsUtil.*;
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

  private final Elevator elevator;

  private final Climber climber;

  private final Claw claw;

  private final ClawAngle clawAngle;

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  private final AprilTagVision vision;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private RobotState robotState;
  private ReefPositionsUtil reefPositions;

  private boolean m_TeleopInitialized = false;

  final LoggedTunableNumber setClawAngle =
      new LoggedTunableNumber("RobotState/ClawAngle/setClawAngle", 0);
  final LoggedTunableNumber setClimberVolts =
      new LoggedTunableNumber("dashboardKey:RobotState/Climber/setVolts", 0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    CanDef.Builder canivoreCanBuilder = CanDef.builder().bus(CanBus.CANivore);
    CanDef.Builder rioCanBuilder = CanDef.builder().bus(CanBus.Rio);

    switch (
    /*Constants.currentMode*/ Mode.REAL) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new AprilTagVision(
                drive::setPose,
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(camera0Name, robotToCamera0),
                new VisionIOPhotonVision(camera1Name, robotToCamera1));

        elevator =
            new Elevator(
                new ElevatorIOTalonFX(rioCanBuilder.id(51).build(), rioCanBuilder.id(52).build()));

        climber = new Climber(new ClimberIOTalonFX(rioCanBuilder.id(10).build()));

        claw =
            new Claw(new ClawIOTalonFX(rioCanBuilder.id(24).build(), rioCanBuilder.id(25).build()));

        clawAngle =
            new ClawAngle(
                new ClawAngleIOTalonFX(rioCanBuilder.id(20).build(), rioCanBuilder.id(26).build()));

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

        vision =
            new AprilTagVision(
                drive::setPose,
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));

        elevator =
            new Elevator(
                new ElevatorIOSim(
                    4,
                    new ElevatorSim(
                        LinearSystemId.createElevatorSystem(
                            DCMotor.getKrakenX60Foc(2),
                            Pounds.of(45).in(Kilograms),
                            Inches.of(Elevator.SPOOL_RADIUS).in(Meters),
                            Elevator.REDUCTION),
                        DCMotor.getKrakenX60Foc(2),
                        Inches.of(0).in(Meters),
                        Inches.of(32).in(Meters),
                        true,
                        Inches.of(0).in(Meters))));

        climber = new Climber(new ClimberIOSim(0));

        claw = new Claw(new ClawIOSim(0));

        clawAngle = new ClawAngle(new ClawAngleIOSim(0));

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

        // (Use same number of dummy implementations as the real robot)
        vision =
            new AprilTagVision(
                drive::setPose, drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        elevator = null;

        climber = null;

        claw = null;

        clawAngle = null;
    }

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

    reefPositions = ReefPositionsUtil.getInstance();

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));

    // co_controller.rightTrigger().whileTrue(elevator.getNewSetDistanceCommand(set));

    // Switch to X pattern when X button is pressed
    driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // // Auto aim command example
    // @SuppressWarnings("resource")
    // PIDController aimController = new PIDController(0.2, 0.0, 0.0);
    // aimController.enableContinuousInput(-Math.PI, Math.PI);
    // keyboard
    //     .button(1)
    //     .whileTrue(
    //         Commands.startRun(
    //             () -> {
    //               aimController.reset();
    //             },
    //             () -> {
    //               drive.run(0.0, aimController.calculate(vision.getTargetX(0).getRadians()));
    //             },
    //             drive));

    // Reset gyro to 0° when start button is pressed
    driver
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    operator
        .a()
        .whileTrue(new PositionL2(clawAngle, elevator))
        .whileFalse(new StowHome(elevator, clawAngle));
    operator
        .b()
        .whileTrue(new PositionL3(clawAngle, elevator))
        .whileFalse(new StowHome(elevator, clawAngle));
    operator
        .x()
        .whileTrue(new ClawAngleAvoidElevator(clawAngle))
        .whileFalse(new ClawAngleHome(clawAngle));
    operator
        .y()
        .whileTrue(new PositionL4(clawAngle, elevator))
        .whileFalse(new StowHome(elevator, clawAngle));
    operator.leftBumper().whileTrue(new IntakeCoral(claw)).whileFalse(new StopIntake(claw));
    operator.rightBumper().whileTrue(new IntakeAlgae(claw)).whileFalse(new StopIntake(claw));
    operator
        .leftTrigger()
        .whileTrue(new PositionAlgaeL3(clawAngle, elevator, claw))
        .whileFalse(new StowHome(elevator, clawAngle));
    operator
        .rightTrigger()
        .whileTrue(new PositionAlgaeL2(clawAngle, elevator, claw))
        .whileFalse(new StowHome(elevator, clawAngle));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void teleopInit() {
    if (!this.m_TeleopInitialized) {
      // Only want to initialize starting position once (if teleop multiple times dont reset pose
      // again)
      vision.updateStartingPosition();
      // Turn on updating odometry based on Apriltags
      vision.enableUpdateOdometryBasedOnApriltags();
      m_TeleopInitialized = true;
      SignalLogger.setPath("/media/sda1/");
      SignalLogger.start();
    }
  }
}
