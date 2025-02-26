package frc.robot;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class RobotState extends VirtualSubsystem {
  private static RobotState instance;

  private MutDistance elevatorHeight = Inches.mutable(0);
  // private MutAngle shoulderAngle = Degrees.mutable(0);
  private MutAngle clawAngle = Degrees.mutable(0);

  // private final LoggedTunableNumber elevatorHeightTune =
  // new LoggedTunableNumber("robotState/elevatorHeight", 0);
  // private final LoggedTunableNumber shoulderAngleTune =
  //     new LoggedTunableNumber("robotState/elbowAngleZ", 0);
  // private final LoggedTunableNumber elbowAngleTune =
  //     new LoggedTunableNumber("robotState/elbowAngleX", 0);
  // private final LoggedTunableNumber wristTwistTune =
  //     new LoggedTunableNumber("robotState/elbowAngleY", 0);

  private final Mechanism2d primaryMechanism2d;

  private final MechanismRoot2d primaryMechanismRoot;
  private final MechanismLigament2d clawLigament2d;
  private final MechanismLigament2d elevatorLigament2d;

  private final MechanismRoot2d robotBaseRoot;
  private final MechanismLigament2d baseLigament2d =
      new MechanismLigament2d("RobotBase", 150, 0, 24, new Color8Bit(Color.kBlue));

  private MutAngle testStuff = Degrees.mutable(0);

  private final String key;

  private RobotState(String key) {
    this.key = key;

    primaryMechanism2d = new Mechanism2d(500, 300);
    elevatorLigament2d =
        new MechanismLigament2d("ElevatorLigament", elevatorHeight.in(Centimeters), 90);
    clawLigament2d =
        new MechanismLigament2d(
            "ClawLigament", Centimeters.convertFrom(15 + 5, Inches), clawAngle.in(Degrees));

    robotBaseRoot = primaryMechanism2d.getRoot("2dBaseRoot", 225, 20);
    robotBaseRoot.append(baseLigament2d);

    primaryMechanismRoot = primaryMechanism2d.getRoot("2dPrimary", 300, 20);
    primaryMechanismRoot.append(elevatorLigament2d);
    elevatorLigament2d.append(clawLigament2d);

    SmartDashboard.putData("Mech2d", primaryMechanism2d);
  }

  public static RobotState instance() {
    if (instance == null) {
      instance = new RobotState("measured");
    }
    return instance;
  }

  @Override
  public void periodic() {
    visualize();
  }

  public Distance getElevatorHeight() {
    return elevatorHeight;
  }

  public Angle getClawAngle() {
    return clawAngle;
  }

  public void setElevatorHeight(Distance elevatorHeight) {
    this.elevatorHeight.mut_replace(elevatorHeight);
  }

  public void setClawAngle(Angle clawAngle) {
    this.clawAngle.mut_replace(clawAngle);
  }

  public void setElevatorSource(MutDistance elevatorHeight) {
    this.elevatorHeight = elevatorHeight;
  }

  public void setClawAngleSource(MutAngle clawAngle) {
    this.clawAngle = clawAngle;
  }

  int counter = 0;

  private void visualize() {
    Pose3d elevatorPose =
        new Pose3d(ELEVATOR_ATTACH_OFFSET.getTranslation(), ELEVATOR_ATTACH_OFFSET.getRotation())
            .transformBy(
                new Transform3d(
                    new Translation3d(0, 0, -this.elevatorHeight.in(Meters)), new Rotation3d()));

    Pose3d clawPose =
        elevatorPose
            .transformBy(CLAW_ATTACH_OFFSET)
            .transformBy(
                new Transform3d(
                    new Translation3d(),
                    new Rotation3d(
                        Degrees.of(-136).minus(this.clawAngle), Degrees.zero(), Degrees.zero())))
            .transformBy(CLAW_PIVOT_OFFSET.inverse());

    double tempClawAngle = 90 + clawAngle.in(Degrees);

    elevatorLigament2d.setLength(elevatorHeight.in(Centimeters) + 103.5);
    clawLigament2d.setAngle(tempClawAngle);

    Logger.recordOutput("RobotState/Elevator/" + key, elevatorPose);
    Logger.recordOutput("RobotState/ClawAngle/" + key, clawPose);
  }

  private static final Transform3d ELEVATOR_ATTACH_OFFSET =
      new Transform3d(
          new Translation3d(Inches.of(2.125), Inches.of(-11.5), Inches.of(3.5)),
          new Rotation3d(Degrees.of(180), Degrees.of(0), Degrees.of(90)));

  private static final Transform3d CLAW_PIVOT_OFFSET =
      new Transform3d(
          new Translation3d(Inches.of(0), Inches.of(0), Inches.of(-38)),
          new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0)));

  private static final Transform3d CLAW_ATTACH_OFFSET =
      new Transform3d(
          new Translation3d(Inches.of(10.5), Inches.of(-1), Inches.of(-34.5)),
          new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0)));
}
