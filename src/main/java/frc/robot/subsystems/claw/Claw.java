package frc.robot.subsystems.claw;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableNumber;

import static edu.wpi.first.units.Units.*;

public class Claw extends SubsystemBase {

    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Claw/Gains/kP", 0.1);
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Claw/Gains/kI", 0.0);
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Claw/Gains/kD", 0.0);
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Claw/Gains/kS", 0.0);
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Claw/Gains/kV", 1.45);
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Claw/Gains/kA", 0.0);
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Claw/Gains/kG", 0.0);

    private final ClawIO io;
    private final ClawIOInputsAutoLogged inputs = new ClawIOInputsAutoLogged();

    private final RobotState actual;
    private final RobotState target;
    private final RobotState goal;

    private Angle setpoint = Degrees.of(0.0);

  // Creates a new Claw Subsystem
  public Claw(ClawIO io) {
    this.io = io;
    this.io.setPID(kP.get(), kI.get(), kD.get());
        this.io.setFF(kS.get(), kG.get(), kV.get(), kA.get());
        this.actual = RobotState.getMeasuredInstance();
        this.target = RobotState.getDesiredInstance();
        this.goal = RobotState.getGoalInstance();
  }

  @Override
    public void periodic() {
        super.periodic();

        if(edu.wpi.first.wpilibj.RobotState.isDisabled()) {
            this.io.stop();
        } else {
            this.io.updateInputs(inputs);
            Logger.processInputs("Claw", inputs);

            LoggedTunableNumber.ifChanged(
                hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);

            LoggedTunableNumber.ifChanged(
                hashCode(), () -> io.setFF(kS.get(), kG.get(), kV.get(), kA.get()), kS, kG, kV, kA);

            this.io.runSetpoint(this.setpoint);
        }

        actual.updateClawAngle(this.inputs.position);
        target.updateClawAngle(this.inputs.setpointPosition);
        goal.updateClawAngle(this.setpoint);
    }

    public Command setPosition(Angle position) {
        return runOnce(() -> this.setpoint = position);
    }

}
