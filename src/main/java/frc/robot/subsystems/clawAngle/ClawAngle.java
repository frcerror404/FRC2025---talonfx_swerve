package frc.robot.subsystems.clawAngle;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ClawAngle extends SubsystemBase {
  private final ClawAngleIO io;
  private final ClawAngleIOInputsAutoLogged inputs = new ClawAngleIOInputsAutoLogged();

  private double positionSetpoint;
  private boolean isFeeding;
  private boolean isEjecting;
  private double desiredVolts;

  // Creates a new ClawAngle subsystem
  public ClawAngle(ClawAngleIO io) {
    this.io = io;
  }

  public Command goToHighSetpoint() {
    return runToTargetPosition(100);
  }

  // Runs ClawAngle motor forwards
  public Command enableForward() {
    return Commands.run(
        () -> {
          // Enable the subsystem
          io.setVelocity(3000);
        },
        this);
  }

  // Runs ClawAngle motor backwards
  public Command enableBackward() {
    return Commands.run(
        () -> {
          // Enable the subsystem
          io.setVelocity(-3000);
        },
        this);
  }

  // Disables ClawAngle Motor
  public Command disable() {
    return Commands.run(
        () -> {
          // Enable the subsystem
          io.setVelocity(0);
        },
        this);
  }

  // Runs the ClawAngle to a target postion
  private Command runToTargetPosition(double position) {
    return Commands.run(
        () -> {
          // Set the position of the subsystem
          positionSetpoint = position;
          io.setPosition(position);
        },
        this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("ClawAngle", inputs);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
