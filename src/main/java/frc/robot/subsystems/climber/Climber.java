package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private double positionSetpoint;
  private boolean isFeeding;
  private boolean isEjecting;
  private double desiredVolts;

  // Creates a new Climber subsystem
  public Climber(ClimberIO io) {
    this.io = io;
  }

  public Command gotToHighSetpoint() {
    return runToTargetPosition(110);
  }

  public Command goToLowSetpoint() {
    return runToTargetPosition(-30);
  }

  // Runs the Climber motors forward
  public Command enableForward() {
    return Commands.run(
        () -> {
          // Enable the subsystem
          io.setVelocity(3000);
        },
        this);
  }

  // Runs the motors Climber motors backwards
  public Command enableBackward() {
    return Commands.run(
        () -> {
          // Enable the subsystem
          io.setVelocity(-3000);
        },
        this);
  }

  // Disables Climber subsystem
  public Command disable() {
    return Commands.run(
        () -> {
          // Enable the subsystem
          io.setVelocity(0);
        },
        this);
  }

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
    Logger.processInputs("Climber", inputs);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
