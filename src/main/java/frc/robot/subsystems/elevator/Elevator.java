package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

// Note: Implement 2 elevator motors in code. And adjust position and velocity values as needed //
//
//
//

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private double positionSetpoint;
  
  // Creates a new ELevator subsystem
  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  public Command goToSetpoint() {
    return runToTargetPosition(50);
  }

  public Command lowerElevator() {
    return runToTargetPosition(0);
  }

  // Runs an Elevator motor forwards
  public Command enableForward() {
    return Commands.run(
        () -> {
          // Enable the subsystem
          io.setVelocity(3000);
        },
        this);
  }

  // Runs a Elevator motor backwards
  public Command enableBackward() {
    return Commands.run(
        () -> {
          // Enable the subsystem
          io.setVelocity(-3000);
        },
        this);
  }

  // Disables the Elevator subsystem
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
    Logger.processInputs("Elevator", inputs);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
