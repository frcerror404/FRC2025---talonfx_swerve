package frc.robot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;

import static edu.wpi.first.units.Units.*;

public class RobotState {
    private static RobotState measuredInstance;
    private static RobotState desiredInstance;
    private static RobotState goalInstance;

    private MutDistance elevatorPosition;
    private MutAngle clawPosition;

    private RobotState() {
        elevatorPosition = Inches.mutable(0);
        clawPosition = Degrees.mutable(0);
    }

    public static RobotState getMeasuredInstance() {
        if (measuredInstance == null) {
            measuredInstance = new RobotState();
        }
        return measuredInstance;
    }

    public static RobotState getDesiredInstance() {
        if (desiredInstance == null) {
            desiredInstance = new RobotState();
        }
        return desiredInstance;
    }

    public static RobotState getGoalInstance() {
        if (goalInstance == null) {
            goalInstance = new RobotState();
        }
        return goalInstance;
    }

    public Distance getElevatorPosition() {
        return elevatorPosition;
    }

    public void updateElevatorPosition(Distance position) {
        elevatorPosition.mut_replace(position);
    }

    public Angle getClawPosition() {
        return clawPosition;
    }
    
    public void updateClawAngle(Angle position) {
        clawPosition.mut_replace(position);
    }
}