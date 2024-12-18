package frc.robot.subsystems;

public interface ArmIO {
  default void setSetpoint(double setpoint, double feedforward) {}

  default void setPower(double power) {}

  default void updateInputs(ArmInputs inputs) {}
}
