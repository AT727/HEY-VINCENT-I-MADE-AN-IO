package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

public class ArmIOSpark implements ArmIO {
  private CANSparkMax motor;
  private AbsoluteEncoder encoder;
  private SparkPIDController pid;

  public ArmIOSpark() {
    motor = new CANSparkMax(0, MotorType.kBrushless);
    encoder = motor.getAbsoluteEncoder();
    pid = motor.getPIDController();

    motor.enableVoltageCompensation(ArmConstants.kNominalVolts);
    motor.setInverted(ArmConstants.kInverted);
    motor.setIdleMode(ArmConstants.kIdleMode);
    motor.setSmartCurrentLimit(ArmConstants.kCurrentLimit);

    encoder.setPositionConversionFactor(ArmConstants.kPositionConversionFactor);
    encoder.setVelocityConversionFactor(ArmConstants.kVelocityConversionFactor);

    pid.setP(ArmConstants.gains.kP());
    pid.setI(ArmConstants.gains.kI());
    pid.setD(ArmConstants.gains.kD());
  }

  @Override
  public void setSetpoint(double setpoint, double feedforward) {
    pid.setReference(setpoint, ControlType.kPosition, 0, feedforward);
  }

  @Override
  public void setPower(double power) {
    motor.set(power);
  }

  @Override
  public void updateInputs(ArmInputs inputs) {
    inputs.position = encoder.getPosition();
    inputs.velocity = encoder.getVelocity();
    inputs.output = motor.get();
  }
}
