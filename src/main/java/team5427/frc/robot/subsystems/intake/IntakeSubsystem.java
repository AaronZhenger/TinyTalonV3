package team5427.frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import team5427.frc.robot.subsystems.intake.io.IntakeIO;
import team5427.frc.robot.subsystems.intake.io.IntakeIOInputsAutoLogged;
import team5427.frc.robot.subsystems.intake.io.IntakeIOKraken;

public class IntakeSubsystem extends SubsystemBase {
  private IntakeIO intake;
  private IntakeIOInputsAutoLogged inputs;

  private Angle targetAngle = Degrees.of(0);
  private LinearVelocity targetSpeeds = MetersPerSecond.of(0);

  public IntakeSubsystem() {
    intake = new IntakeIOKraken();
  }

  @Override
  public void periodic() {
    intake.updateInputs(inputs);
    intake.setPosition(targetAngle);
    intake.setSpeeds(targetSpeeds);
    Logger.processInputs("IntakeInputsAutologged", inputs);
  }

  public void setTargetAngle(Angle targetAngle) {
    this.targetAngle = targetAngle;
  }

  public void setTargetSpeeds(LinearVelocity targetSpeeds) {
    this.targetSpeeds = targetSpeeds;
  }

  public boolean pivotAtTarget() {
    return Math.abs(targetAngle.in(Degrees) - inputs.pivotPosition.in(Degrees)) <= 4;
  }
}
