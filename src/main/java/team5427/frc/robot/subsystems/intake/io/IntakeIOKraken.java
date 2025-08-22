package team5427.frc.robot.subsystems.intake.io;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import team5427.frc.robot.subsystems.intake.IntakeConstants;
import team5427.lib.motors.SteelTalonFX;

public class IntakeIOKraken implements IntakeIO {
  private SteelTalonFX pivot;
  private SteelTalonFX flywheel;

  private StatusSignal<Angle> pivotPosition;
  private StatusSignal<AngularVelocity> pivotVelocity;
  private StatusSignal<AngularAcceleration> pivotAcceleration;
  private StatusSignal<Current> pivotCurrent;
  private StatusSignal<Voltage> pivotVoltage;
  private StatusSignal<Temperature> pivotTemperature;

  private StatusSignal<AngularVelocity> flywheelVelocity;
  private StatusSignal<AngularAcceleration> flywheelAcceleration;
  private StatusSignal<Current> flywheelCurrent;
  private StatusSignal<Voltage> flywheelVoltage;
  private StatusSignal<Temperature> flywheelTemperature;

  public IntakeIOKraken() {
    pivot = new SteelTalonFX(IntakeConstants.kPivotID);
    flywheel = new SteelTalonFX(IntakeConstants.kFlywheelID);
    pivot.apply(IntakeConstants.kPivotConfiguration);
    pivot.useTorqueCurrentFOC(true);
    flywheel.apply(IntakeConstants.kFlywheelConfiguration);
    flywheel.useTorqueCurrentFOC(false);

    pivot.setEncoderPosition(0);
    flywheel.setEncoderPosition(0);
    pivot.getTalonFX().clearStickyFaults();
    flywheel.getTalonFX().clearStickyFaults();

    // SS
    pivotPosition = pivot.getTalonFX().getPosition();
    pivotVelocity = pivot.getTalonFX().getVelocity();
    pivotAcceleration = pivot.getTalonFX().getAcceleration();
    pivotCurrent = pivot.getTalonFX().getStatorCurrent();
    pivotVoltage = pivot.getTalonFX().getMotorVoltage();
    pivotTemperature = pivot.getTalonFX().getDeviceTemp();

    flywheelVelocity = flywheel.getTalonFX().getVelocity();
    flywheelAcceleration = flywheel.getTalonFX().getAcceleration();
    flywheelCurrent = flywheel.getTalonFX().getStatorCurrent();
    flywheelVoltage = flywheel.getTalonFX().getMotorVoltage();
    flywheelTemperature = flywheel.getTalonFX().getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        pivotPosition,
        pivotVelocity,
        pivotAcceleration,
        pivotCurrent,
        pivotVoltage
    );

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        flywheelVelocity,
        flywheelAcceleration,
        flywheelCurrent,
        flywheelVoltage
    );

    BaseStatusSignal.setUpdateFrequencyForAll(
        20.0, 
        pivotTemperature,
        flywheelTemperature
    );

    ParentDevice.optimizeBusUtilizationForAll(pivot.getTalonFX(), flywheel.getTalonFX());
  }

  @Override
  public void updateInputs(IntakeIOInputsAutoLogged inputs) {
    BaseStatusSignal.refreshAll(
        pivotPosition,
        pivotVelocity,
        pivotAcceleration,
        pivotCurrent,
        pivotVoltage
    );
    BaseStatusSignal.refreshAll(
        flywheelVelocity,
        flywheelAcceleration,
        flywheelCurrent,
        flywheelVoltage
    );
    BaseStatusSignal.refreshAll(
        pivotTemperature,
        flywheelTemperature 
    );

    inputs.pivotPosition = pivotPosition.getValue();
    inputs.pivotVelocity = pivotVelocity.getValue();
    inputs.pivotAcceleration = pivotAcceleration.getValue();
    inputs.pivotCurrent = pivotCurrent.getValue();
    inputs.pivotVoltage = pivotVoltage.getValue();
    inputs.pivotTemperature = pivotTemperature.getValue();

    inputs.flywheelVelocity = flywheelVelocity.getValue();
    inputs.flywheelAcceleration = flywheelAcceleration.getValue();
    inputs.flywheelCurrent = flywheelCurrent.getValue();
    inputs.flywheelVoltage = flywheelVoltage.getValue();
    inputs.flywheelTemperature = flywheelTemperature.getValue();
  }

  @Override
  public void setSpeeds(LinearVelocity velocity) {
    flywheel.setSetpoint(velocity);
  }

  @Override
  public void setPosition(Angle angle) {
    pivot.setSetpoint(angle);
  }
}
