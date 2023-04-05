package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanConstants;

public class IntakeSubsystem extends SubsystemBase {

  public final CANSparkMax mIntakeMotor;

  private final RelativeEncoder mEncoder;

  public SparkMaxPIDController mPidController;

  public final int VELOCITY_SLOT = 0;

  public boolean intakeMotorConnected;

  private double setRPM;

  private int loopctr;

  public double deliverSpeed;

  public int deliverSensor;

  public int tstctr;

  public double rpm;

  public double amps;

  // -------- CONSTANTS --------\\
  private final int m_freeLimit = 10;
  private final int m_stallLimit = 30;

  public double voltage;

  public IntakeSubsystem() {

    mIntakeMotor = new CANSparkMax(CanConstants.INTAKE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

    mEncoder = mIntakeMotor.getEncoder();

    mPidController = mIntakeMotor.getPIDController();

    mIntakeMotor.restoreFactoryDefaults();

    mPidController.setFF(0, 0);

    mPidController.setP(.0001, 0);

    mIntakeMotor.setInverted(true);

    mIntakeMotor.setOpenLoopRampRate(0);

    mIntakeMotor.setClosedLoopRampRate(.5);

    mIntakeMotor.enableVoltageCompensation(12);

    mIntakeMotor.setSmartCurrentLimit(m_stallLimit, m_freeLimit);

    mIntakeMotor.setIdleMode(IdleMode.kBrake);

    mEncoder.setPositionConversionFactor(3);

    mEncoder.setVelocityConversionFactor(3);

    setCANTimes();

  }

  @Override
  public void periodic() {

    loopctr++;

    if (loopctr == 5) {

      intakeMotorConnected = checkCANOK();

    }

    if (loopctr == 10) {

      rpm = getRPM();

    }
    if (loopctr == 15) {

      amps = getAmps();

      loopctr = 0;

    }

  }

  @Override
  public void simulationPeriodic() {

  }

  public void close() {
    mIntakeMotor.close();

  }

  public boolean checkCANOK() {
    return RobotBase.isSimulation() || mIntakeMotor.getFirmwareVersion() != 0;

  }

  public void runTorqueMode(double amps) {
    mPidController.setReference(amps, ControlType.kCurrent);
  }

  public double getRPM() {
    return Math.round(mEncoder.getVelocity());
  }

  public boolean getIntakeAtSpeed() {
    return Math.abs(getRPM() - setRPM) < 25;

  }

  public boolean isStopped() {
    return Math.abs(getRPM()) < 10;
  }

  public String getFirmware() {
    return mIntakeMotor.getFirmwareString();
  }

  public double getAmps() {
    return mIntakeMotor.getOutputCurrent();
  }

  public void stop() {

    mIntakeMotor.stopMotor();

    mIntakeMotor.set(0);

  }

  public void setMotorVolts(double volts) {
    voltage = volts;
    mIntakeMotor.setVoltage(volts);
  }

  public double getAppliedOutput() {
    return mIntakeMotor.getAppliedOutput();
  }

  public boolean gamepieceInIntake() {
    return true;
  }

  public void clearFaults() {
    mIntakeMotor.clearFaults();

  }

  public int getIntakeFaults() {
    return mIntakeMotor.getFaults();
  }

  public String faultAsBitString() {
    return Integer.toBinaryString(getIntakeFaults());
  }

  public void stopIntake() {
    Commands.run(() -> this.stop(), this);
  }

  private void setCANTimes() {

    mIntakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    mIntakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    mIntakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50);

  }

}