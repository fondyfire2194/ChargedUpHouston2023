package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Pref;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.ExtendArmConstants;

public class ExtendArmSubsystem extends SubsystemBase {

    public enum presetExtArmDistances {

        /**
         * Exten ard distances ar in inches and range from 0 t0 25
         * Arm will be preset to 1 inch on startup. This allows for a short retract
         * to clear the wrist from the start up position
         * 
         * 
         * 
         */

        RETRACT(0.25),

        HOME(1),

        TRAVEL(2.75),

        PICKUP_CUBE_GROUND(10.0),

        PICKUP_UPRIGHT_CONE_GROUND(10.25),

        PICKUP_SIDE_LOAD_STATION(.2),

        PICKUP_LOAD_STATION(0.2),

        PLACE_GROUND(0),

        PLACE_MID(7.8),

        PLACE_TOP(20.92);

        private double distance;

        private presetExtArmDistances(double distance) {
            this.distance = distance;
        }

        public double getDistance() {
            return distance;
        }
    }

    public final int VELOCITY_SLOT = 0;

    public final CANSparkMax m_motor = new CANSparkMax(CanConstants.EXTEND_ARM_MOTOR,
            CANSparkMaxLowLevel.MotorType.kBrushless);
    private final RelativeEncoder mEncoder = m_motor.getEncoder();

    public SparkMaxPIDController m_posnController;

    public ProfiledPIDController m_extController = new ProfiledPIDController(0.1, 0, 0,
            ExtendArmConstants.extendArmFastConstraints);

    public SparkMaxLimitSwitch m_reverseLimit;

    public SparkMaxLimitSwitch m_forwardLimit;

    private double inPositionBandwidth = .25;

    private double inRangeBandwidth = 2;

    public boolean extendMotorConnected;

    public int extendFaultSeen;

    private double m_positionSim;

    private double maxinchespersec = 2;

    public double inchespersecpervolt = maxinchespersec / 12;

    private double positionChangeper20ms;

    public SimpleMotorFeedforward m_feedforward;

    public double deliverDistance = presetExtArmDistances.PLACE_TOP.getDistance();

    private boolean useSoftwareLimit;

    public double appliedOutput;

    public double amps;

    public double positionInches;

    public double inchespersec;

    private int loopctr;

    public int tstctr;

    public double ff;// feedforward

    public boolean resetFF;

    public double commandIPS;

    public double goalInches;

    public boolean firstUp;

    // public boolean endComm;

    public double pidVal;

    public double volts;

    public boolean inIZone;

    public double gravVal;

    public boolean atGoal;

    private double nextTarget;

    public double positionTarget;

    public boolean atDepth;

    public int extendStickyFaultSeen;

    public ExtendArmSubsystem() {

        firstUp = true;

        useSoftwareLimit = false;

        m_motor.restoreFactoryDefaults();
        m_motor.setInverted(true);
        m_motor.setOpenLoopRampRate(.1);
        m_motor.setClosedLoopRampRate(.1);

        m_posnController = m_motor.getPIDController();

        m_posnController.setP(.01);

        mEncoder.setPositionConversionFactor(ExtendArmConstants.INCHES_PER_ENCODER_REV);

        mEncoder.setVelocityConversionFactor(ExtendArmConstants.INCHES_PER_ENCODER_REV / 60);

        mEncoder.setPosition(presetExtArmDistances.HOME.getDistance());

        m_motor.setSmartCurrentLimit(25);

        m_motor.setIdleMode(IdleMode.kBrake);

        setCANTimes();

        setController(ExtendArmConstants.extendArmFastConstraints, presetExtArmDistances.HOME.getDistance(), true);

        if (RobotBase.isSimulation()) {

            setControllerGoal(ExtendArmConstants.MIN_POSITION);

        }

        m_reverseLimit = m_motor.getReverseLimitSwitch(Type.kNormallyClosed);

        m_reverseLimit.enableLimitSwitch(true);

        m_forwardLimit = m_motor.getForwardLimitSwitch(Type.kNormallyClosed);

        m_forwardLimit.enableLimitSwitch(true);

        SmartDashboard.putNumber("EXENCSET",
                ExtendArmConstants.INCHES_PER_ENCODER_REV);
        SmartDashboard.putNumber("EXENMAXV",
                ExtendArmConstants.MAX_RATE_INCHES_PER_SEC);

        setSoftwareLimits();

        enableSoftLimits(useSoftwareLimit);

        m_feedforward = new SimpleMotorFeedforward(ExtendArmConstants.ksExtArmVolts,
                ExtendArmConstants.kvExtArmVoltSecondsPerInch);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        loopctr++;

        if (extendFaultSeen == 0)
            extendFaultSeen = getFaults();

        if (extendStickyFaultSeen != 0)
            extendStickyFaultSeen = getStickyFaults();

        // for Shuffleboard
        if (loopctr >= 7) {
            // appliedOutput = round2dp(getAppliedOutput());
            amps = round2dp(getAmps());
            positionInches = round2dp(getPositionInches());
            inchespersec = round2dp(getInchesPerSec());
            extendMotorConnected = checkCANOK();
            atGoal = m_extController.atGoal();
            loopctr = 0;

        }

        SmartDashboard.putBoolean("REVLS", m_reverseLimit.isPressed());
        SmartDashboard.putBoolean("FWDLS", m_forwardLimit.isPressed());

    }

    @Override
    public void simulationPeriodic() {

        positionChangeper20ms = getAppliedOutput() * inchespersecpervolt / 50;

        m_positionSim += positionChangeper20ms;

    }

    public boolean checkCANOK() {
        return RobotBase.isSimulation() || m_motor.getFirmwareVersion() != 0;
    }

    public void close() {
        m_motor.close();
    }

    public void setControllerConstraints(TrapezoidProfile.Constraints constraints) {
        m_extController.setConstraints(constraints);
    }

    public void setControllerGoal(double position) {
        m_extController.setGoal(position);
    }

    public void setController(TrapezoidProfile.Constraints constraints, double distance, boolean initial) {

        if (isStopped()) {
            setControllerConstraints(constraints);
            setControllerGoal(distance);
            goalInches = distance;
            if (initial)
                m_extController.reset(new TrapezoidProfile.State(presetExtArmDistances.HOME.getDistance(), 0));
            else
                m_extController.reset(new TrapezoidProfile.State(getPositionInches(), getInchesPerSec()));

        }
        m_feedforward = new SimpleMotorFeedforward(Pref.getPref("extKs"), Pref.getPref("extKv"));
        m_extController.setP(Pref.getPref("extKp"));
    }

    public void setControllerAtPosition() {
        goalInches = getPositionInches();
        setController(ExtendArmConstants.extendArmFastConstraints, goalInches, false);
    }

    public void redoTarget() {
        setController(ExtendArmConstants.extendArmFastConstraints, goalInches, false);
    }

    public void incGoal(double val) {
        double temp = getPositionInches() + val;
        setController(ExtendArmConstants.extendArmFastConstraints, temp, false);

    }

    public void resetPosition(double position) {
        mEncoder.setPosition(position);

    }

    public void setNextTarget(double value) {
        nextTarget = value;
    }

    public double getNextTarget() {
        return nextTarget;
    }

    public boolean atTargetPosition() {
        return Math.abs(goalInches - getPositionInches()) < inPositionBandwidth;
    }

    public boolean inRange(double range) {
        return Math.abs(goalInches - getPositionInches()) < range;
    }

    public boolean isStopped() {
        return Math.abs(mEncoder.getVelocity()) < .5;
    }

    public double getPositionInches() {
        if (RobotBase.isReal())
            return mEncoder.getPosition();
        else
            return m_positionSim;
    }

    public double getAppliedOutput() {
        return m_motor.getAppliedOutput();
    }

    public double getAmps() {
        return m_motor.getOutputCurrent();
    }

    public double getInchesPerSec() {
        return mEncoder.getVelocity();
    }

    public String getFirmware() {
        return m_motor.getFirmwareString();
    }

    public boolean onPlusSoftwareLimit() {
        return m_motor.getFault(FaultID.kSoftLimitFwd);
    }

    public boolean onMinusSoftwareLimit() {
        return m_motor.getFault(FaultID.kSoftLimitRev);
    }

    public boolean onPlusHardwareLimit() {
        return m_motor.getFault(FaultID.kHardLimitRev);
    }

    public boolean onMinusHardwareLimit() {
        return m_motor.getFault(FaultID.kHardLimitRev);
    }

    public void stop() {
        m_motor.set(0);
    }

    public void setSoftwareLimits() {
        m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) ExtendArmConstants.MIN_POSITION);
        m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) ExtendArmConstants.MAX_POSITION);
        m_motor.setIdleMode(IdleMode.kBrake);

    }

    public void enableSoftLimits(boolean on) {
        m_motor.enableSoftLimit(SoftLimitDirection.kForward, on);
        m_motor.enableSoftLimit(SoftLimitDirection.kReverse, on);
    }

    public boolean isBrake() {
        return m_motor.getIdleMode() == IdleMode.kBrake;
    }

    public boolean getSoftwareLimitsEnabled() {
        return m_motor.isSoftLimitEnabled(SoftLimitDirection.kForward)
                || m_motor.isSoftLimitEnabled(SoftLimitDirection.kReverse);
    }

    public Command clearFaults() {
        extendFaultSeen = 0;
        extendStickyFaultSeen = 0;
        return Commands.runOnce(() -> m_motor.clearFaults());

    }

    public int getFaults() {
        return m_motor.getFaults();
    }

    public int getStickyFaults() {
        return m_motor.getStickyFaults();
    }

    public double round2dp(double number) {
        number = Math.round(number * 100);
        number /= 100;
        return number;
    }

    private void setCANTimes() {
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50);// vel

    }

}
