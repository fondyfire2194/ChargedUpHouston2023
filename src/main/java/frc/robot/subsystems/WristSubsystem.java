package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Pref;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.WristConstants;

public class WristSubsystem extends SubsystemBase {

    /**
     * Wrist arms are defined with respect to the extend arm. positive angles are up
     * and negative down
     * Home angle position for the wrist is 90 - 51.3(mark dwg)= 38.7
     * 
     * 
     * 
     */

    public enum presetWristAngles {

        HOME(7),

        TRAVEL(20), //38

        PICKUP_CUBE_GROUND(46),

        PICKUP_CONE_GROUND(54),

        PICKUP_UPRIGHT_CONE_GROUND(55), // 40

        PICKUP_SIDE_LOAD_STATION(15), //10

        PICKUP_LOAD_STATION(85), //90

        PLACE_GROUND(15),

        PLACE_MID(86), // 108

        PLACE_TOP(100); // 108

        private double angle;

        private presetWristAngles(double angle) {
            this.angle = angle;
        }

        public double getAngleRads() {
            return Units.degreesToRadians(this.angle);
        }

    }

    public double pidVal = 0;

    public final CANSparkMax m_motor;

    private final RelativeEncoder mEncoder;

    public ArmFeedforward m_wristfeedforward;

    private double inPositionBandwidth = .1;// 5 degrees

    private double inRangeBandwidth = .2;// rads 10 degrees

    public boolean wristMotorConnected;

    public int wristFaultSeen;

    public int wristStickyFaultSeen;

    private double m_positionSim;

    private double maxdegpersec = 50;

    public double radspersecpervolt = maxdegpersec / 12;

    private double positionChangeper20ms;

    public ProfiledPIDController m_wristController = new ProfiledPIDController(1.0, 0, 0,
            WristConstants.wristFastConstraints, .02);

    public double deliverAngleRads = presetWristAngles.PLACE_TOP.getAngleRads();

    public double appliedOutput;

    public double amps;

    public double radspersec;

    private int loopctr;

    private boolean useSoftwareLimit;

    public TrapezoidProfile.State profile;

    public double ff;

    public boolean resetFF;

    public double commandRadPerSec;

    public double goalAngleRadians;

    public int tstCtr;

    public double volts;

    public boolean inIZone;

    public double angleRadians;

    public double angleDegrees;

    public boolean atGoal;

    public double acceleration;

    public double gravVal;

    public double netWristAngle;

    public WristSubsystem() {

        useSoftwareLimit = false;
        m_motor = new CANSparkMax(CanConstants.WRIST_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        mEncoder = m_motor.getEncoder();
        m_motor.restoreFactoryDefaults();
        m_motor.setInverted(false);
        m_motor.setOpenLoopRampRate(.1);
        m_motor.setClosedLoopRampRate(.1);
        m_motor.enableVoltageCompensation(12);

        mEncoder.setPositionConversionFactor(WristConstants.RADIANS_PER_ENCODER_REV);

        mEncoder.setVelocityConversionFactor(WristConstants.RADIANS_PER_ENCODER_REV / 60);

     //   SmartDashboard.putNumber("WRRDPR", WristConstants.RADIANS_PER_ENCODER_REV);

      //  SmartDashboard.putNumber("WRDGPR", Units.radiansToDegrees(WristConstants.RADIANS_PER_ENCODER_REV));

        mEncoder.setPosition(presetWristAngles.HOME.getAngleRads());

        setController(WristConstants.wristFastConstraints, presetWristAngles.HOME.getAngleRads(), false);

        m_motor.setSmartCurrentLimit(20);

        m_motor.setIdleMode(IdleMode.kBrake);

        setCANTimes();

        setSoftwareLimits();

        enableSoftLimits(useSoftwareLimit);

        m_wristfeedforward = new ArmFeedforward(Pref.getPref("wristKs"), 0,
                Pref.getPref("wristKv"));

        m_wristController.setP(Pref.getPref("wristKp"));

        // m_wristfeedforward = new ArmFeedforward(WristConstants.ksVolts, 0,
        //         WristConstants.kvWristVoltSecondsPerRadian, WristConstants.kaWristVoltSecondsSquaredPerRadian);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        loopctr++;

        if (wristFaultSeen == 0)
            wristFaultSeen = getFaults();

        if (wristStickyFaultSeen == 0)
            wristStickyFaultSeen = getStickyFaults();

        if (loopctr == 9) {

            // appliedOutput = round2dp(getAppliedOutput());
            amps = round2dp(getAmps());
            angleDegrees = round2dp(getAngleDegrees());
            angleRadians = round2dp(Units.degreesToRadians(angleDegrees));
            atGoal = m_wristController.atGoal();

        }

        if (loopctr == 12) {

            radspersec = round2dp(getRadsPerSec());
            wristMotorConnected = checkCANOK();
            loopctr = 0;

        }

    }

    @Override
    public void simulationPeriodic() {

        positionChangeper20ms = getAppliedOutput() * radspersecpervolt / 50;

        m_positionSim += positionChangeper20ms;

    }

    public boolean checkCANOK() {
        return RobotBase.isSimulation() || m_motor.getFirmwareVersion() != 0;
    }

    public void close() {
        m_motor.close();
    }

    public void setControllerConstraints(TrapezoidProfile.Constraints constraints) {
        m_wristController.setConstraints(constraints);
    }

    public void setControllerGoal(double angleRadians) {
        m_wristController.setGoal(angleRadians);
        // SmartDashboard.putNumber("GAR", m_wristController.getGoal().position);
    }

    public void setController(TrapezoidProfile.Constraints constraints, double angleRads, boolean initial) {

        if (isStopped()) {
            setControllerConstraints(constraints);
            setControllerGoal(angleRads);
            goalAngleRadians = angleRads;
            // SmartDashboard.putNumber("GARRRRRRRRRR", angleRads);
            if (initial)
                m_wristController.reset(new TrapezoidProfile.State(presetWristAngles.HOME.getAngleRads(), 0));
            else
                m_wristController.reset(new TrapezoidProfile.State(getAngleRadians(), getRadsPerSec()));

        }

        m_wristfeedforward = new ArmFeedforward(Pref.getPref("wristKs"), 0,
                Pref.getPref("wristKv"));

        m_wristController.setP(Pref.getPref("wristKp"));

    }

    public void setControllerAtPosition() {
        goalAngleRadians = getAngleRadians();
        setController(WristConstants.wristFastConstraints, goalAngleRadians, false);

    }

    public void redoTarget() {
        setController(WristConstants.wristFastConstraints, goalAngleRadians, false);

    }

    public void incGoal(double val) {

        double temp = getAngleRadians() + val;

        setController(WristConstants.wristFastConstraints, temp, false);

    }

    public void resetAngle() {
        mEncoder.setPosition(0);
    }

    public boolean atTargetAngle() {
        return Math.abs(goalAngleRadians - getAngleDegrees()) < inPositionBandwidth;
    }

    public boolean inRange(double range) {
        return Math.abs(goalAngleRadians - getAngleRadians()) < range;
    }

    public double getAngleDegrees() {
        if (RobotBase.isReal())
            return Units.radiansToDegrees(mEncoder.getPosition());
        else
            return m_positionSim;
    }

    public double getAngleRadians() {
        if (RobotBase.isReal())
            return mEncoder.getPosition();
        else
            return Units.degreesToRadians(m_positionSim);
    }

    public double getAppliedOutput() {
        return m_motor.getAppliedOutput();
    }

    public boolean isStopped() {
        return Math.abs(getRadsPerSec()) < .01;
    }

    public double getAmps() {
        return m_motor.getOutputCurrent();
    }

    public double getRadsPerSec() {
        return mEncoder.getVelocity();
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

    public String getFirmware() {
        return m_motor.getFirmwareString();
    }

    public void setSoftwareLimits() {
        m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) WristConstants.MIN_ANGLE);
        m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) WristConstants.MAX_ANGLE);
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
        wristFaultSeen = 0;
        wristStickyFaultSeen = 0;
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
