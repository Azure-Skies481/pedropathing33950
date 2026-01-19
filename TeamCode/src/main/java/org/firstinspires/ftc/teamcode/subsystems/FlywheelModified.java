package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class FlywheelModified {

    private final DcMotorEx shooter;
    private final Telemetry telemetry; // nullable
    private final VoltageSensor voltageSensor; // nullable
    private final ElapsedTime timer = new ElapsedTime();

    @Sorter(sort = 0) public static double MAX_RPM = 6000.0;
    @Sorter(sort = 1) public static double TICKS_PER_REV = 28.0;

    // --- PIDF coefficients (ONLY CLOSE used now) ---
    @Sorter(sort = 2) public static double kP = 0.00245;
    @Sorter(sort = 3) public static double kI = 0.0027;
    @Sorter(sort = 4) public static double kD = 0.0;
    @Sorter(sort = 5) public static double kF = 1.92;
    @Sorter(sort = 6) public static double integralLimit = 50;
    @Sorter(sort = 7) public static double derivativeAlpha = 0.9;
    @Sorter(sort = 8) public static double rpmFilterAlpha = 0.8;
    @Sorter(sort = 9) public static double powerSmoothingAlpha = 0.25;
    @Sorter(sort = 10) public static double ffReferenceVoltage = 12.8;
    @Sorter(sort = 11) public static double ffReferenceMaxTicksPerSec = 5050;
    @Sorter(sort = 12) public static double rpmTolerance = 50.0;

    // --- Target RPM presets ---
    @Sorter(sort = 13) public static double closeRPM = 2600;

    // Legacy compatibility variables
    public static double TARGET_RPM_CLOSE = closeRPM;
    public static double TARGET_TOLERANCE_RPM = rpmTolerance;

    // --- Internal state ---
    private double targetRpm = closeRPM;
    private double lastError = 0.0;
    private double integralSum = 0.0;
    private double lastDerivativeEstimate = 0.0;
    private int lastPos = 0;
    private double currentRpm = 0.0;
    private double lastAppliedPower = 0.0;

    private boolean shooterOn = true;
    private boolean lastAtTarget = false;
    private boolean justReachedTargetFlag = false;
    private boolean leftTriggerLast = false;
    private double savedTargetBeforeTrigger = -1.0;
    private boolean savedShooterOnBeforeTrigger = false;

    public FlywheelModified(DcMotor shooter, Telemetry telemetry, VoltageSensor voltageSensor) {
        if (!(shooter instanceof DcMotorEx)) {
            throw new IllegalArgumentException("Shooter must be a DcMotorEx");
        }
        this.shooter = (DcMotorEx) shooter;
        this.telemetry = telemetry;
        this.voltageSensor = voltageSensor;

        try {
            this.shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } catch (Exception e) {
            if (telemetry != null) telemetry.addData("Flywheel.init", "shooter cfg failed: " + e.getMessage());
        }
        timer.reset();
        lastPos = this.shooter.getCurrentPosition();
    }

    public FlywheelModified(DcMotor shooter, Telemetry telemetry) {
        this(shooter, telemetry, null);
    }
    public FlywheelModified(DcMotor shooter) {
        this(shooter, null, null);
    }

    public void setTargetRpm(double rpm) {
        if (rpm != targetRpm) {
            integralSum = 0.0;
            lastError = 0.0;
            lastDerivativeEstimate = 0.0;
        }
        targetRpm = rpm;
    }
    public double getTargetRpm() { return targetRpm; }
    public boolean isAtSpeed() {
        double error = targetRpm - currentRpm;
        return Math.abs(error) <= rpmTolerance;
    }

    // These getters simply use the (single) PIDF settings now.
    private double getActiveKp() { return kP; }
    private double getActiveKi() { return kI; }
    private double getActiveKd() { return kD; }
    private double getActiveKf() { return kF; }
    private double getActiveIntegralLimit() { return integralLimit; }
    private double getActiveDerivativeAlpha() { return derivativeAlpha; }
    private double getActiveRpmFilterAlpha() { return rpmFilterAlpha; }
    private double getActivePowerSmoothingAlpha() { return powerSmoothingAlpha; }
    private double getActiveFfReferenceVoltage() { return ffReferenceVoltage; }
    private double getActiveFfReferenceMaxTicksPerSec() { return ffReferenceMaxTicksPerSec; }

    public void update() {
        double dt = timer.seconds();
        if (dt <= 0) dt = 1e-3;

        double currentRpmNow = getCurrentRpm(dt);
        double activeRpmFilterAlpha = getActiveRpmFilterAlpha();
        currentRpm = activeRpmFilterAlpha * currentRpmNow + (1.0 - activeRpmFilterAlpha) * currentRpm;

        timer.reset();

        double error = targetRpm - currentRpm;

        double activeKp = getActiveKp();
        double activeKi = getActiveKi();
        double activeKd = getActiveKd();
        double activeKf = getActiveKf();
        double activeIntegralLimit = getActiveIntegralLimit();
        double activeDerivativeAlpha = getActiveDerivativeAlpha();
        double activePowerSmoothingAlpha = getActivePowerSmoothingAlpha();
        double activeFfReferenceVoltage = getActiveFfReferenceVoltage();
        double activeFfReferenceMaxTicksPerSec = getActiveFfReferenceMaxTicksPerSec();

        integralSum += error * dt;
        if (integralSum > activeIntegralLimit) integralSum = activeIntegralLimit;
        if (integralSum < -activeIntegralLimit) integralSum = -activeIntegralLimit;

        double rawDeriv = (error - lastError) / dt;
        double deriv = activeDerivativeAlpha * lastDerivativeEstimate + (1.0 - activeDerivativeAlpha) * rawDeriv;
        lastDerivativeEstimate = deriv;

        double voltage = getBatteryVoltage();
        double maxTicksPerSec = (voltage / activeFfReferenceVoltage) * activeFfReferenceMaxTicksPerSec;
        if (maxTicksPerSec < 1e-3) maxTicksPerSec = 1e-3;
        double targetTicksPerSec = (targetRpm * TICKS_PER_REV) / 60.0;
        double ff = (targetTicksPerSec / maxTicksPerSec) * activeKf;

        double out = ff + (activeKp * error) + (activeKi * integralSum) + (activeKd * deriv);
        out = Math.max(-1.0, Math.min(1.0, out));
        if (!shooterOn) out = 0.0;

        double smoothedOut = activePowerSmoothingAlpha * out + (1.0 - activePowerSmoothingAlpha) * lastAppliedPower;

        try {
            shooter.setPower(smoothedOut);
        } catch (Exception e) {
            if (telemetry != null) telemetry.addData("Flywheel.power", "shooter setPower failed: " + e.getMessage());
        }

        lastAppliedPower = smoothedOut;
        lastError = error;

        boolean atTargetNow = Math.abs(error) <= rpmTolerance;
        if (atTargetNow && !lastAtTarget) justReachedTargetFlag = true;
        lastAtTarget = atTargetNow;

        if (telemetry != null) {
            telemetry.addData("PIDF Mode", "CLOSE ONLY");
        }
    }

    private double getBatteryVoltage() {
        try {
            if (voltageSensor != null) {
                double v = voltageSensor.getVoltage();
                if (v > 1e-3) return v;
            }
        } catch (Exception ignored) {}
        return 12.0;
    }

    private double getCurrentRpm(double dtSeconds) {
        double ticksPerSecond;
        try {
            ticksPerSecond = shooter.getVelocity();
            lastPos = shooter.getCurrentPosition();
        } catch (Exception e) {
            int pos = shooter.getCurrentPosition();
            int delta = pos - lastPos;
            lastPos = pos;
            double dt = (dtSeconds <= 0) ? 1e-3 : dtSeconds;
            ticksPerSecond = delta / dt;
        }
        return (ticksPerSecond * 60.0) / TICKS_PER_REV;
    }

    public void setCloseMode() { setTargetRpm(closeRPM); }
    public void setTargetRPM(double rpm) { setTargetRpm(rpm); }
    public double getTargetRPM() { return getTargetRpm(); }
    public void adjustTargetRPM(double delta) { setTargetRpm(Math.max(0.0, targetRpm + delta)); }
    public void toggleShooterOn() { shooterOn = !shooterOn; }
    public void setShooterOn(boolean on) { shooterOn = on; }
    public boolean isShooterOn() { return shooterOn; }
    public double getCurrentRPM() { return currentRpm; }
    public double getLastAppliedPower() { return lastAppliedPower; }
    public boolean isAtTarget() { return isAtSpeed(); }
    public boolean justReachedTarget() {
        if (justReachedTargetFlag) {
            justReachedTargetFlag = false;
            return true;
        }
        return false;
    }
    public void handleLeftTrigger(boolean leftTriggerNow) {
        if (leftTriggerNow && !leftTriggerLast) {
            savedTargetBeforeTrigger = targetRpm;
            savedShooterOnBeforeTrigger = shooterOn;
            shooterOn = true;
        } else if (!leftTriggerNow && leftTriggerLast) {
            if (savedTargetBeforeTrigger >= 0.0) savedTargetBeforeTrigger = -1.0;
            shooterOn = savedShooterOnBeforeTrigger;
            savedShooterOnBeforeTrigger = false;
        }
        leftTriggerLast = leftTriggerNow;
    }
    public void update(long ignoredNowMs, boolean ignoredCalibPressed) { update(); }
    public void setTargetToleranceRpm(double tolerance) {
        if (tolerance < 0) tolerance = 0;
        rpmTolerance = tolerance;
        TARGET_TOLERANCE_RPM = tolerance;
    }
    public double getTargetToleranceRpm() { return rpmTolerance; }
}