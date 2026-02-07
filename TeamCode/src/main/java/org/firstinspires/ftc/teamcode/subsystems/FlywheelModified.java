package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Simplified PIDF flywheel controller with dual-motor support for HORS robot.
 * Based on FlywheelController but streamlined for basic teleop use.
 */
@Configurable
public class FlywheelModified {

    private final DcMotorEx shooter;
    private final DcMotor shooter2; // mirrors power (opposite direction via motor direction)
    private final Telemetry telemetry; // nullable
    private final VoltageSensor voltageSensor; // nullable
    private final ElapsedTime timer = new ElapsedTime();

    // --- Configurable constants ---
    @Sorter(sort = 0) public static double MAX_RPM = 6000.0;
    @Sorter(sort = 1) public static double TICKS_PER_REV = 28.0;

    // --- PIDF coefficients ---
    @Sorter(sort = 2) public static double kP = 0.0022;
    @Sorter(sort = 3) public static double kI = 0.0016;
    @Sorter(sort = 4) public static double kD = 0.000005;
    @Sorter(sort = 5) public static double kF = 1.85;
    @Sorter(sort = 6) public static double integralLimit = 50;
    @Sorter(sort = 7) public static double derivativeAlpha = 0.9;
    @Sorter(sort = 8) public static double rpmFilterAlpha = 0.72;
    @Sorter(sort = 9) public static double powerSmoothingAlpha = 0.5;
    @Sorter(sort = 10) public static double ffReferenceVoltage = 13.0;
    @Sorter(sort = 11) public static double ffReferenceMaxTicksPerSec = 4930;
    @Sorter(sort = 12) public static double rpmTolerance = 50.0;

    // --- Internal state ---
    private double targetRpm = 2600;
    private double lastError = 0.0;
    private double integralSum = 0.0;
    private double lastDerivativeEstimate = 0.0;
    private int lastPos = 0;
    private double currentRpm = 0.0;
    private double lastAppliedPower = 0.0;

    private boolean shooterOn = true;
    private boolean lastAtTarget = false;

    /**
     * Constructor for dual-motor setup (shooter + shooter2)
     */
    public FlywheelModified(DcMotor shooter, DcMotor shooter2, Telemetry telemetry, VoltageSensor voltageSensor) {
        if (!(shooter instanceof DcMotorEx)) {
            throw new IllegalArgumentException("Primary shooter must be a DcMotorEx");
        }
        this.shooter = (DcMotorEx) shooter;
        this.shooter2 = shooter2;
        this.telemetry = telemetry;
        this.voltageSensor = voltageSensor;

        try {
            this.shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } catch (Exception e) {
            if (telemetry != null) telemetry.addData("FlywheelModified.init", "primary cfg failed: " + e.getMessage());
        }

        if (this.shooter2 != null) {
            try {
                this.shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                this.shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } catch (Exception e) {
                if (telemetry != null) telemetry.addData("FlywheelModified.init", "secondary cfg failed: " + e.getMessage());
            }
        }

        timer.reset();
        lastPos = this.shooter.getCurrentPosition();
    }

    /**
     * Backward-compatible constructor for single-motor setup
     */
    public FlywheelModified(DcMotor shooter, Telemetry telemetry, VoltageSensor voltageSensor) {
        this(shooter, null, telemetry, voltageSensor);
    }

    /**
     * Constructor without voltage sensor
     */
    public FlywheelModified(DcMotor shooter, DcMotor shooter2, Telemetry telemetry) {
        this(shooter, shooter2, telemetry, null);
    }

    /**
     * Single motor, no voltage sensor
     */
    public FlywheelModified(DcMotor shooter, Telemetry telemetry) {
        this(shooter, null, telemetry, null);
    }

    public void setTargetRPM(double rpm) {
        if (rpm != targetRpm) {
            // Reset integral on setpoint changes (anti-windup)
            integralSum = 0.0;
            lastError = 0.0;
            lastDerivativeEstimate = 0.0;
        }
        targetRpm = rpm;
    }

    public double getTargetRPM() {
        return targetRpm;
    }

    public boolean isAtSpeed() {
        double error = targetRpm - currentRpm;
        return Math.abs(error) <= rpmTolerance;
    }

    public void update() {
        double dt = timer.seconds();
        if (dt <= 0) dt = 1e-3; // avoid div/0

        double currentRpmNow = getCurrentRpm(dt);
        // Low-pass filter on measured RPM
        currentRpm = rpmFilterAlpha * currentRpmNow + (1.0 - rpmFilterAlpha) * currentRpm;

        timer.reset();

        double error = targetRpm - currentRpm;

        // Integral with clamping
        integralSum += error * dt;
        if (integralSum > integralLimit) integralSum = integralLimit;
        if (integralSum < -integralLimit) integralSum = -integralLimit;

        // Derivative with low-pass filter
        double rawDeriv = (error - lastError) / dt;
        double deriv = derivativeAlpha * lastDerivativeEstimate + (1.0 - derivativeAlpha) * rawDeriv;
        lastDerivativeEstimate = deriv;

        // Dynamic feedforward based on battery voltage
        double voltage = getBatteryVoltage();
        double maxTicksPerSec = (voltage / ffReferenceVoltage) * ffReferenceMaxTicksPerSec;
        if (maxTicksPerSec < 1e-3) maxTicksPerSec = 1e-3;
        double targetTicksPerSec = (targetRpm * TICKS_PER_REV) / 60.0;
        double ff = (targetTicksPerSec / maxTicksPerSec) * kF;

        // PIDF output
        double out = ff + (kP * error) + (kI * integralSum) + (kD * deriv);

        // Clamp to [-1, 1]
        out = Math.max(-1.0, Math.min(1.0, out));

        if (!shooterOn) out = 0.0;

        // Power smoothing
        double smoothedOut = powerSmoothingAlpha * out + (1.0 - powerSmoothingAlpha) * lastAppliedPower;

        // Apply to both motors
        try {
            shooter.setPower(smoothedOut);
        } catch (Exception e) {
            if (telemetry != null) telemetry.addData("FlywheelModified.power", "primary setPower failed: " + e.getMessage());
        }

        if (shooter2 != null) {
            try {
                shooter2.setPower(smoothedOut); // Motor direction handles opposite spin
            } catch (Exception e) {
                if (telemetry != null) telemetry.addData("FlywheelModified.power", "secondary setPower failed: " + e.getMessage());
            }
        }

        lastAppliedPower = smoothedOut;
        lastError = error;

        // At-target detection
        boolean atTargetNow = Math.abs(error) <= rpmTolerance;
        lastAtTarget = atTargetNow;
    }

    private double getBatteryVoltage() {
        try {
            if (voltageSensor != null) {
                double v = voltageSensor.getVoltage();
                if (v > 1e-3) return v;
            }
        } catch (Exception ignored) {}
        return 12.0; // fallback
    }

    private double getCurrentRpm(double dtSeconds) {
        double ticksPerSecond;
        try {
            ticksPerSecond = shooter.getVelocity(); // ticks/sec
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

    // --- Public API ---
    public void setShooterOn(boolean on) {
        shooterOn = on;
    }

    public boolean isShooterOn() {
        return shooterOn;
    }

    public double getCurrentRPM() {
        return currentRpm;
    }

    public double getLastAppliedPower() {
        return lastAppliedPower;
    }

    public boolean isAtTarget() {
        return isAtSpeed();
    }

    public void adjustTargetRPM(double delta) {
        setTargetRPM(Math.max(0.0, targetRpm + delta));
    }
}