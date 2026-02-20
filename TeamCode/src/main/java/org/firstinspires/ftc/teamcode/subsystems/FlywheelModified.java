package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * PIDF flywheel controller with dual-motor support for HORS robot.
 * Tuned for stable RPM control.
 */
@Configurable
public class FlywheelModified {

    private final DcMotorEx shooter;
    private final DcMotorEx shooter2;
    private final Telemetry telemetry;
    private final VoltageSensor voltageSensor;
    private final ElapsedTime timer = new ElapsedTime();

    // --- Configurable constants ---
    @Sorter(sort = 0) public static double MAX_RPM = 6000.0;
    @Sorter(sort = 1) public static double TICKS_PER_REV = 28.0;

    // --- TUNED PIDF coefficients ---
    @Sorter(sort = 2) public static double kP = 0.00015;     // REDUCED - P alone was causing full power
    @Sorter(sort = 3) public static double kI = 0.0001;      // Small integral to eliminate steady-state error
    @Sorter(sort = 4) public static double kD = 0.00008;     // INCREASED - more damping to slow down near target
    @Sorter(sort = 5) public static double kF = 1.0;         // FIXED - this is the main driver, ~1.0 is good baseline
    @Sorter(sort = 6) public static double integralLimit = 15;  // Reduced to prevent windup
    @Sorter(sort = 7) public static double derivativeAlpha = 0.7;
    @Sorter(sort = 8) public static double rpmFilterAlpha = 0.5;   // Smoother RPM reading
    @Sorter(sort = 9) public static double powerSmoothingAlpha = 0.15;  // Much smoother power changes
    @Sorter(sort = 10) public static double ffReferenceVoltage = 13.0;
    @Sorter(sort = 11) public static double ffReferenceMaxTicksPerSec = 4930;
    @Sorter(sort = 12) public static double rpmTolerance = 75.0;

    // --- Internal state ---
    private double targetRpm = 2600;
    private double lastError = 0.0;
    private double integralSum = 0.0;
    private double lastDerivativeEstimate = 0.0;
    private int lastPos = 0;
    private double currentRpm = 0.0;
    private double lastAppliedPower = 0.0;

    private boolean shooterOn = false;
    private boolean lastAtTarget = false;

    public FlywheelModified(DcMotor shooter, DcMotor shooter2, Telemetry telemetry, VoltageSensor voltageSensor) {
        if (!(shooter instanceof DcMotorEx)) {
            throw new IllegalArgumentException("Primary shooter must be a DcMotorEx");
        }
        this.shooter = (DcMotorEx) shooter;

        if (shooter2 != null && shooter2 instanceof DcMotorEx) {
            this.shooter2 = (DcMotorEx) shooter2;
        } else {
            this.shooter2 = null;
        }

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

    public FlywheelModified(DcMotor shooter, Telemetry telemetry, VoltageSensor voltageSensor) {
        this(shooter, null, telemetry, voltageSensor);
    }

    public FlywheelModified(DcMotor shooter, DcMotor shooter2, Telemetry telemetry) {
        this(shooter, shooter2, telemetry, null);
    }

    public FlywheelModified(DcMotor shooter, Telemetry telemetry) {
        this(shooter, null, telemetry, null);
    }

    public void setTargetRPM(double rpm) {
        rpm = Math.max(0.0, Math.min(rpm, MAX_RPM));

        if (rpm != targetRpm) {
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
        if (dt <= 0) dt = 1e-3;

        double currentRpmNow = getCurrentRpm(dt);
        currentRpm = rpmFilterAlpha * currentRpmNow + (1.0 - rpmFilterAlpha) * currentRpm;

        timer.reset();

        // If shooter is off, set power to 0
        if (!shooterOn) {
            try {
                shooter.setPower(0.0);
            } catch (Exception ignored) {}

            if (shooter2 != null) {
                try {
                    shooter2.setPower(0.0);
                } catch (Exception ignored) {}
            }

            lastAppliedPower = 0.0;
            return;
        }

        double error = targetRpm - currentRpm;

        // Only accumulate integral when close to target (prevents windup during spinup)
        if (Math.abs(error) < 300) {
            integralSum += error * dt;
            if (integralSum > integralLimit) integralSum = integralLimit;
            if (integralSum < -integralLimit) integralSum = -integralLimit;
        } else {
            // Reset integral when far from target
            integralSum = 0.0;
        }

        // Reset integral on overshoot to recover quickly
        if (error < -50) {
            integralSum = 0.0;
        }

        // Derivative with low-pass filter
        double rawDeriv = (error - lastError) / dt;
        double deriv = derivativeAlpha * lastDerivativeEstimate + (1.0 - derivativeAlpha) * rawDeriv;
        lastDerivativeEstimate = deriv;

        // Feedforward: calculate base power needed for target RPM
        double voltage = getBatteryVoltage();
        double maxTicksPerSec = (voltage / ffReferenceVoltage) * ffReferenceMaxTicksPerSec;
        if (maxTicksPerSec < 1e-3) maxTicksPerSec = 1e-3;
        double targetTicksPerSec = (targetRpm * TICKS_PER_REV) / 60.0;
        double ff = (targetTicksPerSec / maxTicksPerSec) * kF;

        // PID correction on top of feedforward
        double pid = (kP * error) + (kI * integralSum) + (kD * deriv);

        // Total output
        double out = ff + pid;

        // Clamp to [0, 1]
        out = Math.max(0.0, Math.min(1.0, out));

        // Power smoothing - blend new output with previous
        double smoothedOut = powerSmoothingAlpha * out + (1.0 - powerSmoothingAlpha) * lastAppliedPower;

        // Apply to both motors
        try {
            shooter.setPower(smoothedOut);
        } catch (Exception e) {
            if (telemetry != null) telemetry.addData("FlywheelModified.power", "primary setPower failed: " + e.getMessage());
        }

        if (shooter2 != null) {
            try {
                shooter2.setPower(smoothedOut);
            } catch (Exception e) {
                if (telemetry != null) telemetry.addData("FlywheelModified.power", "secondary setPower failed: " + e.getMessage());
            }
        }

        lastAppliedPower = smoothedOut;
        lastError = error;

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

    public void setShooterOn(boolean on) {
        if (on && !shooterOn) {
            integralSum = 0.0;
            lastError = 0.0;
            lastDerivativeEstimate = 0.0;
            lastAppliedPower = 0.0;
        }
        shooterOn = on;
    }

    public boolean isShooterOn() {
        return shooterOn;
    }

    public void toggleShooterOn() {
        setShooterOn(!shooterOn);
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
        setTargetRPM(targetRpm + delta);
    }

    public boolean isDualMotor() {
        return shooter2 != null;
    }
}
