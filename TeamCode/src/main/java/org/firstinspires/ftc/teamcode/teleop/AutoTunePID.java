package org.firstinspires.ftc.teamcode. teleop;

import com.bylazar.configurables.annotations. Configurable;
import com.bylazar. configurables.annotations. Sorter;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode. TeleOp;
import com.qualcomm. robotcore.hardware.DcMotor;
import com.qualcomm.robotcore. hardware.DcMotorEx;
import com. qualcomm.robotcore.hardware.DcMotorSimple;
import com. qualcomm.robotcore.hardware. Servo;
import com.qualcomm. robotcore.util. ElapsedTime;

@TeleOp
@Configurable
public class AutoTunePID extends LinearOpMode {

    private DcMotorEx intake = null;
    private DcMotorEx shooterMotor = null;
    private Servo servoGate = null;

    // Panels-configurable values (tuned live via Panels/Sorter)
    @Sorter(sort = 0) public static double CFG_MAX_RPM = 1400.0;
    @Sorter(sort = 1) public static double CFG_TICKS_PER_REV = 112.0;   // 28 PPR * 4
    @Sorter(sort = 2) public static double CFG_DEFAULT_RPM = 670;
    @Sorter(sort = 3) public static double CFG_RPM_INCREMENT = 50.0;
    @Sorter(sort = 4) public static double CFG_RPM_TOLERANCE = 100.0;

    // PIDF tuning - will be auto-tuned
    @Sorter(sort = 5) public static double CFG_kP = 10.0;
    @Sorter(sort = 6) public static double CFG_kI = 0.1;
    @Sorter(sort = 7) public static double CFG_kD = 0.0;
    @Sorter(sort = 8) public static double CFG_kF = 10.0;

    // Auto-tune settings
    @Sorter(sort = 9) public static double CFG_TARGET_SETTLE_TIME_MS = 500.0;  // desired time to reach RPM
    @Sorter(sort = 10) public static double CFG_TUNE_STEP = 1.5;               // multiplier for tuning adjustments

    // Runtime constants
    private static final double SHOOTER_MIN_RPM = 0.0;
    private static final int RUMBLE_DURATION_MS = 2000;
    private static final long DPAD_DEBOUNCE_MS = 20;

    // Servo gate positions
    private static final double GATE_OPEN = 0.1;
    private static final double GATE_CLOSED = 0.6;

    // State
    private double targetShooterRpm = CFG_DEFAULT_RPM;
    private boolean shooterEnabled = false;

    // Edge/debounce
    private boolean prevDpadDown = false;
    private boolean prevDpadLeft = false;
    private boolean prevDpadRight = false;
    private long lastDpadLeftMs = 0;
    private long lastDpadRightMs = 0;

    // Gate toggle
    private boolean gateOpen = false;
    private boolean prevY = false;

    // Auto-tune state
    private boolean prevA = false;
    private boolean tuning = false;
    private final ElapsedTime tuneTimer = new ElapsedTime();
    private double tuneStartRpm = 0.0;
    private double tuneTargetRpm = 0.0;
    private double lastSettleTimeMs = 0.0;
    private boolean wasAtTarget = false;
    private double peakOvershoot = 0.0;
    private int tuneIteration = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Drivetrain motors (unchanged)
        DcMotor frontLeftMotor = hardwareMap.dcMotor. get("frontleftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backleftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor. get("frontrightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor. get("backrightMotor");

        // Directions (match dualmotor style)
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction. REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction. FORWARD);
        backLeftMotor. setDirection(DcMotorSimple. Direction.REVERSE);

        intake = hardwareMap. get(DcMotorEx.class, "intakemotor");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior. BRAKE);

        // Shooter (flywheel) motor
        shooterMotor = hardwareMap.get(DcMotorEx. class, "shootermotor");
        shooterMotor. setDirection(DcMotorSimple. Direction.REVERSE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior. FLOAT);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        applyPidfFromConfig();

        // Servo gate
        servoGate = hardwareMap.get(Servo.class, "servogate");
        setGate(gateOpen); // start closed

        targetShooterRpm = CFG_DEFAULT_RPM;
        shooterEnabled = false;

        waitForStart();
        if (isStopRequested()) return;

        // Turn on shooter at start of teleop
        shooterEnabled = true;
        shooterMotor.setVelocity(rpmToTicksPerSecond(targetShooterRpm));

        while (opModeIsActive()) {
            long nowMs = System.currentTimeMillis();

            // --- Drive (unchanged) ---
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor. setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // Intake control
            intake.setVelocity(gamepad1.right_trigger * 1000);

            // Shooter controls
            boolean dpadDown = gamepad1.dpad_down;
            boolean dpadLeft = gamepad1.dpad_left;
            boolean dpadRight = gamepad1.dpad_right;

            // Toggle shooter on/off with dpad down (edge-triggered)
            if (dpadDown && !prevDpadDown) {
                shooterEnabled = !shooterEnabled;
                if (! shooterEnabled) {
                    shooterMotor.setPower(0);
                    tuning = false;
                } else {
                    shooterMotor. setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    applyPidfFromConfig();
                    shooterMotor.setVelocity(rpmToTicksPerSecond(targetShooterRpm));
                }
            }

            // Decrease RPM with dpad left (edge + debounce)
            if (dpadLeft && !prevDpadLeft && (nowMs - lastDpadLeftMs) >= DPAD_DEBOUNCE_MS) {
                targetShooterRpm = Math.max(SHOOTER_MIN_RPM, targetShooterRpm - CFG_RPM_INCREMENT);
                lastDpadLeftMs = nowMs;
            }

            // Increase RPM with dpad right (edge + debounce)
            if (dpadRight && !prevDpadRight && (nowMs - lastDpadRightMs) >= DPAD_DEBOUNCE_MS) {
                targetShooterRpm = Math.min(CFG_MAX_RPM, targetShooterRpm + CFG_RPM_INCREMENT);
                lastDpadRightMs = nowMs;
            }

            // Auto-tune:  press A to start a tuning cycle
            boolean aPressed = gamepad1.a;
            if (aPressed && !prevA && shooterEnabled) {
                startTuningCycle();
            }
            prevA = aPressed;

            // Run tuning logic if active
            if (tuning) {
                runTuningCycle();
            }

            // Apply velocity if enabled
            if (shooterEnabled) {
                applyPidfFromConfig();
                shooterMotor. setVelocity(rpmToTicksPerSecond(targetShooterRpm));

                double currentRpm = ticksPerSecondToRpm(shooterMotor.getVelocity());
                double rpmError = Math.abs(targetShooterRpm - currentRpm);

                // Rumble continuously while within tolerance
                if (rpmError <= CFG_RPM_TOLERANCE) {
                    gamepad1.rumble(1.0, 1.0, RUMBLE_DURATION_MS);
                }
            }

            // Gate toggle on Y
            boolean yPressed = gamepad1.y;
            if (yPressed && !prevY) {
                gateOpen = !gateOpen;
                setGate(gateOpen);
            }
            prevY = yPressed;

            // Save button states
            prevDpadDown = dpadDown;
            prevDpadLeft = dpadLeft;
            prevDpadRight = dpadRight;

            // Telemetry
            double currentRpmDisplay = ticksPerSecondToRpm(shooterMotor.getVelocity());
            double rpmErrorDisplay = targetShooterRpm - currentRpmDisplay;

            telemetry.addData("Shooter Enabled", shooterEnabled);
            telemetry.addData("Target RPM", targetShooterRpm);
            telemetry.addData("Current RPM", String.format("%.1f", currentRpmDisplay));
            telemetry.addData("RPM Error", String.format("%. 1f", rpmErrorDisplay));
            telemetry.addLine();
            telemetry.addData("kP", String.format("%.2f", CFG_kP));
            telemetry.addData("kI", String.format("%. 4f", CFG_kI));
            telemetry.addData("kD", String.format("%. 2f", CFG_kD));
            telemetry.addData("kF", String.format("%. 2f", CFG_kF));
            telemetry. addLine();
            telemetry.addData("Tuning", tuning ?  "ACTIVE (iter " + tuneIteration + ")" : "Press A to tune");
            telemetry. addData("Last Settle Time (ms)", String.format("%.0f", lastSettleTimeMs));
            telemetry.addData("Target Settle Time (ms)", String.format("%.0f", CFG_TARGET_SETTLE_TIME_MS));
            telemetry.addData("Last Overshoot", String.format("%. 1f RPM", peakOvershoot));
            telemetry.addData("Gate", gateOpen ?  "OPEN" :  "CLOSED");
            telemetry. update();
        }
    }

    private void startTuningCycle() {
        tuning = true;
        tuneIteration++;
        tuneStartRpm = ticksPerSecondToRpm(shooterMotor.getVelocity());

        // Set a new target that's different from current to measure response
        if (targetShooterRpm < CFG_MAX_RPM - 200) {
            tuneTargetRpm = targetShooterRpm + 200;
        } else {
            tuneTargetRpm = targetShooterRpm - 200;
        }

        // Temporarily change target to measure step response
        targetShooterRpm = tuneTargetRpm;

        tuneTimer.reset();
        wasAtTarget = false;
        peakOvershoot = 0.0;

        // Apply current PIDF
        applyPidfFromConfig();
        shooterMotor. setVelocity(rpmToTicksPerSecond(targetShooterRpm));
    }

    private void runTuningCycle() {
        double currentRpm = ticksPerSecondToRpm(shooterMotor.getVelocity());
        double error = targetShooterRpm - currentRpm;
        double absError = Math.abs(error);

        // Track overshoot
        if (tuneTargetRpm > tuneStartRpm) {
            // Speeding up:  overshoot is when current > target
            if (currentRpm > tuneTargetRpm) {
                peakOvershoot = Math.max(peakOvershoot, currentRpm - tuneTargetRpm);
            }
        } else {
            // Slowing down: overshoot is when current < target
            if (currentRpm < tuneTargetRpm) {
                peakOvershoot = Math.max(peakOvershoot, tuneTargetRpm - currentRpm);
            }
        }

        // Check if we've reached target
        if (absError <= CFG_RPM_TOLERANCE && !wasAtTarget) {
            wasAtTarget = true;
            lastSettleTimeMs = tuneTimer.milliseconds();

            // Apply auto-tune adjustments
            applyAutoTuneAdjustments();

            // End tuning cycle
            tuning = false;

            // Rumble to indicate tuning complete
            gamepad1.rumble(0.5, 0.5, 300);
        }

        // Timeout:  if it takes too long, adjust and end
        if (tuneTimer.milliseconds() > 5000 && ! wasAtTarget) {
            lastSettleTimeMs = tuneTimer.milliseconds();
            applyAutoTuneAdjustments();
            tuning = false;
            gamepad1.rumble(1.0, 0.0, 500); // left rumble only = timeout
        }
    }

    private void applyAutoTuneAdjustments() {
        double settleRatio = lastSettleTimeMs / CFG_TARGET_SETTLE_TIME_MS;

        // If settle time is too slow (ratio > 1), increase gains
        // If settle time is too fast with overshoot, decrease gains slightly
        // If settle time is good with no overshoot, we're done

        boolean hasOvershoot = peakOvershoot > CFG_RPM_TOLERANCE * 0.5;

        if (settleRatio > 1.5) {
            // Way too slow:  increase kP and kF significantly
            CFG_kP *= CFG_TUNE_STEP;
            CFG_kF *= CFG_TUNE_STEP;
            CFG_kI *= 1.2;
        } else if (settleRatio > 1.1) {
            // A bit slow: mild increase
            CFG_kP *= 1.2;
            CFG_kF *= 1.1;
        } else if (settleRatio < 0.8 && hasOvershoot) {
            // Fast but overshooting: reduce kP, increase kD
            CFG_kP *= 0.85;
            CFG_kD += 0.5;
            CFG_kI *= 0.9;
        } else if (hasOvershoot) {
            // Overshoot without being too fast: add damping
            CFG_kD += 0.3;
            CFG_kP *= 0.95;
        }
        // else: settle time is good and no significant overshoot, keep gains

        // Clamp values to reasonable ranges
        CFG_kP = clamp(CFG_kP, 1.0, 100.0);
        CFG_kI = clamp(CFG_kI, 0.0, 5.0);
        CFG_kD = clamp(CFG_kD, 0.0, 10.0);
        CFG_kF = clamp(CFG_kF, 1.0, 50.0);

        // Apply new PIDF
        applyPidfFromConfig();
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private void applyPidfFromConfig() {
        shooterMotor.setVelocityPIDFCoefficients(CFG_kP, CFG_kI, CFG_kD, CFG_kF);
    }

    private double rpmToTicksPerSecond(double rpm) {
        return rpm * CFG_TICKS_PER_REV / 60.0;
    }

    private double ticksPerSecondToRpm(double ticksPerSecond) {
        return ticksPerSecond * 60.0 / CFG_TICKS_PER_REV;
    }

    private void setGate(boolean open) {
        if (servoGate != null) {
            servoGate.setPosition(open ? GATE_OPEN :  GATE_CLOSED);
        }
    }
}