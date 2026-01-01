package org.firstinspires.ftc.teamcode. teleop;

import com.bylazar.configurables.annotations. Configurable;
import com.bylazar.configurables. annotations. Sorter;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode. TeleOp;
import com.qualcomm. robotcore.hardware. DcMotor;
import com.qualcomm.robotcore. hardware.DcMotorEx;
import com. qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm. robotcore.hardware. Servo;
import com.qualcomm.robotcore.hardware. Gamepad;

@TeleOp
@Configurable
public class TeamOfficial extends LinearOpMode {

    private DcMotorEx intake = null;
    private DcMotorEx shooterMotor = null;
    private Servo servoGate = null;

    // Panels-configurable values (tuned live via Panels/Sorter)
    @Sorter(sort = 0) public static double CFG_MAX_RPM = 1400.0;
    @Sorter(sort = 1) public static double CFG_TICKS_PER_REV = 112.0;   // 28 PPR * 4
    @Sorter(sort = 2) public static double CFG_DEFAULT_RPM = 800.0;
    @Sorter(sort = 3) public static double CFG_RPM_INCREMENT = 50.0;
    @Sorter(sort = 4) public static double CFG_RPM_TOLERANCE = 100.0;
    @Sorter(sort = 5) public static double CFG_kP = 4.0;
    @Sorter(sort = 6) public static double CFG_kI = 0.0005;
    @Sorter(sort = 7) public static double CFG_kD = 0.1;
    @Sorter(sort = 8) public static double CFG_kF = 5.0;

    // Runtime constants
    private static final double SHOOTER_MIN_RPM = 0.0;
    private static final long DPAD_DEBOUNCE_MS = 200;
    private static final double BOOST_ERROR_RPM = 200.0;
    private static final double BOOST_MULT = 1.15;

    // Servo gate positions
    private static final double GATE_OPEN = 0.1;
    private static final double GATE_CLOSED = 0.8;

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

    // Rumble state
    private boolean wasAtTarget = false;
    private boolean rumbleActive = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Drivetrain motors (unchanged)
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontleftMotor");
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
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior. BRAKE);
        shooterMotor.setMode(DcMotor.RunMode. STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        applyPidfFromConfig(CFG_kF);

        // Servo gate
        servoGate = hardwareMap.get(Servo.class, "servogate");
        setGate(gateOpen);

        targetShooterRpm = CFG_DEFAULT_RPM;
        shooterEnabled = false;

        waitForStart();
        if (isStopRequested()) return;

        // Turn on shooter at start of teleop
        shooterEnabled = true;
        shooterMotor. setVelocity(rpmToTicksPerSecond(targetShooterRpm));

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
                    stopRumble();
                } else {
                    shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    applyPidfFromConfig(CFG_kF);
                    shooterMotor. setVelocity(rpmToTicksPerSecond(targetShooterRpm));
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

            // Apply velocity if enabled
            boolean isAtTarget = false;
            if (shooterEnabled) {
                double currentRpm = ticksPerSecondToRpm(shooterMotor.getVelocity());
                double rpmError = Math.abs(targetShooterRpm - currentRpm);

                // Boost kF when far from target to speed up acceleration
                double kfToUse = CFG_kF;
                if (rpmError > BOOST_ERROR_RPM) {
                    kfToUse = Math.min(CFG_kF * BOOST_MULT, CFG_kF * 1.5);
                }

                applyPidfFromConfig(kfToUse);
                shooterMotor.setVelocity(rpmToTicksPerSecond(targetShooterRpm));

                // Check if at target
                isAtTarget = rpmError <= CFG_RPM_TOLERANCE;
            }

            // Handle rumble state changes
            if (shooterEnabled && isAtTarget) {
                if (! rumbleActive) {
                    // Start continuous rumble using rumbleBlips or long duration
                    startContinuousRumble();
                    rumbleActive = true;
                }
            } else {
                if (rumbleActive) {
                    stopRumble();
                    rumbleActive = false;
                }
            }

            // Gate toggle on Y
            boolean yPressed = gamepad1.y;
            if (yPressed && ! prevY) {
                gateOpen = !gateOpen;
                setGate(gateOpen);
            }
            prevY = yPressed;

            // Save button states
            prevDpadDown = dpadDown;
            prevDpadLeft = dpadLeft;
            prevDpadRight = dpadRight;
            wasAtTarget = isAtTarget;

            // Telemetry
            double currentRpmDisplay = ticksPerSecondToRpm(shooterMotor. getVelocity());
            double rpmErrorDisplay = Math.abs(targetShooterRpm - currentRpmDisplay);

            telemetry.addData("Shooter Enabled", shooterEnabled);
            telemetry.addData("/nTarget RPM", targetShooterRpm);
            telemetry.addData("/nCurrent RPM", Math.round(currentRpmDisplay * 10.0) / 10.0);
//            telemetry.addData("RPM Error", Math.round(rpmErrorDisplay * 10.0) / 10.0);
            telemetry. addData("At Target", isAtTarget ?  "YES - RUMBLE" : "NO");
//            telemetry.addData("kP", Math.round(CFG_kP * 100.0) / 100.0);
//            telemetry. addData("kI", Math.round(CFG_kI * 10000.0) / 10000.0);
//            telemetry.addData("kD", Math.round(CFG_kD * 100.0) / 100.0);
//            telemetry.addData("kF", Math.round(CFG_kF * 100.0) / 100.0);
//            telemetry. addData("Gate", gateOpen ?  "OPEN" :  "CLOSED");
            telemetry. update();
        }
    }

    private void startContinuousRumble() {
        // Use a long rumble duration (10 seconds) - will be stopped when leaving tolerance
        // This avoids the issue of short rumbles getting lost
        gamepad1.rumble(1.0, 1.0, 10000);
    }

    private void stopRumble() {
        // Stop any ongoing rumble
        gamepad1.stopRumble();
    }

    private static double computeKF(double maxRpm) {
        double maxTicksPerSec = maxRpm * CFG_TICKS_PER_REV / 60.0;
        return (maxTicksPerSec > 1e-6) ? (32767.0 / maxTicksPerSec) : 0.0;
    }

    private void applyPidfFromConfig(double kfOverride) {
        double kf = (kfOverride > 0) ? kfOverride :  computeKF(CFG_MAX_RPM);
        shooterMotor. setVelocityPIDFCoefficients(CFG_kP, CFG_kI, CFG_kD, kf);
    }

    private double rpmToTicksPerSecond(double rpm) {
        return rpm * CFG_TICKS_PER_REV / 60.0;
    }

    private double ticksPerSecondToRpm(double ticksPerSecond) {
        return ticksPerSecond * 60.0 / CFG_TICKS_PER_REV;
    }

    private void setGate(boolean open) {
        if (servoGate != null) {
            servoGate.setPosition(open ? GATE_OPEN : GATE_CLOSED);
        }
    }
}