package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Configurable
public class TeamOfficial extends LinearOpMode {

    private DcMotorEx intake = null;
    private DcMotorEx shooterMotor = null;

    // Panels-configurable values (tuned live via Panels/Sorter)
    @Sorter(sort = 0) public static double CFG_MAX_RPM = 6000.0;
    @Sorter(sort = 1) public static double CFG_TICKS_PER_REV = 112.0;   // 28 PPR * 4
    @Sorter(sort = 2) public static double CFG_DEFAULT_RPM = 800.0;
    @Sorter(sort = 3) public static double CFG_RPM_INCREMENT = 50.0;
    @Sorter(sort = 4) public static double CFG_RPM_TOLERANCE = 100.0;
    @Sorter(sort = 5) public static double CFG_kP = 6.0;
    @Sorter(sort = 6) public static double CFG_kI = 0.0;
    @Sorter(sort = 7) public static double CFG_kD = 1.0;
    @Sorter(sort = 8) public static double CFG_kF = 2.93; // recomputed each loop from CFG_MAX_RPM, but exposed for override

    // Runtime constants
    private static final double SHOOTER_MIN_RPM = 0.0;
    private static final int RUMBLE_DURATION_MS = 200;
    private static final long DPAD_DEBOUNCE_MS = 200;

    // State
    private double targetShooterRpm = CFG_DEFAULT_RPM;
    private boolean shooterEnabled = false;

    // Edge/debounce
    private boolean prevDpadDown = false;
    private boolean prevDpadLeft = false;
    private boolean prevDpadRight = false;
    private long lastDpadLeftMs = 0;
    private long lastDpadRightMs = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Drivetrain motors (unchanged)
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontleftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backleftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontrightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backrightMotor");

        // Directions (match dualmotor style)
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotorEx.class, "intakemotor");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Shooter (flywheel) motor
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shootermotor");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE); // reversed per earlier request
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Apply initial PIDF (will be refreshed each loop from config)
        applyPidfFromConfig();

        targetShooterRpm = CFG_DEFAULT_RPM;
        shooterEnabled = false;

        waitForStart();
        if (isStopRequested()) return;

        // Turn on shooter at start of teleop
        shooterEnabled = true;
        shooterMotor.setVelocity(rpmToTicksPerSecond(targetShooterRpm));

        while (opModeIsActive()) {
            long nowMs = System.currentTimeMillis();

            // Drive (unchanged)
            double y = -gamepad1.left_stick_y; // Y is reversed
            double x = gamepad1.left_stick_x;  // Strafing
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
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
                if (!shooterEnabled) {
                    shooterMotor.setPower(0);
                } else {
                    shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

            // Apply velocity if enabled (keeps trying to meet target)
            if (shooterEnabled) {
                applyPidfFromConfig(); // pull live panel values
                shooterMotor.setVelocity(rpmToTicksPerSecond(targetShooterRpm));
            }

            // Continuous rumble when within tolerance
            if (shooterEnabled) {
                double currentRpm = ticksPerSecondToRpm(shooterMotor.getVelocity());
                double rpmError = Math.abs(targetShooterRpm - currentRpm);
                if (rpmError <= CFG_RPM_TOLERANCE) {
                    gamepad1.rumble(1.0, 1.0, RUMBLE_DURATION_MS);
                }
            }

            // Save button states
            prevDpadDown = dpadDown;
            prevDpadLeft = dpadLeft;
            prevDpadRight = dpadRight;

            // Telemetry
            telemetry.addData("Shooter Enabled", shooterEnabled);
            telemetry.addData("Target RPM", targetShooterRpm);
            telemetry.addData("Current RPM", ticksPerSecondToRpm(shooterMotor.getVelocity()));
            telemetry.addData("RPM Error", Math.abs(targetShooterRpm - ticksPerSecondToRpm(shooterMotor.getVelocity())));
            telemetry.addData("kP/kI/kD/kF", "%.3f / %.3f / %.3f / %.3f", CFG_kP, CFG_kI, CFG_kD, CFG_kF);
            telemetry.update();
        }
    }

    private static double computeKF(double maxRpm) {
        double maxTicksPerSec = maxRpm * CFG_TICKS_PER_REV / 60.0;
        return (maxTicksPerSec > 1e-6) ? (32767.0 / maxTicksPerSec) : 0.0;
    }

    private void applyPidfFromConfig() {
        double kf = (CFG_kF > 0) ? CFG_kF : computeKF(CFG_MAX_RPM);
        shooterMotor.setVelocityPIDFCoefficients(CFG_kP, CFG_kI, CFG_kD, kf);
    }

    private double rpmToTicksPerSecond(double rpm) {
        return rpm * CFG_TICKS_PER_REV / 60.0;
    }

    private double ticksPerSecondToRpm(double ticksPerSecond) {
        return ticksPerSecond * 60.0 / CFG_TICKS_PER_REV;
    }
}