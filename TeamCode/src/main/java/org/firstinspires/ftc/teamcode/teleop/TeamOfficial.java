package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TeamOfficial extends LinearOpMode {

    private DcMotorEx intake = null;
    private DcMotorEx shooterMotor = null;

    // Shooter (flywheel) configuration
    // Encoder spec: 28 PPR at output shaft, quadrature -> 28 * 4 = 112 counts per motor rev.
    private static final double SHOOTER_TICKS_PER_REV = 112.0;
    private static final double SHOOTER_MAX_RPM = 6000.0;   // theoretical max
    private static final double SHOOTER_MIN_RPM = 0.0;
    private static final double SHOOTER_DEFAULT_RPM = 4000.0; // tested no-load speed
    private static final double SHOOTER_RPM_INCREMENT = 50.0; // adjust to taste

    // Rumble when at speed
    private static final double RPM_TOLERANCE = 100.0; // ±100 RPM window
    private static final int RUMBLE_DURATION_MS = 250;
    private static final int RUMBLE_COOLDOWN_MS = 500;
    private long lastRumbleMs = 0;

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

        // Shooter (flywheel) motor
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shootermotor");
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // enables built-in velocity PID

        // Shooter state
        double targetShooterRpm = SHOOTER_DEFAULT_RPM;
        boolean shooterEnabled = false;

        // Button edge tracking for toggles/increments
        boolean prevDpadDown = false;
        boolean prevDpadLeft = false;
        boolean prevDpadRight = false;

        waitForStart();
        if (isStopRequested()) return;

        // Turn on shooter at start of teleop (not during init)
        shooterEnabled = true;
        shooterMotor.setVelocity(rpmToTicksPerSecond(targetShooterRpm));

        while (opModeIsActive()) {
            long nowMs = System.currentTimeMillis();

            // --- Drive (unchanged) ---
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
                    shooterMotor.setVelocity(rpmToTicksPerSecond(targetShooterRpm));
                }
            }

            // Decrease RPM with dpad left (edge-triggered)
            if (dpadLeft && !prevDpadLeft) {
                targetShooterRpm = Math.max(SHOOTER_MIN_RPM, targetShooterRpm - SHOOTER_RPM_INCREMENT);
            }

            // Increase RPM with dpad right (edge-triggered)
            if (dpadRight && !prevDpadRight) {
                targetShooterRpm = Math.min(SHOOTER_MAX_RPM, targetShooterRpm + SHOOTER_RPM_INCREMENT);
            }

            // Apply velocity if enabled
            if (shooterEnabled) {
                shooterMotor.setVelocity(rpmToTicksPerSecond(targetShooterRpm));
            }

            // Rumble when within ±100 RPM of target (with cooldown)
            if (shooterEnabled) {
                double currentRpm = ticksPerSecondToRpm(shooterMotor.getVelocity());
                double rpmError = Math.abs(targetShooterRpm - currentRpm);
                if (rpmError <= RPM_TOLERANCE && (nowMs - lastRumbleMs) >= RUMBLE_COOLDOWN_MS) {
                    gamepad1.rumble(0.7, 0.7, RUMBLE_DURATION_MS);
                    lastRumbleMs = nowMs;
                }
            }

            // Save button states
            prevDpadDown = dpadDown;
            prevDpadLeft = dpadLeft;
            prevDpadRight = dpadRight;

            // Telemetry
            telemetry.addData("Shooter Enabled", shooterEnabled);
            telemetry.addData("Target RPM", targetShooterRpm);
            telemetry.addData("Current Velocity (ticks/s)", shooterMotor.getVelocity());
            telemetry.addData("Current RPM", ticksPerSecondToRpm(shooterMotor.getVelocity()));
            telemetry.update();
        }
    }

    private double rpmToTicksPerSecond(double rpm) {
        return rpm * SHOOTER_TICKS_PER_REV / 60.0;
    }

    private double ticksPerSecondToRpm(double ticksPerSecond) {
        return ticksPerSecond * 60.0 / SHOOTER_TICKS_PER_REV;
    }
}