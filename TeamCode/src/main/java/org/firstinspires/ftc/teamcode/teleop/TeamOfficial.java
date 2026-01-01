package org.firstinspires.ftc.teamcode. teleop;

import com.bylazar.configurables.annotations. Configurable;
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
    private Servo servoGate = null;

    // Runtime constants
    private static final double SHOOTER_MIN_RPM = 0.0;
    private static final long DPAD_DEBOUNCE_MS = 200;
    private static final double BOOST_ERROR_RPM = 200.0;
    private static final double BOOST_MULT = 1.15;

    // Servo gate positions
    private static final double GATE_OPEN = 0.1;
    private static final double GATE_CLOSED = 0.8;
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

        double maxSpeed = 2570;
        double feedback = 0.003;
        double targetShooterRPM = 650;

        // Directions (match dualmotor style)
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction. REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction. FORWARD);
        backLeftMotor. setDirection(DcMotorSimple. Direction.REVERSE);

        intake = hardwareMap. get(DcMotorEx.class, "intakemotor");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior. BRAKE);

        // Shooter (flywheel) motor
        DcMotorEx shooterMotor = hardwareMap.get(DcMotorEx.class, "shootermotor");
        shooterMotor. setDirection(DcMotorSimple. Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode. STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Servo gate
        servoGate = hardwareMap.get(Servo.class, "servogate");
        setGate(gateOpen);
        shooterEnabled = false;

        waitForStart();
        if (isStopRequested()) return;

        // Turn on shooter at start of teleop
        shooterEnabled = true;

        while (opModeIsActive()) {
            long nowMs = System.currentTimeMillis();

            // --- Drive (unchanged) ---
            double y = -gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y);
            double x = gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x);
            double rx = Math.pow(gamepad1.right_stick_x, 3.0);

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
                }
            }

            if (dpadLeft && !prevDpadLeft) {
                targetShooterRPM -= 50;
            }

            if (dpadRight && !prevDpadRight) {
                targetShooterRPM += 50;
            }

            double actual = -shooterMotor.getVelocity();

            shooterMotor.setPower(feedback*(targetShooterRPM-actual) + (actual/maxSpeed));


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

            // Telemetry
            double currentRpmDisplay = shooterMotor. getVelocity();
            telemetry.addData("Shooter Enabled", shooterEnabled);
            telemetry.addData("/nTarget RPM", targetShooterRPM);
            telemetry.addData("/nCurrent RPM", -Math.round(currentRpmDisplay * 10.0) / 10.0);
//            telemetry.addData("RPM Error", Math.round(rpmErrorDisplay * 10.0) / 10.0);
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

    private void setGate(boolean open) {
        if (servoGate != null) {
            servoGate.setPosition(open ? GATE_OPEN : GATE_CLOSED);
        }
    }
}