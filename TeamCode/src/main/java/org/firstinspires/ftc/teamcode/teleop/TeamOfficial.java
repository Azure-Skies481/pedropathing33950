package org.firstinspires.ftc.teamcode. teleop;

import com.bylazar.configurables.annotations. Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode. TeleOp;
import com.qualcomm. robotcore.hardware. DcMotor;
import com.qualcomm.robotcore. hardware.DcMotorEx;
import com. qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm. robotcore.hardware. Servo;
import com.qualcomm.robotcore.hardware. Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
        double targetShooterRPM = 900;

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

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

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

            if (gamepad1.options) {
                imu.resetYaw();
            }


            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double slowMode = 0.5;

            if (gamepad1.left_stick_button) {
                slowMode = 1;
            }

            double denominator = (Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1));
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(slowMode*frontLeftPower);
            backLeftMotor.setPower(slowMode*backLeftPower);
            frontRightMotor.setPower(slowMode*frontRightPower);
            backRightMotor.setPower(slowMode*backRightPower);

            if (gamepad1.right_bumper){
                frontLeftMotor.setPower(0.2);
                backLeftMotor.setPower(0.2);
                frontRightMotor.setPower(-0.2);
                backRightMotor.setPower(-0.2);
            }

            if (gamepad1.left_bumper){
                frontLeftMotor.setPower(-0.2);
                backLeftMotor.setPower(-0.2);
                frontRightMotor.setPower(0.2);
                backRightMotor.setPower(0.2);
            }

            // Intake control
            if (gamepad2.right_trigger > 0.8) {
                intake.setVelocity(1000);
            }

            if (gamepad2.left_trigger > 0.8) {
                intake.setVelocity(-1000);
            }

            intake.setVelocity(gamepad2.right_trigger * 1000);
            // Shooter controls
            boolean dpadDown = gamepad2.dpad_down;
            boolean dpadLeft = gamepad2.dpad_left;
            boolean dpadRight = gamepad2.dpad_right;

            // Toggle shooter on/off with dpad down (edge-triggered)
            if (gamepad2.dpadDownWasReleased()) {

                if (shooterEnabled){
                    shooterEnabled = false;
                }

                if (!shooterEnabled){
                    shooterEnabled = true;
                }

            }

            if (dpadLeft && !prevDpadLeft) {
                targetShooterRPM -= 50;
            }

            if (dpadRight && !prevDpadRight) {
                targetShooterRPM += 50;
            }

            double actual = -shooterMotor.getVelocity();

            if (shooterEnabled) {
                shooterMotor.setPower(feedback * (targetShooterRPM - actual) + (actual / maxSpeed));
            }
            if (!shooterEnabled){
                shooterMotor.setPower(0);
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
        gamepad2.rumble(1.0, 1.0, 10000);
    }

    private void stopRumble() {
        // Stop any ongoing rumble
        gamepad2.stopRumble();
    }

    private void setGate(boolean open) {
        if (servoGate != null) {
            servoGate.setPosition(open ? GATE_OPEN : GATE_CLOSED);
        }
    }
}