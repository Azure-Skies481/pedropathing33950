package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
@Configurable
public class IMUalign extends LinearOpMode {

    private DcMotorEx intake = null;
    private Servo servoGate = null;
    private IMU imu = null;

    // Runtime constants
    private static final double SHOOTER_MIN_RPM = 0.0;
    private static final long DPAD_DEBOUNCE_MS = 200;
    private static final double BOOST_ERROR_RPM = 200.0;
    private static final double BOOST_MULT = 1.15;

    // Servo gate positions
    private static final double GATE_OPEN = 0.9;
    private static final double GATE_CLOSED = 0.45;
    private boolean shooterEnabled = false;

    // Edge/debounce
    private boolean prevDpadDown = false;
    private boolean prevDpadLeft = false;
    private boolean prevDpadRight = false;
    private boolean prevY = false;

    // Heading align toggle
    private boolean aligningToStart = false;
    private boolean prevX = false;
    private double startHeadingDeg = 0.0;

    // Turn-to-heading gains (PD to reduce oscillation)
    private static final double ALIGN_KP = 0.005;
    private static final double ALIGN_KD = 0.5;
    private static final double ALIGN_MIN_PWR = 0.75;
    private static final double ALIGN_MAX_PWR = 1.0;
    private static final double ALIGN_TOL_DEG = 15.0;
    private static final double ALIGN_STOP_RATE_DPS = 8.0;

    private double prevErrorDeg = 0.0;
    private long prevTimeNanos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Drivetrain motors
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontleftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backleftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontrightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backrightMotor");

        double maxSpeed = 2570;
        double feedback = 0.003;
        double targetShooterRPM = 950;

        // Directions (from provided teleop)
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotorEx.class, "intakemotor");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Shooter (flywheel) motor
        DcMotorEx shooterMotor = hardwareMap.get(DcMotorEx.class, "shootermotor");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Servo gate
        servoGate = hardwareMap.get(Servo.class, "servogate");
        setGate(gateOpen);
        shooterEnabled = false;

        // IMU init (Expansion Hub IMU, logo LEFT, USB UP)
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        waitForStart();
        if (isStopRequested()) return;

        // Capture start heading and enable shooter
        startHeadingDeg = getHeadingDeg();
        aligningToStart = false;
        shooterEnabled = true;
        prevErrorDeg = 0.0;
        prevTimeNanos = System.nanoTime();

        while (opModeIsActive()) {
            long nowNanos = System.nanoTime();

            // --- Drive (with align override on rotation) ---
            double y = -gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y);
            double x = gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x);
            double rxDriver = Math.pow(gamepad1.right_stick_x, 3.0);

            if (gamepad1.options) {
                imu.resetYaw();
                startHeadingDeg = 0.0; // optional reset reference
            }

            double slowMode = gamepad1.right_trigger+0.5;
            // Align toggle on X
            boolean xPressed = gamepad1.x;
            if (xPressed && !prevX) {
                aligningToStart = !aligningToStart;
                if (aligningToStart) {
                    prevErrorDeg = angleWrapDeg(startHeadingDeg - getHeadingDeg());
                    prevTimeNanos = nowNanos;
                }
            }
            prevX = xPressed;

            double rx;
            if (aligningToStart) {
                double heading = getHeadingDeg();
                double error = angleWrapDeg(startHeadingDeg - heading);
                double dt = (nowNanos - prevTimeNanos) / 1e9;
                if (dt <= 0) dt = 1e-3;
                double dError = (error - prevErrorDeg) / dt;

                double turnCmd = ALIGN_KP * error + ALIGN_KD * dError;
                turnCmd = Range.clip(turnCmd, -ALIGN_MAX_PWR, ALIGN_MAX_PWR);
                if (Math.abs(turnCmd) < ALIGN_MIN_PWR) {
                    turnCmd = Math.copySign(ALIGN_MIN_PWR, turnCmd);
                }
                rx = turnCmd;

                boolean withinTol = Math.abs(error) <= ALIGN_TOL_DEG;
                boolean slowRate = Math.abs(dError) <= ALIGN_STOP_RATE_DPS;
                if (withinTol && slowRate) {
                    aligningToStart = false;
                    rx = 0.0;
                }

                prevErrorDeg = error;
                prevTimeNanos = nowNanos;
            } else {
                rx = rxDriver;
            }

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(slowMode * frontLeftPower);
            backLeftMotor.setPower(slowMode * backLeftPower);
            frontRightMotor.setPower(slowMode * frontRightPower);
            backRightMotor.setPower(slowMode * backRightPower);

            // Bumpers rotate in place (manual)
            if (gamepad1.right_bumper || gamepad2.right_bumper) {
                frontLeftMotor.setPower(0.2);
                backLeftMotor.setPower(0.2);
                frontRightMotor.setPower(-0.2);
                backRightMotor.setPower(-0.2);
            }
            if (gamepad1.left_bumper || gamepad2.left_bumper) {
                frontLeftMotor.setPower(-0.2);
                backLeftMotor.setPower(-0.2);
                frontRightMotor.setPower(0.2);
                backRightMotor.setPower(0.2);
            }

            // Intake control (as provided)
            if (gamepad2.right_trigger > 0.8) {
                intake.setVelocity(1000);
            }
            if (gamepad2.left_trigger > 0.8) {
                intake.setVelocity(-1000);
            }
            intake.setVelocity(gamepad2.right_trigger * 1000);

            // Shooter controls (gamepad2)
            boolean dpadDown = gamepad2.dpad_down;
            boolean dpadLeft = gamepad2.dpad_left;
            boolean dpadRight = gamepad2.dpad_right;

            if (dpadDown && !prevDpadDown) {
                shooterEnabled = !shooterEnabled;
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
            } else {
                shooterMotor.setPower(0);
            }

            // Gate toggle on Y (gamepad1)
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
            double currentRpmDisplay = shooterMotor.getVelocity();
            telemetry.addData("Shooter Enabled", shooterEnabled);
            telemetry.addData("Target RPM", targetShooterRPM);
            telemetry.addData("Current RPM", -Math.round(currentRpmDisplay * 10.0) / 10.0);
            telemetry.addData("Heading (deg)", getHeadingDeg());
            telemetry.addData("Start Heading (deg)", startHeadingDeg);
            telemetry.addData("Aligning to Start?", aligningToStart);
            telemetry.update();
        }
    }

    private double getHeadingDeg() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    private double angleWrapDeg(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    private void startContinuousRumble() {
        gamepad2.rumble(1.0, 1.0, 10000);
    }

    private void stopRumble() {
        gamepad2.stopRumble();
    }

    private void setGate(boolean open) {
        if (servoGate != null) {
            servoGate.setPosition(open ? GATE_OPEN : GATE_CLOSED);
        }
    }

    // State for gate
    private boolean gateOpen = false;
}