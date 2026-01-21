package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelModified;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.concurrent.TimeUnit;

@TeleOp
@Configurable
public class HORSreplica extends LinearOpMode {

    private DcMotorEx intake = null;
    private DcMotorEx shooter = null;
    private Servo gate = null;
    private double driveSpeed;
    @Sorter(sort = 0)
    public static double feedback = 0.0008; // not used anymore

    private boolean shooterToggle = false;
    private boolean shooterToggleWasPressed = false;

    private boolean gateOpen = false;
    private boolean gateToggleWasPressed = false;

    private boolean fastMode = false;
    private boolean fastModeWasPressed = false;

    private boolean imuReferenceResetWasPressed = false;

    private boolean dpadLeftWasPressed = false;
    private boolean dpadRightWasPressed = false;

    private double imuAlignAngle;

    private DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    @Sorter(sort = 1)
    public static int power = 2850;  // DEFAULT RPM IS 2600 NOW

    private IMU imu = null;
    private final IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
            RevHubOrientationOnRobot.UsbFacingDirection.UP));
    ElapsedTime Timer = new ElapsedTime();

    private FlywheelModified flywheel;

    private boolean wasVibratingLastLoop = false;

    public void getImuAlignAngle() {
        imuAlignAngle = imu.getRobotYawPitchRollAngles().getYaw();
    }

    // Improved IMU align function with simple braking (P controller + brake when done)
    // Will brake and hold the robot at heading after reaching target
    public void imuAlign() {
        final double kP = 0.02;    // Proportional gain
        final double kD = 0.003;   // Derivative gain for damping/anti-overshoot (play with this value)
        final double timeout = 1.5; // seconds
        final double minPower = 0.07; // minimum power to overcome static friction
        final double angleTolerance = 1.5; // degrees
        final double velocityTolerance = 0.2; // encoder ticks/sec

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        double lastError = 0;
        double lastTime = timer.seconds();

        while (opModeIsActive()) {
            double imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double error = imuAlignAngle - imuAngle;
            double currentTime = timer.seconds();
            double deltaTime = currentTime - lastTime;
            double derivative = 0;
            if (deltaTime > 0) {
                derivative = (error - lastError) / deltaTime;
            }

            double turnPower = kP * error + kD * derivative;

            // Clamp power for very small error (to prevent micro-oscillations)
            if (Math.abs(turnPower) < minPower && Math.abs(error) > angleTolerance) {
                turnPower = Math.signum(turnPower) * minPower;
            }
            if (Math.abs(error) < angleTolerance) {
                turnPower = 0;
            }

            frontLeftMotor.setPower(-turnPower);
            backLeftMotor.setPower(-turnPower);
            frontRightMotor.setPower(turnPower);
            backRightMotor.setPower(turnPower);

            // Get average velocity for braking test
            double velocity = (Math.abs(frontLeftMotor.getVelocity()) + Math.abs(backLeftMotor.getVelocity())
                    + Math.abs(frontRightMotor.getVelocity()) + Math.abs(backRightMotor.getVelocity())) / 4;

            telemetry.addData("IMU Angle", imuAngle);
            telemetry.addData("Target Angle", imuAlignAngle);
            telemetry.addData("Error", error);
            telemetry.addData("Turn Power", turnPower);
            telemetry.addData("Velocity", velocity);
            telemetry.update();

            // Check if in angle tolerance and then if the robot is stopped
            if (Math.abs(error) < angleTolerance && velocity < velocityTolerance) {
                break;
            }
            if (timer.seconds() > timeout) {
                telemetry.addData("IMU Align", "Timeout reached");
                telemetry.update();
                break;
            }
            lastError = error;
            lastTime = currentTime;
            idle();
        }
        // Brake robot and hold heading
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        sleep(150); //so that we can briefly stop before overloading controller
    }

    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);
        frontLeftMotor = (DcMotorEx) hardwareMap.dcMotor.get("frontleftMotor");
        backLeftMotor = (DcMotorEx) hardwareMap.dcMotor.get("backleftMotor");
        frontRightMotor = (DcMotorEx) hardwareMap.dcMotor.get("frontrightMotor");
        backRightMotor = (DcMotorEx) hardwareMap.dcMotor.get("backrightMotor");

        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotorEx.class, "intakemotor");
        shooter = hardwareMap.get(DcMotorEx.class, "shootermotor");
        gate = hardwareMap.get(Servo.class, "gateServo");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheel = new FlywheelModified(shooter, telemetry);
        flywheel.setTargetRPM(2600);

        imuAlignAngle = imu.getRobotYawPitchRollAngles().getYaw();
        waitForStart();
        if (isStopRequested()) return;

        double time;
        while (opModeIsActive()) {
            time = Timer.time(TimeUnit.MILLISECONDS);

            // Fast mode toggle (gamepad1 right bumper rising edge)
            if (gamepad1.right_bumper) {
                if (!fastModeWasPressed) {
                    fastMode = !fastMode;
                }
                fastModeWasPressed = true;
            } else {
                fastModeWasPressed = false;
            }

            driveSpeed = fastMode ? 1.0 : 0.4;
            driveSpeed += gamepad1.right_trigger * (fastMode ? 0.0 : 0.6);

            // --- Drive ---
            double y = -gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y);
            double x = gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x);
            double rx = Math.pow(gamepad1.right_stick_x, 3.0);

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = driveSpeed * ((y + x + rx) / denominator);
            double backLeftPower = driveSpeed * ((y - x + rx) / denominator);
            double frontRightPower = driveSpeed * ((y - x - rx) / denominator);
            double backRightPower = driveSpeed * ((y + x - rx) / denominator);

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // ---- GAMEPAD1: Shooter RPM increment/decrement ----
            if (gamepad1.dpad_left || gamepad2.dpad_left) {
                if (!dpadLeftWasPressed) {
                    power = Math.max(0, power - 50);
                    flywheel.setTargetRPM(power);
                }
                dpadLeftWasPressed = true;
            } else {
                dpadLeftWasPressed = false;
            }
            if (gamepad1.dpad_right || gamepad2.dpad_right) {
                if (!dpadRightWasPressed) {
                    power = power + 50;
                    flywheel.setTargetRPM(power);
                }
                dpadRightWasPressed = true;
            } else {
                dpadRightWasPressed = false;
            }

            // --- Intake control ---
            double intakePower = 0.0;
            if (gamepad1.left_trigger > 0.05 || gamepad1.right_trigger > 0.05) {
                intakePower = gamepad1.left_trigger - gamepad1.right_trigger;
            } else {
                intakePower = gamepad2.left_trigger - gamepad2.right_trigger * 0.85;
            }
            intake.setPower(intakePower);

            // Toggle shooter
            if (gamepad1.dpad_down || gamepad2.dpad_down) {
                if (!shooterToggleWasPressed) {
                    shooterToggle = !shooterToggle;
                    flywheel.setShooterOn(shooterToggle);
                }
                shooterToggleWasPressed = true;
            } else {
                shooterToggleWasPressed = false;
            }

            // Gate toggle
            if (gamepad1.b || gamepad2.b) {
                if (!gateToggleWasPressed) {
                    gateOpen = !gateOpen;
                }
                gateToggleWasPressed = true;
            } else {
                gateToggleWasPressed = false;
            }

            // Reset IMU reference heading
            if (gamepad1.a || gamepad2.a) {
                if (!imuReferenceResetWasPressed) {
                    imuAlignAngle = imu.getRobotYawPitchRollAngles().getYaw();
                }
                imuReferenceResetWasPressed = true;
            } else {
                imuReferenceResetWasPressed = false;
            }

            // Shooter and Gate
            gate.setPosition(gateOpen ? 0.5 : 0.36);

            // Shooter logic
            flywheel.update();

            // -------- CONTROLLER RUMBLE LOGIC --------
            double targetRpm = flywheel.getTargetRPM();
            double currentRpm = flywheel.getCurrentRPM();
            boolean withinTolerance = Math.abs(targetRpm - currentRpm) <= 50.0;

            if (flywheel.isShooterOn() && !withinTolerance) {
                gamepad1.rumble(0.5, 0.5, 100);
                gamepad2.rumble(0.5, 0.5, 100);
                wasVibratingLastLoop = true;
            } else if (wasVibratingLastLoop) {
                gamepad1.stopRumble();
                gamepad2.stopRumble();
                wasVibratingLastLoop = false;
            }

            // IMU Align (gamepad1.x rising edge)
            if (gamepad1.x) {
                imuAlign();
            }

            telemetry.addData("Shooter RPM", flywheel.getCurrentRPM());
            telemetry.addData("Shooter Target RPM", flywheel.getTargetRPM());
            telemetry.addData("Gate Open", gateOpen);
            telemetry.addData("Shooter On", flywheel.isShooterOn());
            telemetry.addData("Intake Power", intakePower);
            telemetry.addData("Fast Mode", fastMode);
            telemetry.addData("Reference Heading", imuAlignAngle);
            telemetry.update();
        }
    }
}