//package org.firstinspires.ftc.teamcode.teleop;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.Pose;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.VoltageSensor;
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.configurables.annotations.Sorter;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//import org.firstinspires.ftc.teamcode.subsystems.FlywheelModified;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//
//import java.util.concurrent.TimeUnit;
//
//@TeleOp
//@Configurable
//public class ExampleCode extends LinearOpMode {
//
//    private DcMotorEx intake = null;
//    public static Pose startPosition;
//    private Follower follower;
//    private Pose currentPosition;
//    private boolean aimed = true;
//
//    private DcMotorEx shooter = null;
//    private DcMotorEx shooter2 = null; // NEW: Second shooter motor
//    private Servo gate = null;
//    private VoltageSensor voltageSensor = null;
//
//    private double driveSpeed;
//    ShootingHelp shootingHelp = new ShootingHelp();
//
//    @Sorter(sort = 0)
//    public static double feedback = 0.0008; // not used anymore
//
//    private boolean shooterToggle = false;
//    private boolean shooterToggleWasPressed = false;
//
//    private boolean gateOpen = false;
//    private boolean gateToggleWasPressed = false;
//
//    private boolean fastMode = false;
//    private boolean fastModeWasPressed = false;
//
//    private boolean imuReferenceResetWasPressed = false;
//
//    private boolean dpadLeftWasPressed = false;
//    private boolean dpadRightWasPressed = false;
//
//    private double imuAlignAngle;
//
//    private DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
//
//    @Sorter(sort = 1)
//    public static int power = 500;  // DEFAULT RPM IS 2850
//
//    private IMU imu = null;
//    private final IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
//            RevHubOrientationOnRobot.UsbFacingDirection.UP));
//    ElapsedTime Timer = new ElapsedTime();
//
//    // Shooter control object with dual motors
//    private FlywheelModified flywheel;
//
//    // Rumble state check
//    private boolean wasVibratingLastLoop = false;
//
//    // IMU Align
//    public void getImuAlignAngle() {
//        imuAlignAngle = imu.getRobotYawPitchRollAngles().getYaw();
//    }
//
//    public void imuAlign() {
//        double timeout = 0.5;
//        ElapsedTime alignTimer = new ElapsedTime();
//        alignTimer.reset();
//
//        while (opModeIsActive()) {
//            double imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//            double error = imuAlignAngle - imuAngle;
//            double power = 0.02 * error;
//
//            frontLeftMotor.setPower(-power);
//            backLeftMotor.setPower(-power);
//            frontRightMotor.setPower(power);
//            backRightMotor.setPower(power);
//
//            double velocity = (frontLeftMotor.getVelocity() + backLeftMotor.getVelocity()
//                    + frontRightMotor.getVelocity() + backRightMotor.getVelocity()) / 4;
//
//            telemetry.addData("imu: ", imuAngle);
//            telemetry.addData("error: ", error);
//            telemetry.update();
//
//            if (Math.abs(error) <= 2.5 && Math.abs(velocity) <= 0.3) {
//                telemetry.addData("skibidi", "yes it's done yo");
//                telemetry.update();
//                break;
//            }
//            if (alignTimer.seconds() > timeout) {
//                telemetry.addData("imuAlign", "Timeout reached, aborting");
//                telemetry.update();
//                break;
//            }
//            idle();
//        }
//        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    }
//
//    // Launch System
//    public void launchSystem() {
//        stopMove();
//        Timer.reset();
//        double skibidi = Timer.time(TimeUnit.MILLISECONDS);
//
//        gate.setPosition(0.5);
//        intake.setPower(-0.85);
//        while (skibidi < 1000) {
//            skibidi = Timer.time(TimeUnit.MILLISECONDS);
//            flywheel.setTargetRPM(power);
//            flywheel.update(); // Keep updating during launch
//        }
//        intake.setPower(0);
//        gate.setPosition(0.45);
//    }
//
//    public void stopMove() {
//        frontLeftMotor.setVelocity(0);
//        backLeftMotor.setVelocity(0);
//        frontRightMotor.setVelocity(0);
//        backRightMotor.setVelocity(0);
//    }
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        //follower = Constants.createFollower(hardwareMap);
//        imu = hardwareMap.get(IMU.class, "imu");
//        imu.initialize(parameters);
//
//
//        frontLeftMotor = (DcMotorEx) hardwareMap.dcMotor.get("frontLeft");
//        backLeftMotor = (DcMotorEx) hardwareMap.dcMotor.get("backLeft");
//        frontRightMotor = (DcMotorEx) hardwareMap.dcMotor.get("frontRight");
//        backRightMotor = (DcMotorEx) hardwareMap.dcMotor.get("backRight");
//
//        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//
//        intake = hardwareMap.get(DcMotorEx.class, "intakeMotor");
//        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
//        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        // NEW: Initialize second shooter motor
//        try {
//            shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
//            telemetry.addData("Shooter2", "Initialized ✓");
//        } catch (IllegalArgumentException e) {
//            shooter2 = null;
//            telemetry.addData("Shooter2", "Not found (running single motor)");
//        }
//
//        gate = hardwareMap.get(Servo.class, "gateServo");
//
//        // Get voltage sensor
//        try {
//            voltageSensor = hardwareMap.voltageSensor.iterator().next();
//        } catch (Exception e) {
//            voltageSensor = null;
//        }
//
//        intake.setDirection(DcMotorSimple.Direction.REVERSE);
//        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
//
//        // NEW: Set shooter2 direction (opposite to shooter for counter-rotation)
//        if (shooter2 != null) {
//            shooter2.setDirection(DcMotorSimple.Direction.FORWARD); // Opposite direction
//        }
//
//        // NEW: Initialize FlywheelModified with both motors
//
//
//        imuAlignAngle = imu.getRobotYawPitchRollAngles().getYaw();
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.addData("Dual Motors", shooter2 != null ? "YES" : "NO");
//        telemetry.update();
//
//        waitForStart();
//        if (isStopRequested()) return;
//
//        double time;
//        while (opModeIsActive()) {
//            //currentPosition = follower.getPose();
//            time = Timer.time(TimeUnit.MILLISECONDS);
//
//            // Auto Launch System Check
//            if (gamepad1.y || gamepad2.y) {
//                launchSystem();
//            }
//
//            // Fast mode toggle (gamepad1 right bumper rising edge)
//            if (gamepad1.right_bumper) {
//                if (!fastModeWasPressed) {
//                    fastMode = !fastMode;
//                }
//                fastModeWasPressed = true;
//            } else {
//                fastModeWasPressed = false;
//            }
//
//            driveSpeed = fastMode ? 1.0 : 0.4;
//            driveSpeed += gamepad1.right_trigger * (fastMode ? 0.0 : 0.6);
//
//            // --- Drive ---
//            double y = -gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y);
//            double x = gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x);
//            double rx = Math.pow(gamepad1.right_stick_x, 3.0);
//
//            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//            double frontLeftPower = driveSpeed * ((y + x + rx) / denominator);
//            double backLeftPower = driveSpeed * ((y - x + rx) / denominator);
//            double frontRightPower = driveSpeed * ((y - x - rx) / denominator);
//            double backRightPower = driveSpeed * ((y + x - rx) / denominator);
//
//            frontLeftMotor.setPower(frontLeftPower);
//            backLeftMotor.setPower(backLeftPower);
//            frontRightMotor.setPower(frontRightPower);
//            backRightMotor.setPower(backRightPower);
//
//            // Changing shooter power
//            if (gamepad1.dpad_left || gamepad2.dpad_left) {
//                if (!dpadLeftWasPressed) {
//                    power = Math.max(0, power - 50);
//                }
//                dpadLeftWasPressed = true;
//            } else {
//                dpadLeftWasPressed = false;
//            }
//
//            if (gamepad1.dpad_right || gamepad2.dpad_right) {
//                if (!dpadRightWasPressed) {
//                    power = power + 50;
//                }
//                dpadRightWasPressed = true;
//            } else {
//                dpadRightWasPressed = false;
//            }
//
//            // Intake control
//            double intakePower = 0.0;
//            if (gamepad1.left_trigger > 0.05 || gamepad1.right_trigger > 0.05) {
//                intakePower = gamepad1.left_trigger * 0.85 - gamepad1.right_trigger;
//            } else {
//                intakePower = gamepad2.left_trigger * 0.85 - gamepad2.right_trigger * 1;
//            }
//            intake.setPower(intakePower);
//
//            // Toggle shooter on/off
//            if (gamepad1.dpad_down || gamepad2.dpad_down) {
//                if (!shooterToggleWasPressed) {
//                    shooterToggle = !shooterToggle;
//                    shootingHelp.getPID(shooter, 0);
//                }
//                shooterToggleWasPressed = true;
//            } else {
//                shooterToggleWasPressed = false;
//            }
//
//            // Gate toggle
//            if (gamepad1.b || gamepad2.b) {
//                if (!gateToggleWasPressed) {
//                    gateOpen = !gateOpen;
//                }
//                gateToggleWasPressed = true;
//            } else {
//                gateToggleWasPressed = false;
//            }
//
//            // Reset IMU reference point
//            if (gamepad1.a || gamepad2.a) {
//                if (!imuReferenceResetWasPressed) {
//                    imuAlignAngle = imu.getRobotYawPitchRollAngles().getYaw();
//                }
//                imuReferenceResetWasPressed = true;
//            } else {
//                imuReferenceResetWasPressed = false;
//            }
//
//            // Gate position logic
//            gate.setPosition(gateOpen ? 0.5 : 0.45);
//
//            // Update PIDF controller
//            shootingHelp.getPID(shooter, power);
//
//            // IMU Align
//            if (gamepad1.x) {
//                imuAlign();
//            }
//            //aimed = Math.abs(currentPosition.getHeading()-45)<15 || Math.abs(currentPosition.getHeading()-135)<15;
//
//            // Telemetry
//            //telemetry.addData("Aimed? ", aimed ? "✓" : "✗");
//            telemetry.addData("Shooter RPM", "%.0f", flywheel.getCurrentRPM());
//            telemetry.addData("Target RPM", "%.0f", flywheel.getTargetRPM());
//            telemetry.addData("Power", "%.2f", flywheel.getLastAppliedPower());
//
//            telemetry.addData("At Speed", flywheel.isAtTarget() ? "✓" : "✗");
//            telemetry.addData("Gate Open", gateOpen);
//            telemetry.addData("Shooter On", flywheel.isShooterOn());
//            telemetry.addData("Intake Power", "%.2f", intakePower);
//            telemetry.addData("Fast Mode", fastMode);
//            telemetry.addData("Dual Motors", shooter2 != null ? "YES" : "NO");
//            telemetry.addData("Reference Heading", "%.1f°", Math.toDegrees(imuAlignAngle));
//            telemetry.update();
//        }
//    }
//}