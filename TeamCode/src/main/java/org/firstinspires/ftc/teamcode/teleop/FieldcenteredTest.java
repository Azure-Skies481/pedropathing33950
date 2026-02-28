//package org.firstinspires.ftc.teamcode.teleop;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.Range;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//
//@TeleOp
//public class FieldcenteredTest extends LinearOpMode {
//
//
//    private DcMotorEx intake = null;
//
//    private DcMotorEx reverseintake = null;
//    private DcMotorEx shooter = null;
//    private Servo gate = null; //new servo js added
//    private double feedback = 0.001;
//    boolean wasPressedLastFrame = false;
//    boolean gateWasPressedLastFrame = false; //Satoru...
//    boolean gateOpen = false; //Suguru...
//    boolean shooterToggle = false;
//    double power = 1400;
//    double drivespeed;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//
//        // Declare our motors
//        // Make sure your ID's match your configuration
//        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontleftMotor");
//        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backleftMotor");
//        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontrightMotor");
//        DcMotor backRightMotor = hardwareMap.dcMotor.get("backrightMotor");
//        // Reverse the right side motors. This may be wrong for your setup.
//        // If your robot moves backwards when commanded to go forwards,
//        // reverse the left side instead.
//        // See the note about this earlier on this page.
//        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//
//        intake = hardwareMap.get(DcMotorEx.class, "intakemotor");
//        reverseintake = hardwareMap.get(DcMotorEx.class, "intakemotor");
//
//        shooter = hardwareMap.get(DcMotorEx.class, "shootermotor");
//        gate = hardwareMap.get(Servo.class, "gateServo");
//
//        intake.setDirection(DcMotorSimple.Direction.REVERSE);
//        reverseintake.setDirection(DcMotorSimple.Direction.REVERSE);
//        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
//        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        // Retrieve the IMU from the hardware map
//        IMU imu = hardwareMap.get(IMU.class, "imu");
//        // Adjust the orientation parameters to match your robot
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
//                RevHubOrientationOnRobot.UsbFacingDirection.UP));
//        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
//        imu.initialize(parameters);
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        while (opModeIsActive()) {
//            drivespeed = 0.4;
//            drivespeed += 0.6 * gamepad1.right_trigger;
//
//            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
//            double x = gamepad1.left_stick_x;
//            double rx = gamepad1.right_stick_x;
//
//            // This button choice was made so that it is hard to hit on accident,
//            // it can be freely changed based on preference.
//            // The equivalent button is start on Xbox-style controllers.
//            if (gamepad1.options) {
//                imu.resetYaw();
//            }
//
//            double actualspeed = shooter.getVelocity();
//            if (gamepad2.dpad_up){
//                power = 1400;
//            } else if (gamepad2.dpad_left){
//                power = 1250;
//            } else if (gamepad2.dpad_down){
//                power = 1150;
//            }
//
//            if (gamepad2.right_bumper) {
//                gateWasPressedLastFrame = true;
//            } else {
//                if (gateWasPressedLastFrame) {
//                    gateOpen = !gateOpen;
//                }
//                gateWasPressedLastFrame = false;
//            }
//            if (gateOpen) {
//                //open
//                gate.setPosition(0.5);
//            } else {
//                //close
//                gate.setPosition(0.0);
//            }
//
//            if (gamepad2.left_bumper) {
//                wasPressedLastFrame = true;
//            } else {
//                if (wasPressedLastFrame) {
//                    shooterToggle = !shooterToggle;
//                }
//                wasPressedLastFrame = false;
//            }
//            if (shooterToggle) {
//
//                double maxspeed = 2800;
//                this.shooter.setPower(feedback * (power - actualspeed) + actualspeed/ maxspeed);
//            } else {
//                this.shooter.setVelocity(0);
//            }
//
//
//            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//
//            // Rotate the movement direction counter to the bot's rotation
//            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
//            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
//
//            rotX = rotX * 1.1;  // Counteract imperfect strafing
//
//            // Denominator is the largest motor power (absolute value) or 1
//            // This ensures all the powers maintain the same ratio,
//            // but only if at least one is out of the range [-1, 1]
//            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
//            double frontLeftPower = drivespeed*((rotY + rotX + rx) / denominator);
//            double backLeftPower = drivespeed*((rotY - rotX + rx) / denominator);
//            double frontRightPower = drivespeed*((rotY - rotX - rx) / denominator);
//            double backRightPower = drivespeed*((rotY + rotX - rx) / denominator);
//
//            frontLeftMotor.setPower(frontLeftPower);
//            backLeftMotor.setPower(backLeftPower);
//            frontRightMotor.setPower(frontRightPower);
//            backRightMotor.setPower(backRightPower);
//
//            reverseintake.setVelocity(gamepad2.left_trigger * 1500);
//            intake.setVelocity(gamepad2.right_trigger * -1500);
//            telemetry.addData("Shooter Real Velocity", shooter.getVelocity());
//            telemetry.addData("Shooter Target Velocity", power);
//            telemetry.addData("Gate Open?", gateOpen);
//            telemetry.update();
//
//
//        }
//    }
//}