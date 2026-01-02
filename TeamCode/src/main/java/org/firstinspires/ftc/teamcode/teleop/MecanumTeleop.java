package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class MecanumTeleop extends LinearOpMode {

    private DcMotorEx intake = null;

    private DcMotorEx reverseintake = null;
    private DcMotorEx shooter = null;
    private Servo gate = null; //new servo js added

    double actualspeed;

    double driveSpeed;

    private double maxspeed = 2800;
    private double feedback = 0.001;

    private double targetspeed = 1000;


    boolean wasPressedLastFrame = false;

    boolean gateWasPressedLastFrame = false;

    boolean gateOpen = false; //satoru ... suguru...
    boolean shooterToggle = false;

    double power = 1000;

    @Override
    public void runOpMode() throws InterruptedException {
        // Drivetrain motors (unchanged)
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontleftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backleftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontrightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backrightMotor");

        // Directions (match dualmotor style)
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);// start at CLOSE target

        intake = hardwareMap.get(DcMotorEx.class, "intakemotor");
        reverseintake = hardwareMap.get(DcMotorEx.class, "intakemotor");

        shooter = hardwareMap.get(DcMotorEx.class, "shootermotor");
        gate = hardwareMap.get(Servo.class, "gateServo"); //new servo js added



        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        reverseintake.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            long nowMs = System.currentTimeMillis();
            driveSpeed = 0.3;

            driveSpeed += gamepad1.right_trigger*0.7;


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
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            double actualspeed = shooter.getVelocity();

            if (gamepad2.dpad_up){
                power = power + 2;
            } else if (gamepad2.dpad_down){
                power = power - 2;
            }

            if (gamepad2.right_bumper) {
                gateWasPressedLastFrame = true;
            } else {
                if (gateWasPressedLastFrame) {
                    gateOpen = !gateOpen;
                }
                gateWasPressedLastFrame = false;
            }
            if (gateOpen) {
                //open
                gate.setPosition(0.15);
            } else {
                //close
                gate.setPosition(0.0);
            }


            if (gamepad2.left_bumper) {
                wasPressedLastFrame = true;
            } else {
                if (wasPressedLastFrame) {
                    shooterToggle = !shooterToggle;
                }
                wasPressedLastFrame = false;
            }
            if (shooterToggle) {

                this.shooter.setPower(driveSpeed*(feedback * (power - actualspeed) + actualspeed/maxspeed));
            } else {
                this.shooter.setVelocity(0);
            }

            reverseintake.setVelocity(gamepad2.left_trigger * 1500);
            intake.setVelocity(gamepad2.right_trigger * -1500);
            telemetry.addData("Shooter Real Velocity", shooter.getVelocity());
            telemetry.addData("Shooter Target Velocity", power);
            telemetry.addData("Gate Open?", gateOpen);
            telemetry.update();
        }
    }
}