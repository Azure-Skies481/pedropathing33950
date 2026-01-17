package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teleop.ShootingHelp;


@TeleOp
@Configurable
public class MecanumTeleop extends LinearOpMode {
    ShootingHelp shootingHelp = new ShootingHelp();

    private DcMotorEx intake = null;
    private DcMotorEx shooter = null;
    private Servo gate = null; //new servo js added
    private double driveSpeed;
    private double maxspeed = 2800;
    @Sorter(sort = 0) public static double feedback = 0.0008;

    private boolean wasPressedLastFrame = false;

    private boolean gateWasPressedLastFrame = false; //satoru...

    private boolean gateOpen = false; //suguru...
    private boolean shooterToggle = false;
    private boolean intakeUsedLastFrame = false;
    private double imuAlignAngle;

    private DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    @Sorter(sort = 1) public static int power = 1400;
    private IMU imu = null;
    private final IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
            RevHubOrientationOnRobot.UsbFacingDirection.UP));

    public void getImuAlignAngle(){
        imuAlignAngle=imu.getRobotYawPitchRollAngles().getYaw();
    }

    public void imuAlign(){
        while (opModeIsActive()){
            double imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double error = imuAlignAngle - imuAngle;
            double power = 0.02*error;
            frontLeftMotor.setPower(-power);
            backLeftMotor.setPower(-power);
            frontRightMotor.setPower(power);
            backRightMotor.setPower(power);
            double velocity = (frontLeftMotor.getVelocity() + backLeftMotor.getVelocity() + frontRightMotor.getVelocity() + backRightMotor.getVelocity())/4;

            telemetry.addData("imu: ", imuAngle);
            telemetry.addData("error: ", error);
            telemetry.update();
            if (Math.abs(error) <= 2.5 && velocity<=0.3){
                telemetry.addData("skibidi", "yes it's done yo");
                telemetry.update();
                break;
            }
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {
        // Drivetrain motors (unchanged)

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);
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

        shooter = hardwareMap.get(DcMotorEx.class, "shootermotor");
        gate = hardwareMap.get(Servo.class, "gateServo"); //new servo js added



        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imuAlignAngle=imu.getRobotYawPitchRollAngles().getYaw();

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            long nowMs = System.currentTimeMillis();
            driveSpeed = 0.4;

            driveSpeed += gamepad1.right_trigger*0.6;


            // --- Drive (unchanged) ---
            double y = -gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y);
            double x = gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x);
            double rx = Math.pow(gamepad1.right_stick_x, 3.0);

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = driveSpeed*((y + x + rx) / denominator);
            double backLeftPower = driveSpeed*((y - x + rx) / denominator);
            double frontRightPower = driveSpeed*((y - x - rx) / denominator);
            double backRightPower = driveSpeed*((y + x - rx) / denominator);

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            double actualspeed = shooter.getVelocity();



            if (gamepad2.dpad_up){
                power = 1450;
            } else if (gamepad2.dpad_left){
                power = 1350;
            } else if (gamepad2.dpad_down){
                power = 1250;
            } else if (gamepad2.dpad_right) power = 1650;

            if (intakeUsedLastFrame && gamepad2.right_trigger==0 && gateOpen) gate.setPosition(0.0);

            if (gamepad2.right_bumper) {
                gateWasPressedLastFrame = true;
            } else {
                if (gateWasPressedLastFrame) {
                    gateOpen = !gateOpen;
                }
                gateWasPressedLastFrame = false;
            }

            if (gamepad1.yWasPressed()){
                power += 20;
            }
            if (gamepad1.aWasPressed()){
                power -=20;
            }
            if (gamepad1.bWasPressed()){
                getImuAlignAngle();
            }

            if (gamepad1.xWasPressed()){
                imuAlign();
            }

            if (gateOpen) {
                //open
                gate.setPosition(0.5);
            } else {
                //close
                gate.setPosition(0.0);
            }
            //gate.setPosition(0.5);
            if (intakeUsedLastFrame && gamepad2.right_trigger == 0) {
                gateOpen = false;
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

                this.shooter.setPower((shootingHelp.getPID(shooter, power)));
            } else {
                this.shooter.setVelocity(0);
            }

            intake.setPower(gamepad2.right_trigger * -0.5 + gamepad2.left_trigger);
            intakeUsedLastFrame = gamepad2.right_trigger > 0;
            telemetry.addData("Shooter Real Velocity", shooter.getVelocity());
            telemetry.addData("Shooter Target Velocity", power);
            telemetry.addData("Gate Open?", gateOpen);
            telemetry.update();
        }
    }
}