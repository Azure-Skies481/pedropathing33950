package org.firstinspires.ftc.teamcode.autonomous;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class AutoClose extends LinearOpMode{

    private DcMotorEx intake = null;

    private DcMotorEx reverseintake = null;
    private DcMotorEx shooter = null;
    private Servo gate = null; //new servo js added
    private double maxspeed = 2800;
    private double feedback = 0.001;
    boolean wasPressedLastFrame = false;
    boolean gateWasPressedLastFrame = false; //Satoru...
    boolean gateOpen = false; //Suguru...
    boolean shooterToggle = false;
    double power = 1550;
    double drivespeed;

    boolean aura = true;

    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;

    //IMU imu = hardwareMap.get(IMU.class, "imu");
    //IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
    //RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
    //RevHubOrientationOnRobot.UsbFacingDirection.UP));
    public void moveForward(double amount) {
        while (opModeIsActive()) {
            double position = (double) (frontLeftMotor.getCurrentPosition() + backLeftMotor.getCurrentPosition() +
                    frontRightMotor.getCurrentPosition() + backRightMotor.getCurrentPosition()) / 4;
            double error = amount - position;
            double power = 0.01 * error;
            frontLeftMotor.setPower(power);
            frontRightMotor.setPower(power);
            backLeftMotor.setPower(power);
            backRightMotor.setPower(power);
            if (Math.abs(error) <= 15) break;
        }
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
    public void strafe(double amount, boolean left) {
        while (opModeIsActive()){
            double position = (double) (frontLeftMotor.getCurrentPosition() + backLeftMotor.getCurrentPosition() +
                    frontRightMotor.getCurrentPosition() + backRightMotor.getCurrentPosition()) / 4;
            double error = amount - position;
            double power = 0.01 * error;
            int skibidi = -1;
            if (left) skibidi = skibidi * -1;
            frontLeftMotor.setPower(power * skibidi);
            frontRightMotor.setPower(power * -skibidi);
            backLeftMotor.setPower(power * -skibidi);
            backRightMotor.setPower(power * skibidi);
        }
    }
    /*

    public void turn (double angle){
        while (opModeIsActive()){
            double imuAngle = imu.getRobotYawPitchRollAngles().getYaw();
            double error = angle - imuAngle;
            double power = 0.01*error;
            frontLeftMotor.setPower(power);
            backLeftMotor.setPower(power);
            frontRightMotor.setPower(power);
            backRightMotor.setPower(power);
        }
    }

     */


    @Override
    public void runOpMode() throws InterruptedException {
        //imu.initialize(parameters);
        frontLeftMotor = hardwareMap.dcMotor.get("frontleftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backleftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontrightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backrightMotor");

        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


         */
        frontLeftMotor.setTargetPosition(0);
        frontRightMotor.setTargetPosition(0);
        backLeftMotor.setTargetPosition(0);
        backRightMotor.setTargetPosition(0);



        intake = hardwareMap.get(DcMotorEx.class, "intakemotor");

        shooter = hardwareMap.get(DcMotorEx.class, "shootermotor");
        gate = hardwareMap.get(Servo.class, "gateServo"); //new servo js added

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        if (isStopRequested()) return;
        telemetry.addData("skibidi", 1);
        telemetry.update();
        moveForward(300);
        telemetry.addData("a", 2);
        telemetry.update();
        Thread.sleep(700);
        shooter.setVelocity(1200);
        telemetry.addData("b", 3);
        telemetry.update();
        Thread.sleep(3000);
        gate.setPosition(0.5);
        telemetry.addData("c", 4);
        telemetry.update();
        intake.setPower(1.0);
        telemetry.addData("d", 5);
        telemetry.update();
        Thread.sleep(10000);
        shooter.setPower(0);
        intake.setPower(0);
        telemetry.addData("e", 6);
        telemetry.update();
    }
}
