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
public class AutoFar extends LinearOpMode{

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

    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;

    IMU imu = hardwareMap.get(IMU.class, "imu");
    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
            RevHubOrientationOnRobot.UsbFacingDirection.UP));
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
        }
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
            backLeftMotor.setPower(power * - skibidi);
            backRightMotor.setPower(power * skibidi);
        }
    }

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


    @Override
    public void runOpMode() throws InterruptedException {
        imu.initialize(parameters);
        frontLeftMotor = hardwareMap.dcMotor.get("frontleftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backleftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontrightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backrightMotor");

        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotorEx.class, "intakemotor");

        shooter = hardwareMap.get(DcMotorEx.class, "shootermotor");
        gate = hardwareMap.get(Servo.class, "gateServo"); //new servo js added

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        reverseintake.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) shooter.setVelocity(1650);

        //gate.setPosition(0.5);

        moveForward(100);
        //turn(30);
        //while (opModeIsActive()) intake.setPower(0.5);
    }
}
