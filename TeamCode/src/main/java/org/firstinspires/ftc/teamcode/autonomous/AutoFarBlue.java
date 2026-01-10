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
public class AutoFarBlue extends LinearOpMode {
    private DcMotorEx intake = null;
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

    DcMotorEx frontLeftMotor;
    DcMotorEx backLeftMotor;
    DcMotorEx frontRightMotor;
    DcMotorEx backRightMotor;

    IMU imu = null;
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
            double velocity = (frontLeftMotor.getVelocity() + backLeftMotor.getVelocity() + frontRightMotor.getVelocity() + backRightMotor.getVelocity())/4;
            if (Math.abs(error) <= 15 && velocity <= 0.3) break;
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


    public void turn (double angle){
        imu.initialize(parameters);
        imu.resetYaw();
        while (opModeIsActive()){
            double imuAngle = imu.getRobotYawPitchRollAngles().getYaw();
            double error = angle - imuAngle;
            double power = 0.02*error;
            frontLeftMotor.setPower(-power);
            backLeftMotor.setPower(-power);
            frontRightMotor.setPower(power);
            backRightMotor.setPower(power);
            double velocity = (frontLeftMotor.getVelocity() + backLeftMotor.getVelocity() + frontRightMotor.getVelocity() + backRightMotor.getVelocity())/4;
            telemetry.addData("imu: ", imuAngle);
            telemetry.addData("error: ", error);
            telemetry.update();
            if (error <= 1 && velocity<=0.3) break;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontleftMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backleftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontrightMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backrightMotor");

        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setTargetPosition(0);
        frontRightMotor.setTargetPosition(0);
        backLeftMotor.setTargetPosition(0);
        backRightMotor.setTargetPosition(0);

        intake = hardwareMap.get(DcMotorEx.class, "intakemotor");

        shooter = hardwareMap.get(DcMotorEx.class, "shootermotor");
        gate = hardwareMap.get(Servo.class, "gateServo"); //new servo js added

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        if (isStopRequested()) return;
        //while (opModeIsActive()) shooter.setVelocity(1650);

        //gate.setPosition(0.5);
        shooter.setVelocity(1625);
        Thread.sleep(1500);
        gate.setPosition(0.5);
        Thread.sleep(1000);
        intake.setVelocity(-500);
        Thread.sleep(1000);
        intake.setVelocity(0);
        Thread.sleep(2000);
        intake.setVelocity(-500);
        Thread.sleep(1500);
        intake.setVelocity(0);


        /*
        if (shooter.getVelocity() <= shooterPower-25){
            intake.setVelocity(0);
            Thread.sleep(shootTwoPause);
            if (shooter.getVelocity() >= shooterPower){
                intake.setVelocity(-500);
            }
            else{
                Thread.sleep(1000);
                intake.setVelocity(-500);
            }
        }
         */

//        Thread.sleep(1000);
//        intake.setVelocity(-500);
//        Thread.sleep(3000);
        shooter.setVelocity(0);
        moveForward(-350);
        //turn(30);
        //while (opModeIsActive()) intake.setPower(0.5);
    }}