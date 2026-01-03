/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Robot: Auto Drive By Encoder", group="Robot")
public class AutoTesting extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotorEx         frontleftMotor   = null;
    private DcMotorEx         frontrightMotor  = null;
    private DcMotorEx         backleftMotor   = null;
    private DcMotorEx         backrightMotor  = null;
    private DcMotorEx intake = null;
    private Servo servoGate = null;
    double maxSpeed = 2570;
    double feedback = 0.003;
    double targetShooterRPM = 950;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        frontleftMotor = hardwareMap.get(DcMotorEx.class, "frontleftMotor");
        frontrightMotor = hardwareMap.get(DcMotorEx.class, "frontrightMotor");
        backleftMotor = hardwareMap.get(DcMotorEx.class, "backleftMotor");
        backrightMotor = hardwareMap.get(DcMotorEx.class, "backrightMotor");
        DcMotorEx shooterMotor = hardwareMap.get(DcMotorEx.class, "shootermotor");
        servoGate = hardwareMap.get(Servo.class, "servogate");
        intake = hardwareMap. get(DcMotorEx.class, "intakemotor");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontleftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backleftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        frontrightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backrightMotor.setDirection(DcMotorEx.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        frontleftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontrightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backrightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backleftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode. STOP_AND_RESET_ENCODER);


        frontleftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontrightMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backrightMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backleftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d :%7d :%7d",
                frontrightMotor.getCurrentPosition(),
                frontleftMotor.getCurrentPosition(),
                backleftMotor.getCurrentPosition(),
                backrightMotor.getCurrentPosition());

        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        while (opModeIsActive()){
            double actual = -shooterMotor.getVelocity();
            shooterMotor.setPower(feedback * (targetShooterRPM - actual) + (actual / maxSpeed));
            if (targetShooterRPM - actual <= 75){
                break;
            }
        }
        intake.setVelocity(1000);
        servoGate.setPosition(0.0);
        moveStraight(-2000);
        servoGate.setPosition(0.5);
        sleep(2000);
        Strafing(100);
        sleep(1000);  // pause to display final telemetry message.

        frontleftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontrightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backrightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backrightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void moveStraight(double desiredPosition) {
        resetMotors();
        while (opModeIsActive()) {

            int currentPosition = (-frontleftMotor.getCurrentPosition() + frontrightMotor.getCurrentPosition() + backleftMotor.getCurrentPosition() - backrightMotor.getCurrentPosition()) / 4;

            double difference = desiredPosition - currentPosition;
            double power = 0.01 * difference;

            frontleftMotor.setPower(power);
            frontrightMotor.setPower(power);
            backleftMotor.setPower(power);
            backrightMotor.setPower(power);

            telemetry.addData("Current Position: ", currentPosition);
            telemetry.addData("Difference: ", difference);
            telemetry.addData("Power: ",power);
            telemetry.addData("Positions:",  "%7d :%7d :%7d :%7d",
                    frontrightMotor.getCurrentPosition(),
                    frontleftMotor.getCurrentPosition(),
                    backleftMotor.getCurrentPosition(),
                    backrightMotor.getCurrentPosition());
            telemetry.update();

            double currentVelocity = (frontleftMotor.getVelocity() + frontrightMotor.getVelocity() + backleftMotor.getVelocity() + backrightMotor.getVelocity())/4;
            telemetry.addData("current Velocity: ", currentVelocity);
            if ((Math.abs(currentVelocity) <= 1) && (Math.abs(difference) <= 5)) {
                break;
            }
        }
    }
    public void Strafing (double desiredStrafe) {
        resetMotors();

        while (opModeIsActive()) {
            int currentPosition = (frontleftMotor.getCurrentPosition() - frontrightMotor.getCurrentPosition() + backleftMotor.getCurrentPosition() + backrightMotor.getCurrentPosition()) / 4;

            double difference = desiredStrafe - currentPosition;
            double power = 0.01 * difference;

            frontleftMotor.setPower(-power);
            frontrightMotor.setPower(power);
            backleftMotor.setPower(power);
            backrightMotor.setPower(-power);

            telemetry.addData("Current Position: ", currentPosition);
            telemetry.addData("Difference: ", difference);
            telemetry.addData("Power: ",power);
            telemetry.addData("Positions:",  "%7d :%7d :%7d :%7d",
                    frontrightMotor.getCurrentPosition(),
                    frontleftMotor.getCurrentPosition(),
                    backleftMotor.getCurrentPosition(),
                    backrightMotor.getCurrentPosition());
            telemetry.update();

            double currentVelocity = (frontleftMotor.getVelocity() + frontrightMotor.getVelocity() + backleftMotor.getVelocity() + backrightMotor.getVelocity())/4;
            telemetry.addData("current Velocity: ", currentVelocity);
            if ((Math.abs(currentVelocity) <= 1) && (Math.abs(difference) <= 5)) {
                telemetry.addData("It's","done");
                break;
            }
        }
    }

    public void resetMotors() {
        frontleftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontrightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backrightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backleftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        frontleftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontrightMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backrightMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backleftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
}
