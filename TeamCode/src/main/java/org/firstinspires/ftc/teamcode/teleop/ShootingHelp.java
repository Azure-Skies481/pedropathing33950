package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.teleop.MecanumTeleop.feedback;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ShootingHelp {
    double maxSpeed = 2800;
    double feedback = 0.008;

    public double getCurrentRPM(DcMotorEx motor){
        return (motor.getVelocity() * 60) / 28;
    }

    public double getPID(DcMotorEx motor, double power){
        double actualspeed= motor.getVelocity();
        return feedback * (power-actualspeed) + actualspeed/maxSpeed;
    }


}
