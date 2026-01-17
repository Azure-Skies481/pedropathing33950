package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;

@Configurable
public class ShootingHelp {
    private final double maxSpeed = 2800;
    @Sorter(sort = 0) public static double feedback = 0.009;

    public double getCurrentRPM(DcMotorEx motor){
        return (motor.getVelocity() * 60) / 28;
    }

    public double getPID(DcMotorEx motor, double power){
        double actualSpeed= motor.getVelocity();
        return feedback * (power-actualSpeed) + actualSpeed/maxSpeed;
    }



}
