package OfflineCode.OfflineHW;


import com.qualcomm.robotcore.util.Range;

/**
 * Created by Spiessbach on 12/22/20.
 * Offline HW class that replicates a Continuous Servo
 */

public class CRServo {


    public double servoPower =0;
    public double timeStep = 0.0;

    public void setPower(double power){
        this.servoPower = Range.clip(power,-1,1);
    }

    public double getPower(){

        return this.servoPower;
    }




}
