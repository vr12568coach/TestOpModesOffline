package OfflineCode.OfflineHW;


import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Spiessbach on 12/22/20.
 * Offline HW class that replicates a Continuous Servo
 */

public class CRServo {


    public double servoPower =0;
    public double timeStep = 0.0;
    public double powerSign = 1.0;

    public void setPower(double power){
        this.servoPower = powerSign * Range.clip(power,-1,1);
    }

    public double getPower(){

        return this.servoPower;
    }
    public void setDirection(DcMotorSimple.Direction Dir){
        switch (Dir) {
            case FORWARD:
                this.powerSign = 1.0;
                break;
            case REVERSE:
                this.powerSign = -1.0;
                break;
        }
    }



}
