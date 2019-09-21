package OfflineClasses;


import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Spiessbach on 8/26/2018.
 */

public class DcMotor {
//    final double ROBOT_INCH_TO_MOTOR_DEG = 360 / (3.875 * Math.PI);// 360 deg. / wheel circumference (Wheel diameter x pi)
//    final int DEGREES_TO_COUNTS = 1440 / 360; //Counts per 1 revolution
//    double driveDistance = 24;//Target distance inches for robot motion in any omni direction for drive motors
//    int maxCounts = (int) Math.round(driveDistance*ROBOT_INCH_TO_MOTOR_DEG*DEGREES_TO_COUNTS);
    public double timeStep = 0.0;//determined a fixed time step so that faster speeds will show shorter time to distance
    public int fakePosition=0;
    public int targetPosition=0;
    public double motorPower =0;
    public double powerSign = 1;
    public double motorTol = 1;// tolerance value for instances of the motor for their speed
    public String targetMode = "No";
    public Boolean busyStatus = false;
    public void setPower(double power){
//        int deltaPosition;
        this.motorPower = Range.clip(power,-1,1);
//        deltaPosition = (int) Math.round(power*timeStep);
//        fakePosition += deltaPosition;
        this.targetMode = "No";
    }

    public double getPower(){

        return this.motorPower;
    }
//    public boolean isBusy(){
//
//        return this.busyStatus;
//    }
//    public void setPowerFloat(){
//
//    };
//    public DcMotorController getController(){
//        return this.FakeController;
//    };
    public int getCurrentPosition(){
        if(this.targetMode == "Yes"){
            int deltaPosition;
            if((this.fakePosition != this.targetPosition) && (this.motorPower != 0)) {
                deltaPosition = (int) Math.round(this.powerSign*this.motorPower * this.timeStep * this.motorTol);
                this.fakePosition += deltaPosition;
                if(this.powerSign < 0){
                    this.fakePosition = Range.clip(this.fakePosition,this.targetPosition,this.fakePosition);
                }
                if(this.powerSign >= 0){
                    this.fakePosition = Range.clip(this.fakePosition,this.fakePosition,this.targetPosition);
                }
            }
        }
        else {
            int deltaPosition;
            deltaPosition = (int) Math.round(this.motorPower*this.timeStep* this.motorTol);
            this.fakePosition += deltaPosition;
        }
        return this.fakePosition;
    }
    public void setTargetPosition(int position){
        this.targetPosition = position;
        this.targetMode = "Yes";
        int deltaPosition;
        if(this.fakePosition != this.targetPosition) {
            if(this.fakePosition < this.targetPosition) {
                this.powerSign = 1;
            }
            if(this.fakePosition > this.targetPosition) {
                this.powerSign = -1;
            }
            deltaPosition = (int) Math.round(this.powerSign*this.motorPower * this.timeStep* this.motorTol);
            this.fakePosition += deltaPosition;
        }
    }
    public enum ZeroPowerBehavior{
        UNKNOWN,
        BRAKE,
        FLOAT
    }

    public void setZeroPowerBehavior(ZeroPowerBehavior zpb){

    }

    public enum RunMode{
        RUN_WITHOUT_ENCODER,
        RUN_USING_ENCODER,
        RUN_TO_POSITION,
        STOP_AND_RESET_ENCODER
    }

    public void setMode(RunMode rm){

    }


    public void setDirection(DcMotorSimple.Direction Dir){

    };


}
