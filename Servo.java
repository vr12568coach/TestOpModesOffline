package TestOpModesOffline;

/**
 * Created by Spiessbach on 8/26/2018.
 */

public class Servo {

    public double timeStep = 0.0;//determined a fixed time step so that faster speeds will show shorter time to distance
    public double fakeServoPosition =0;
    public double targetPosition=0;
    public double servoPower = 1;

    public double getPosition(){
        return fakeServoPosition;
    }
    public void setPosition(double position){
        targetPosition = position;
        double deltaPosition;
        if(fakeServoPosition != targetPosition) {
            deltaPosition = (double) Math.round(servoPower * timeStep);
            fakeServoPosition += deltaPosition;
        }
    }


}
