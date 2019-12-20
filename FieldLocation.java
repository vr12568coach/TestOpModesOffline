package TestOpModesOffline;

public class FieldLocation {
    public double x = 0;
    public double y = 0;
    public double rotx = 0;
    public double roty = 0;
    public double deltaX = 0;
    public double deltaY = 0;
    public double theta = 0;
    public double thetaOffset = 0;
    public boolean heldByRobot = false;
    public boolean priorHold = false;
    public FieldLocation(double x, double y, double theta){
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.rotx = x;
        this.roty = y;
        this.thetaOffset = 0;
        this.heldByRobot = false;
        this.priorHold = false;

    }
    public FieldLocation(double x, double y, double rx, double ry,double theta, double thetaOffset){
        this.x = x;
        this.y = y;
        this.rotx = rx;
        this.roty = ry;
        this.theta = theta;
        this.thetaOffset = thetaOffset;

    }
    public FieldLocation(double x, double y, double rx, double ry,double theta, double thetaOffset, boolean held, boolean priorHold){
        this.x = x;
        this.y = y;
        this.rotx = rx;
        this.roty = ry;
        this.theta = theta;
        this.thetaOffset = thetaOffset;
        this.heldByRobot = held;
        this.priorHold = priorHold;

    }
    public void setHold(boolean held){
        this.priorHold = this.heldByRobot;
        this.heldByRobot = held;

    }
    public void setPriorHold(boolean ph){
        this.priorHold = ph;

    }
    public void setLocation(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public void setRotCenter(double rx, double ry, double thetaOffset) {
        this.rotx = rx;
        this.roty = ry;
        this.thetaOffset = thetaOffset;

    }
    public void rotationMatrix(){
                //calculate new location of point based on angle rotation and distance for rotation center
        this.x = this.rotx + this.deltaX*Math.cos(this.theta) - this.deltaY*Math.sin(this.theta);
        this.y = this.roty + this.deltaX*Math.sin(this.theta) + this.deltaY*Math.cos(this.theta);

    }
}