package OfflineCode.Field;

import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.ArrayList;

import OfflineCode.OfflineHW.DcMotor;
import UltimateGoal_RobotTeam.HarwareConfig.WobbleArm;
import UltimateGoal_RobotTeam.OpModes.Autonomous.BasicAuto;
import UltimateGoal_RobotTeam.Utilities.FieldLocation;

/** CLASS IS USED TO SET SKYSTONE FIELD
 * ADAPT FOR ULTIMATE GOAL FOR WOBBLE GOAL & RINGS - CLASS OBJECT IS "the field"
 * COMMENTED OUT PORTION OF CODE FOR VARIABLES NOT IN OPMODE
 */

public class FieldConfiguration {
    public ArrayList<FieldLocation> RedRingPoints =new ArrayList();
    public ArrayList<FieldLocation> BlueRingPoints =new ArrayList();
    public ArrayList<FieldLocation> BlueWobble1Points =new ArrayList();
    public ArrayList<FieldLocation> RedWobble1Points =new ArrayList();
    public ArrayList<FieldLocation> BlueWobble2Points =new ArrayList();
    public ArrayList<FieldLocation> RedWobble2Points =new ArrayList();
    public ArrayList<FieldLocation> PursuitPoints =new ArrayList();
    public ArrayList<FieldLocation> NavPoints =new ArrayList();
    public ArrayList<FieldLocation> RobotPoints =new ArrayList();

    private String ringType = "ring";
//    public ArrayList<FieldLocation> GripperPoints =new ArrayList();
//    public ArrayList<FieldLocation> RobotPoints =new ArrayList();

//    Define Red Single Ring initial position - default for sim is on field
    public FieldLocation redRing = new FieldLocation(36, -23, 0);
    //Define Blue Ring Stack initial position - default for sim is on field
    public FieldLocation blueRing = new FieldLocation(-36, -23, 0);

    //    Define Red Ring Stack initial position - default for sim is off field
    public FieldLocation redRingStack = new FieldLocation(100, -100, 0);
    //Define Blue Ring Stack initial position - default for sim is off field
    public FieldLocation blueRingStack = new FieldLocation(-100, -100, 0);

    //Define Blue Wobble Goal 1 initial position
    public FieldLocation blueWobble1 = new FieldLocation(-50, -48, 0);

    //Define Blue Wobble Goal 2 initial position
    public FieldLocation blueWobble2 = new FieldLocation(-26, -48, 0);

    //Define Red Wobble Goal ` initial position
    public FieldLocation redWobble1 = new FieldLocation(26, -48, 0);

    //Define Red Wobble Goal 2 initial position
    public FieldLocation redWobble2 = new FieldLocation(50, -48, 0);

    public boolean ringFound = false;

    public FieldConfiguration(){
        // Blank constructor
    }
    public FieldConfiguration(int ringSet){
        // constructor to set ring type in 0, 1, 4

        //    Define Ring Configurations position based on ringSet input 0 = none on field, 1 = single

        // Would it be simpler to pass a variable that defines what image to use in the Viz code vs. track 2 items?
        if(ringSet ==0) {
            ringType = "none";
            //Define Red Single Ring initial position - off field
            redRing = new FieldLocation(100, -100, 0);
            //Define Blue Ring Stack initial position - off field
            blueRing = new FieldLocation(-100, -100, 0);

        }
        else {
            //Define Red Single Ring initial position
            redRing = new FieldLocation(36, -23, 0);
            //Define Blue Ring Stack initial position
            blueRing = new FieldLocation(-36, -23, 0);
            if(ringSet ==1) {
                ringType = "ring";
            }
            else {
                ringType = "ringStack";
            }
        }
    }

public void updateField(BasicAuto opMode) {
            //Wobble Goal and Ring calculations

    redRing.setHold(opMode.haveRedRing);
    blueRing.setHold(opMode.haveBlueRing);
    // Is having the rings held necessary?
    // Does the ring stack need to be added?

    blueWobble1.setHold(opMode.haveBlueWobble1);
    blueWobble2.setHold(opMode.haveBlueWobble2);
    redWobble1.setHold(opMode.haveRedWobble1);
    redWobble2.setHold(opMode.haveRedWobble2);
    opMode.telemetry.addLine("--------- Field Configuration ---------");
    if(redRing.heldByRobot){
        redRing = updateGameItem(redRing,opMode.robotUG.driveTrain.imu.robotOnField);
    }
    if(blueRing.heldByRobot){
        blueRing = updateGameItem(blueRing,opMode.robotUG.driveTrain.imu.robotOnField);
    }
    if(blueWobble1.heldByRobot){

        blueWobble1 = updateWobbleGoal(blueWobble1,opMode.robotUG.driveTrain.imu.robotOnField, opMode.robotUG.wobbleArm, opMode);
    }
    if(blueWobble2.heldByRobot){
        blueWobble2 = updateWobbleGoal(blueWobble2,opMode.robotUG.driveTrain.imu.robotOnField, opMode.robotUG.wobbleArm, opMode);
    }
    if(redWobble1.heldByRobot){
        redWobble1 = updateWobbleGoal(redWobble1,opMode.robotUG.driveTrain.imu.robotOnField, opMode.robotUG.wobbleArm, opMode);
    }
    if(redWobble2.heldByRobot){
        redWobble2 = updateWobbleGoal(redWobble2,opMode.robotUG.driveTrain.imu.robotOnField, opMode.robotUG.wobbleArm, opMode);
    }

    ringFound = seeRing(blueRing, blueRingStack, redRing, redRingStack,opMode.robotUG.driveTrain.imu.robotOnField);

    RedRingPoints.add(new FieldLocation(redRing.x, redRing.y, redRing.theta));
    BlueRingPoints.add(new FieldLocation(blueRing.x,blueRing.y,blueRing.theta));

    BlueWobble1Points.add(new FieldLocation(blueWobble1.x, blueWobble1.y, blueWobble1.theta));
    BlueWobble2Points.add(new FieldLocation(blueWobble2.x, blueWobble2.y, blueWobble2.theta));

    RedWobble1Points.add(new FieldLocation(redWobble1.x, redWobble1.y, redWobble1.theta));
    RedWobble2Points.add(new FieldLocation(redWobble2.x, redWobble2.y, redWobble2.theta));

    PursuitPoints.add(new FieldLocation(opMode.robotUG.driveTrain.targetPoint.x,opMode.robotUG.driveTrain.targetPoint.y,0));

    RobotPoints.add(new FieldLocation(opMode.robotUG.driveTrain.imu.robotOnField.x,opMode.robotUG.driveTrain.imu.robotOnField.y,opMode.robotUG.driveTrain.imu.robotOnField.theta));// ADDED this to change points for field

    NavPoints.add(new FieldLocation(opMode.robotUG.driveTrain.robotFieldLocation.x,opMode.robotUG.driveTrain.robotFieldLocation.y,0));
    opMode.telemetry.addLine("------------- END -------------");
    opMode.telemetry.update();

}

    private FieldLocation updateGameItem(FieldLocation field, FieldLocation robot){
        if (field.heldByRobot && !field.priorHold){
            // robot just grabbed item, set field deltaX & deltaY and angle offset to robot angle
            field.deltaX = field.x - robot.x;
            field.deltaY = field.y - robot.y;
            field.thetaOffset = field.theta;
            robot.thetaOffset = robot.theta - field.theta;

            double angleRotate = robot.theta - robot.thetaOffset - field.thetaOffset;
            field = rotationMatrix(field, robot, angleRotate);

        }
        else if (!field.heldByRobot && field.priorHold){
            // robot just released item, set field deltaX & deltaY to zero and angle offset
            field.thetaOffset = field.theta;
            field.deltaX = 0;
            field.deltaY = 0;
        }
        else {
//            field.rotx = robot.x;
//            field.roty = robot.y;
            double angleRotate = robot.theta - robot.thetaOffset - field.thetaOffset;

            field = rotationMatrix(field, robot, angleRotate);

        }
        return  field;
    }
    private FieldLocation updateWobbleGoal(FieldLocation wobble, FieldLocation robot, WobbleArm wga, BasicAuto om){
        double totalAngle = 0.7*wga.convertArmAngleDegrees(wga.wobbleGoalArm.fakePosition)+wga.ARM_INIT_ANGLE_DEG;
        double length = Math.cos(totalAngle*Math.PI/180.0) * wga.ARM_LENGTH;
        if (wobble.heldByRobot && !wobble.priorHold){
            // robot just grabbed item, set field deltaX & deltaY and angle offset to robot angle
            // use the WGA motor to determine the position for the gripper
            // use the internal position vs. calling a method that will recalculate and move the arm
            wobble.deltaY = wga.ARM_Y + length;
            wobble.deltaX = wga.ARM_X;
            wobble.thetaOffset = wobble.theta;
            robot.thetaOffset = robot.theta - wobble.theta;

            double angleRotate = robot.theta - robot.thetaOffset - wobble.thetaOffset;
            wobble = rotationMatrix(wobble, robot, angleRotate);

        }
        else if (!wobble.heldByRobot && wobble.priorHold){
            // robot just released item, set field deltaX & deltaY to zero and angle offset
            wobble.thetaOffset = wobble.theta;
            wobble.deltaX = 0;
            wobble.deltaY = 0;
        }
        else {
            //update position while held
            wobble.deltaY = wga.ARM_Y + length;
            wobble.deltaX = wga.ARM_X;
            double angleRotate = robot.theta - robot.thetaOffset - wobble.thetaOffset;
            wobble = rotationMatrix(wobble, robot, angleRotate);

        }
        om.telemetry.addData("\t Blue Wobble "," Hold Status: Current = %s, Prior = %s, ",wobble.heldByRobot,wobble.priorHold);
        om.telemetry.addData("\t Blue Wobble ","X = %.1f, Y = %.1f",wobble.x,wobble.y);
        om.telemetry.addData("\t Blue Wobble ","Relative Angle = %.1f, Total Angle = %.1f, length= %.1f,",wga.convertArmAngleDegrees(wga.wobbleGoalArm.fakePosition),totalAngle,length);

        return  wobble;
    }
    private FieldLocation rotationMatrix(FieldLocation field, FieldLocation robot, double angle){
        //calculate new location of point based on angle rotation and distance for rotation center
        field.x = robot.x + field.deltaX*Math.cos(Math.toRadians(angle)) - field.deltaY*Math.sin(Math.toRadians(angle));
        field.y = robot.y + field.deltaX*Math.sin(Math.toRadians(angle)) + field.deltaY*Math.cos(Math.toRadians(angle));
        field.theta = field.thetaOffset + angle;
        return  field;
    }

    public void writeFieldAsText(FileOutputStream fos, ArrayList<FieldLocation> fieldList, int size){

        try {

            OutputStreamWriter osw = new OutputStreamWriter(fos);

//          Write the data in text format that can be read back in by the Java visualization programs in IntelliJ
//          Only writing out the FieldLocation X,Y, Theta as formatted for reading in, Used for foundations,stones, robot, and gripper
            osw.write("X (in.)"+"\t"+"Y (in.)"+"\t"+"Theta (rad.)"+"\n"); // Header Row

            for (int j = 0; j < size; j++) {
                // writes the bytes for each double in the array
                osw.write(Double.toString(fieldList.get(j).x)+"\t"); // FieldLocation X position on field in inches
                osw.write(Double.toString(fieldList.get(j).y)+"\t");   // FieldLocation Y position on field in inches
                osw.write(Double.toString(Math.toRadians(fieldList.get(j).theta))+"\n");   // FieldLocation angle on field in radians
            }
            if(osw != null){
                osw.flush();
                osw.close();
            }

        }
        catch(IOException e){
            e.printStackTrace();
            System.out.println(String.format("Error occurred","%S",e));
        }

    }
    public void writeRingType(FileOutputStream fos){

        try {

            OutputStreamWriter osw = new OutputStreamWriter(fos);

//          Write the data in text format that can be read back in by the Java visualization programs in IntelliJ
//          Only writing out a string to indicate "ring or "ringStack"
            osw.write(ringType); //string for ring type
            if(osw != null){
                osw.flush();
                osw.close();
            }

        }
        catch(IOException e){
            e.printStackTrace();
            System.out.println(String.format("Error occurred","%S",e));
        }

    }
    private boolean seeRing(FieldLocation b1, FieldLocation b2, FieldLocation r1, FieldLocation r2, FieldLocation robot) {
        boolean ringInView = false;
        // Should update the checks to include that the robot is facing the rings
        // atan2(ringY-botY,ringX-botX) within +/- 10 deg{?} of IMU angle
        //Math.hypot(ringY-botY,ringX-botX) < 15" {?} distance to camera is close enough
        if((Math.abs(b1.y - robot.y) < 4) && (Math.abs(b1.x - robot.x) < 20)){
            ringInView = true;
        }
        else if((Math.abs(b2.y - robot.y) < 4) && (Math.abs(b2.x - robot.x) < 20)){
            ringInView = true;
        }
        else if((Math.abs(r1.y - robot.y) < 4) && (Math.abs(r1.x - robot.x) < 20)){
            ringInView = true;
        }
        else if((Math.abs(r2.y - robot.y) < 4) && (Math.abs(r2.x - robot.x) < 20)){
            ringInView = true;
        }
        else{ringInView = false;}
        //should this return an int[] that indicates which item is in view in case it could be multiple?
        return ringInView;
    }
}
