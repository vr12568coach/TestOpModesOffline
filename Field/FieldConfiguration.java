package OfflineCode.Field;

import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.ArrayList;

import UltimateGoal_RobotTeam.OpModes.Autonomous.BasicAuto;
import UltimateGoal_RobotTeam.Utilities.FieldLocation;

/** CLASS IS USED TO SET SKYSTONE FIELD
 * ADAPT FOR ULTIMATE GOAL FOR WOBBLE GOAL & RINGS - CLASS OBJECT IS "the field"
 * COMMENTED OUT PORTION OF CODE FOR VARIABLES NOT IN OPMODE
 */

public class FieldConfiguration {
    public ArrayList<FieldLocation> RedFoundationPoints =new ArrayList();
    public ArrayList<FieldLocation> BlueFoundationPoints =new ArrayList();
    public ArrayList<FieldLocation> BlueSkyStone1Points =new ArrayList();
    public ArrayList<FieldLocation> RedSkyStone1Points =new ArrayList();
    public ArrayList<FieldLocation> BlueSkyStone2Points =new ArrayList();
    public ArrayList<FieldLocation> RedSkyStone2Points =new ArrayList();
    public ArrayList<FieldLocation> PursuitPoints =new ArrayList();
    public ArrayList<FieldLocation> NavPoints1 =new ArrayList();
    public ArrayList<FieldLocation> NavPoints2 =new ArrayList();

    private int stonePosition = 1;
//    public ArrayList<FieldLocation> GripperPoints =new ArrayList();
//    public ArrayList<FieldLocation> RobotPoints =new ArrayList();

//    Define RedFoundation initial position
    public FieldLocation redFound = new FieldLocation(15, 51, 0);
    //Define BlueFoundation initial position
    public FieldLocation blueFound = new FieldLocation(-10, 51, 0);

    //Define Blue SkyStone 1 initial position
    public FieldLocation blueStone1 = new FieldLocation(-20, -27, 0);

    //Define Blue SkyStone 2 initial position
    public FieldLocation blueStone2 = new FieldLocation(blueStone1.x, blueStone1.y-24, 0);

    //Define Red SkyStone ` initial position
    public FieldLocation redStone1 = new FieldLocation(22.5, -27, 0);//Updated location with visualization changes to make field more square

    //Define Red SkyStone 2 initial position
    public FieldLocation redStone2 = new FieldLocation(redStone1.x, redStone1.y-24, 0);

    public boolean stoneFound = false;

    public FieldConfiguration(){
        // Blank constructor
    }
    public FieldConfiguration(int stonePos){
        // constructor to set stone locations in 0, 1, 2
        stonePosition = stonePos;
        //Define Blue SkyStone 1 initial position
        blueStone1 = new FieldLocation(-20, -27-stonePos*8, 0);

        //Define Blue SkyStone 2 initial position
        blueStone2 = new FieldLocation(blueStone1.x, blueStone1.y-24, 0);

        //Define Red SkyStone ` initial position
        redStone1 = new FieldLocation(22.5, blueStone1.y, 0);//Updated location with visualization changes to make field more square

        //Define Red SkyStone 2 initial position
        redStone2 = new FieldLocation(redStone1.x, redStone1.y-24, 0);

    }

public void updateField(BasicAuto opMode) {
            //FoundationOpMode and Stone calculations

    redFound.setHold(opMode.haveRedFoundation);
    blueFound.setHold(opMode.haveBlueFoundation);
    blueStone1.setHold(opMode.haveBlueSkyStone1);
    blueStone2.setHold(opMode.haveBlueSkyStone2);
    redStone1.setHold(opMode.haveRedSkyStone1);
    redStone2.setHold(opMode.haveRedSkyStone2);

    if(redFound.heldByRobot){
        redFound = updateGameItem(redFound,opMode.robotUG.driveTrain.imu.robotOnField);
    }
    if(blueFound.heldByRobot){
        blueFound = updateGameItem(blueFound,opMode.robotUG.driveTrain.imu.robotOnField);
    }
    if(blueStone1.heldByRobot){
        blueStone1 = updateGameItem(blueStone1,opMode.robotUG.driveTrain.imu.robotOnField);
    }
    if(blueStone2.heldByRobot){
        blueStone2 = updateGameItem(blueStone2,opMode.robotUG.driveTrain.imu.robotOnField);
    }
    if(redStone1.heldByRobot){
        redStone1 = updateGameItem(redStone1,opMode.robotUG.driveTrain.imu.robotOnField);
    }
    if(redStone2.heldByRobot){
        redStone2 = updateGameItem(redStone2,opMode.robotUG.driveTrain.imu.robotOnField);
    }

    stoneFound = seeStone(blueStone1,blueStone2,redStone1,redStone2,opMode.robotUG.driveTrain.imu.robotOnField);

    RedFoundationPoints.add(new FieldLocation(redFound.x,redFound.y,redFound.theta));
    BlueFoundationPoints.add(new FieldLocation(blueFound.x,blueFound.y,blueFound.theta));

    BlueSkyStone1Points.add(new FieldLocation(blueStone1.x,blueStone1.y,blueStone1.theta));
    BlueSkyStone2Points.add(new FieldLocation(blueStone2.x,blueStone2.y,blueStone2.theta));

    RedSkyStone1Points.add(new FieldLocation(redStone1.x,redStone1.y,redStone1.theta));
    RedSkyStone2Points.add(new FieldLocation(redStone2.x,redStone2.y,redStone2.theta));

    PursuitPoints.add(new FieldLocation(opMode.robotUG.driveTrain.targetPoint.x,opMode.robotUG.driveTrain.targetPoint.y,0));
    NavPoints1.add(new FieldLocation(opMode.robotUG.driveTrain.robotFieldLocationNav1.x,opMode.robotUG.driveTrain.robotFieldLocationNav1.y,0));
    NavPoints2.add(new FieldLocation(opMode.robotUG.driveTrain.robotFieldLocationNav2.x,opMode.robotUG.driveTrain.robotFieldLocationNav2.y,0));



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

    private boolean seeStone(FieldLocation b1,FieldLocation b2,FieldLocation r1,FieldLocation r2, FieldLocation robot) {
        boolean stoneInView = false;
        if((Math.abs(b1.y - robot.y) < 4) && (Math.abs(b1.x - robot.x) < 20)){
            stoneInView = true;
        }
        else if((Math.abs(b2.y - robot.y) < 4) && (Math.abs(b2.x - robot.x) < 20)){
            stoneInView = true;
        }
        else if((Math.abs(r1.y - robot.y) < 4) && (Math.abs(r1.x - robot.x) < 20)){
            stoneInView = true;
        }
        else if((Math.abs(r2.y - robot.y) < 4) && (Math.abs(r2.x - robot.x) < 20)){
            stoneInView = true;
        }
        else{stoneInView = false;}
        return stoneInView;
    }
}
