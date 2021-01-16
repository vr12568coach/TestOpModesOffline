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

package OfflineCode.OfflineOpModes;


import UltimateGoal_RobotTeam.HarwareConfig.Conveyor;
import UltimateGoal_RobotTeam.HarwareConfig.DriveTrain;
import UltimateGoal_RobotTeam.HarwareConfig.HardwareRobotMulti;
import UltimateGoal_RobotTeam.OpModes.Autonomous.BasicAuto;
//import com.qualcomm.robotcore.hardware.HardwareMap;

import java.io.DataOutputStream;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.ArrayList;
import java.util.Arrays;
import UltimateGoal_RobotTeam.OpModes.Autonomous.PurePursuit.PurePursuitAutoDemo;
import UltimateGoal_RobotTeam.Utilities.PursuitLines;
import UltimateGoal_RobotTeam.Utilities.PursuitPoint;
import OfflineCode.Field.FieldConfiguration;
import UltimateGoal_RobotTeam.Utilities.FieldLocation;

/**
 * This is NOT an opmode.
 *
 *
 */

public class OfflineOpModeLibs extends BasicAuto {


//****************************************
//DECLARE VARIABLES FROM CODE BEING TESTED
//****************************************

//****************************************
// DECLARE VARIABLES NEEDED FOR TEST CODE
//****************************************
    private FieldConfiguration fc = new FieldConfiguration();
    PurePursuitAutoDemo ppOpMode = new PurePursuitAutoDemo();

    boolean writeBR = false;
    boolean writeRR = false;
    boolean writeBW1 = false;
    boolean writeBW2 = false;
    boolean writeRW1 = false;
    boolean writeRW2 = false;

    //********** Added to OfflineOpModeLibs - were in BasicAuto or Hardware ******************
    boolean robotSeeStone = false;

    private int robotNumber = 1;

    //Instantiate the Autonomous OpMode you wish to Run - can be multiple for each of 4 robots

    // Finish instantiations

    //****************** Added above *************************************************

    int counter;
    final static int size = 300;
    int[] flCounts = new int[size];
    int[] frCounts = new int[size];
    int[] brCounts = new int[size];
    int[] blCounts = new int[size];
    int[] flIMU = new int[size];
    int[] frIMU = new int[size];
    int[] brIMU = new int[size];
    int[] blIMU = new int[size];

    int[] collectorArray = new int[size];
    int[] conveyorArray = new int[size];
    int[] shooterArray = new int[size];
    double[] wgaAngleArray = new double[size];

    double[] arrayRobotX = new double[size];
    double[] arrayRobotY= new double[size];
    double[] arrayRobotDist = new double[size];
    double[] arrayRobotAngle = new double[size];

    public double[] arrayFLBR=new double[size];
    public double[] arrayFRBL=new double[size];

    double[] arrayFieldX = new double[size];
    double[] arrayFieldY= new double[size];
    double[] arrayFieldDist = new double[size];

    double[] timeArray= new double[size];
    /* Added time step to BasicOpMode */
//    public double timeStep = 135;//determined a fixed time step (in milliseconds) so that faster speeds will show shorter time to distance
    // really not the time step but the speed of the motor at maxpower in counts/second
    //timeStep was 100 in seconds to fill 30 seconds / size of array = 1000 * 30/size;
    //  Measured motor speed 60 inches in 4.0 seconds @ 0.75 = 60/4.0/0.75 = 20 in/s * 360 / (3.875 * 3.14159) * 4 = 2360 counts/s
    //  10 time steps = 1 second then 1 time step = 0.1 seconds
    //    in 1 time step the max speed = 2360 counts/s * 0.1 s = 236 counts adjust to 230
    // tried 230 but too fast with other driving methods not using IMU, compromised to 150


    static FileReader in = null;
    static FileWriter out = null;
    static FileOutputStream fileOutStream = null;
    static DataOutputStream dataOutStream = null;
    static DataOutputStream dos = null;
    static FileOutputStream fos = null;
    public String SelectedOpMode = null;

    static String fileLocation;
    static computer location;
    int IMUCounter =0;

    public boolean opModeIsRunning = true;


    /* Constructor */
    public OfflineOpModeLibs(){

    };


    //----------------------------------------------------------------------------------------------
    // TEST MODE CODE ONLY - EXTRACTS ARRAY DATA FROM classes
    //----------------------------------------------------------------------------------------------
    public void extractArrayData(){


        flIMU = robotUG.driveTrain.imu.flArray;
        frIMU = robotUG.driveTrain.imu.frArray;
        brIMU = robotUG.driveTrain.imu.brArray;
        blIMU = robotUG.driveTrain.imu.blArray;

        timeArray = robotUG.driveTrain.imu.timeArray;

        arrayRobotX = robotUG.driveTrain.imu.robotXArray;
        arrayRobotY = robotUG.driveTrain.imu.robotYArray;
        arrayRobotDist = robotUG.driveTrain.imu.robotDistArray;
        arrayRobotAngle = robotUG.driveTrain.imu.robotAngleArray;
        IMUCounter = robotUG.driveTrain.imu.counter;

        arrayFLBR = robotUG.driveTrain.imu.FLBRArray;
        arrayFRBL = robotUG.driveTrain.imu.FRBLArray;


        Arrays.fill(arrayRobotDist,IMUCounter,(size),arrayRobotDist[IMUCounter-1]);
        Arrays.fill(arrayRobotAngle,IMUCounter,(size),arrayRobotAngle[IMUCounter-1]);

        Arrays.fill(arrayFieldDist,IMUCounter,(size),arrayFieldDist[IMUCounter-1]);
        Arrays.fill(collectorArray,IMUCounter,(size), collectorArray[IMUCounter-1]);
        Arrays.fill(conveyorArray,IMUCounter,(size), conveyorArray[IMUCounter-1]);
        Arrays.fill(shooterArray,IMUCounter,(size), shooterArray[IMUCounter-1]);
        Arrays.fill(wgaAngleArray,IMUCounter,(size), wgaAngleArray[IMUCounter-1]);

        double deltaTime = (timeArray[1] - timeArray[0]);
        for(int k = IMUCounter-1; k < size;k++){// needed to reduce counter by 1 -- means there is an extra count somewhere
            timeArray[k] = timeArray[k-1] + deltaTime;
            robotUG.driveTrain.imu.RobotPoints.add(robotUG.driveTrain.imu.RobotPoints.get(k-1));

            fc.BlueRingPoints.add(fc.BlueRingPoints.get(k-1));
            fc.RedRingPoints.add(fc.RedRingPoints.get(k-1));

            fc.RedWobble1Points.add(fc.RedWobble1Points.get(k-1));
            fc.BlueWobble1Points.add(fc.BlueWobble1Points.get(k-1));

            fc.BlueWobble2Points.add(fc.BlueWobble2Points.get(k-1));
            fc.RedWobble2Points.add(fc.RedWobble2Points.get(k-1));
            fc.PursuitPoints.add(fc.PursuitPoints.get(k-1));
            fc.NavPoints.add(fc.NavPoints.get(k-1));

        }

    }

    //----------------------------------------------------------------------------------------------
    // TEST MODE CODE ONLY - WRITES DATA TO FILE
    //----------------------------------------------------------------------------------------------
    public void writeToFile( FileWriter out, DataOutputStream fileData){
        //Delete FileWriter Portion or entire method??
        int countVar = Math.max(size, IMUCounter);

        try {
            out.write(String.format("Calculated IMU Counts\r"));
            out.write(String.format("Time,FL_IMU,FR_IMU,BR_IMU,BL_IMU,LS_IMU,FLBR_cnt,FRBL_cnt,"+
                    "RobotY, RobotX, RobotAngle, Field_Y, Field_X\r"));

            for (int i = 0; i < countVar; i++) {
//                out.write(String.format("%d,%d,%d,%d,%d,", i,  flCounts[i], frCounts[i], brCounts[i], blCounts[i]));
                out.write(String.format("%.3f,%d,%d,%d,%d,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f",
                        timeArray[i], flIMU[i], frIMU[i], brIMU[i], blIMU[i], collectorArray[i],
                        arrayFLBR[i],arrayFRBL[i],arrayRobotY[i], arrayRobotX[i], arrayRobotAngle[i],
                        arrayFieldY[i], arrayFieldX[i]));
                out.write(String.format("\n"));
            }

//            out.write(String.format("Robot Motion Final Values\n"));
//            out.write(String.format("Robot final rotation angle (deg.):,%.3f\n", arrayRobotAngle[size-1]));
//            out.write(String.format("Robot final X distance (in.):,%.3f \n", arrayRobotX[size-1]));
//            out.write(String.format("Robot final Y distance (in.):,%.3f \n", arrayRobotY[size-1]));
//            out.write(String.format("Robot final total distance (in.):,%.3f \n", arrayRobotDist[size-1]));

//
//            out.write(String.format("Final Distance:,%.3f\r", distanceTraveledArray[distanceTraveledArray.length - 1]));
//            out.write(String.format("Final Error:,%.3f\r", arrayRobotDist[size-1] - distanceTraveledArray[distanceTraveledArray.length - 1]));

//            out.write(String.format("Distance Traveled Array Values:\r"));
//            out.write(String.format("Time, code Dist Travel, IMU RobotY, IMU Robot X, IMU RobotDist, IMU Angle, IMU Field Y, IMU Field X, IMU Field Dist\r"));
//            for (int j = 0; j < distanceTraveledArray.length; j++) {
//                out.write(String.format("%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%.3f,%.3f,%.3f\r",
//                        timeArray[j], distanceTraveledArray[j],
//                        arrayRobotY[j], arrayRobotX[j], arrayRobotDist[j], arrayRobotAngle[j],
//                        arrayFieldY[j], arrayFieldX[j], arrayFieldDist[j]));
//            }
            if (out != null) {
                out.close();
            }
        }catch(IOException e){
            e.printStackTrace();

        }

        try {
//          Write the data in binary format that can be read back in by the Java plotting programs in IntelliJ
//          Only writing out the robot X,Y, Theta as formatted for reading in

            for (int x = 0; x < countVar; x++) {
                // writes the bytes for each double in the array
                fileData.writeDouble(arrayFieldX[x]);   // robot position on field in X in inches
                fileData.writeDouble(arrayFieldY[x]);   // robot position on field in Y in inches
                fileData.writeDouble(Math.toRadians(arrayRobotAngle[x]));   // robot angle on field in radians
            }
            if(fileData != null){
                fileData.flush();
                fileData.close();
            }

        }
        catch(IOException e){
            e.printStackTrace();

        }

    }

    //----------------------------------------------------------------------------------------------
    // TEST MODE CODE ONLY - WRITES DATA TO FILE
    //----------------------------------------------------------------------------------------------
    public void writeListToFile(DataOutputStream fileData, ArrayList<FieldLocation> fieldList){
        int countVar = Math.max(size, IMUCounter);

        try {
//          Write the data in binary format that can be read back in by the Java plotting programs in IntelliJ
//          Only writing out the FieldLocation X,Y, Theta as formatted for reading in, Used for foundations, and stones so could also add robot

            for (int j = 0; j < countVar; j++) {
                // writes the bytes for each double in the array
                fileData.writeDouble(fieldList.get(j).x);   // FieldLocation X position on field in inches
                fileData.writeDouble(fieldList.get(j).y);   // FieldLocation Y position on field in inches
                fileData.writeDouble(Math.toRadians(fieldList.get(j).theta));   // FieldLocation angle on field in radians
            }
            if(fileData != null){
                fileData.flush();
                fileData.close();
            }

        }
        catch(IOException e){
            e.printStackTrace();
            telemetry.addData("Error occurred","%S",e);
            telemetry.update();

        }

    }

    //----------------------------------------------------------------------------------------------
    // TEST MODE CODE ONLY - WRITES DATA TO FILE
    //----------------------------------------------------------------------------------------------
    public void writeExtrasToFile(FileOutputStream fos){
        int countVar = Math.max(size, IMUCounter);


        try {

            OutputStreamWriter osw = new OutputStreamWriter(fos);

//          Write the data in text format that can be read back in by the Java visualization programs in IntelliJ
//          Only writing out the power or position as formatted for reading in, Used for accessory parts that are always attached to robot
            for (int j = 0; j < size; j++) {
                // writes the data as text for each value in the array
                osw.write(Integer.toString(collectorArray[j])+"\t");   // collector Power - use to determine if ON
                osw.write(Integer.toString(conveyorArray[j])+"\t");   // conveyor Power - use to determine if ON
                osw.write(Integer.toString(shooterArray[j])+"\t");   // shooter Power - use to determine if ON
                osw.write(Double.toString(wgaAngleArray[j])+"\n");   // Wobble Goal Arm angle / motion
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

    public void writePath(FileOutputStream fos, ArrayList<PursuitLines> lines, int size){

        try {

            OutputStreamWriter osw = new OutputStreamWriter(fos);

//          Write the data in text format that can be read back in by the Java visualization programs in IntelliJ
//          Only writing out the FieldLocation X,Y, Theta as formatted for reading in, Used for foundations,stones, robot, and gripper
            osw.write("X1 (in.)"+"\t"+"Y1 (in.)"+"\t"+"X2 (in.)"+"\t"+"Y2 (in.)"+"\n"); // Header Row

            for (int j = 0; j < size; j++) {
                // writes the bytes for each double in the array
                osw.write(Double.toString(lines.get(j).x1)+"\t"); // Path X1 position on field in inches
                osw.write(Double.toString(lines.get(j).y1)+"\t");   // Path Y1 position on field in inches
                osw.write(Double.toString(lines.get(j).x2)+"\t"); // Path X2 position on field in inches
                osw.write(Double.toString(lines.get(j).y2)+"\n");   // FPath Y2 position on field in inches
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

    //----------------------------------------------------------------------------------------------
    // TEST MODE CODE ONLY - UPDATES FAKE IMU
    //----------------------------------------------------------------------------------------------
    @Override
    public void updateIMU(){
        //Move this imu portion to robot update method
        robotUG.driveTrain.imu.flCnt = robotUG.driveTrain.frontLeft.getCurrentPosition();
        robotUG.driveTrain.imu.frCnt = robotUG.driveTrain.frontRight.getCurrentPosition();
        robotUG.driveTrain.imu.brCnt = robotUG.driveTrain.backRight.getCurrentPosition();
        robotUG.driveTrain.imu.blCnt = robotUG.driveTrain.backLeft.getCurrentPosition();

        IMUCounter = robotUG.driveTrain.imu.counter;

        collectorArray[IMUCounter] = (int) Math.round(robotUG.collector.collectorPower);
        conveyorArray[IMUCounter] = (int) Math.round(robotUG.conveyor.conveyor_Power);
        shooterArray[IMUCounter] = (int) Math.round(robotUG.shooter.shooter_Power);
        wgaAngleArray[IMUCounter] = robotUG.wobbleArm.getArmAngleDegrees() * Math.PI/180.0;

        fc.updateField(this);

        robotSeeStone= fc.ringFound;


        if(haveBlueRing){writeBR = true;}
        if(haveRedRing){writeRR = true;}
        if(haveBlueWobble1){writeBW1 = true;}
        if(haveBlueWobble2){writeBW2 = true;}
        if(haveRedWobble1){writeRW1 = true;}
        if(haveRedWobble2){writeRW2 = true;}

        try {
//
                if (IMUCounter >= size) {
                    int a = 1 / 0;
                }
            } catch (ArithmeticException e) {
                System.out.println(String.format("Exceeded %d counter steps", size));
                System.out.println(String.format("IMU Counter = %d", IMUCounter));
            }


    }


    //----------------------------------------------------------------------------------------------
    // TEST MODE METHOD prepOpMode is used to initialize offline items - instead of initOpMode
    //----------------------------------------------------------------------------------------------
   public void prepOpMode() {

   //************* BELOW IS TEST CODE ********************************


       testModeActive = true;// set for each OpMode


       ringSelect = 1;// Options are 0, 1, 4 rings on the field
       fc = new FieldConfiguration(ringSelect);//KS added 12/20 to set stone position

       haveBlueRing = false;
       haveRedRing = false;
       haveBlueWobble1 = false;
       haveBlueWobble2 = false;
       haveRedWobble1 = false;
       haveRedWobble2 = false;

       writeBR = false;
       writeRR = false;
       writeBW1 = false;
       writeBW2 = false;
       writeRW1 = false;
       writeRW2 = false;

       switch(location) {
           case KARL:
               fileLocation = "/Users/karl/LocalDocuments/FTC/IntelliJ/RobotVisualization/";
               break;
           case PC:
               fileLocation = "C:/Users/Spiessbach/Documents/FTC/IntelliJ Projects/RobotVisualization/";
               break;
           case MAC:
               fileLocation = "/Users/caleb/Documents/FTC/IntelliJ/RobotVisualization/";
               break;
           case WILL:
               fileLocation = "/";
               break;
       }

       //************* ABOVE IS TEST CODE ********************************

       // %%%%%% UPDATED BELOW FOR HardwareRobotMulti class %%%%%%%%%

       // configure the robot needed - for this demo only need DriveTrain
       // configArray has True or False values for each subsystem HW element
       //
       /** configArray is arranged as
        * [0] = DriveTrain
        * [1] = Shooter
        * [2] = Conveyor
        * [3] = WobbleArm
        * [4] = Collector
        * [5] = ImageRecog
        * items that are 1 = true will be configured to the robot
        */
       // HW ELEMENTS *****************    DriveTrain  Shooter  Conveyor	WobbleArm	Collector   ImageRecog
       boolean[] configArray = new boolean[]{ true, 	true, 	true, 		true, 		true,       false};

       robotUG = new HardwareRobotMulti(this, configArray, testModeActive);

       telemetry.update();
       //***********************************************************
       //Code that needs to be Kept in init to initialize functions
       //***********************************************************



//       MOVED timeStep to constructors on HW items for test mode

       fc.RedRingPoints.clear();
       fc.BlueRingPoints.clear();
       fc.BlueWobble1Points.clear();
       fc.RedWobble1Points.clear();
       fc.BlueWobble2Points.clear();
       fc.RedWobble2Points.clear();

       collectorArray= new int[size];;
       conveyorArray= new int[size];
       shooterArray= new int[size];
       wgaAngleArray= new double[size];

       fc.updateField(this);


       robotUG.driveTrain.imu.RobotPoints.clear();
//       robotUG.driveTrain.imu.RobotPoints.add(new FieldLocation(robotUG.driveTrain.imu.robotOnField.x, robotUG.driveTrain.imu.robotOnField.y, robotUG.driveTrain.imu.robotOnField.theta));

       //Setting counter to capture array data is unique to offline running of code
       counter = 1;
       robotUG.driveTrain.imu.counter = counter;


       if(robotNumber == 1) {
           cons.DRIVE_POWER_LIMIT = 1.0;
           robotUG.driveTrain.frontLeft.motorTol=1.0;
           robotUG.driveTrain.frontRight.motorTol=1.0;
           robotUG.driveTrain.backRight.motorTol=1.0;
           robotUG.driveTrain.backLeft.motorTol=1.0;
           //field angle orientation is + = CCW , while robot frame is + = CW
//           robotUG.driveTrain.robotFieldLocation.setLocation(-36,-63,90);// FROM CompleteAutonomous
           robotUG.driveTrain.imu.robotOnField.x = -36;//initial x position on field in inches
           robotUG.driveTrain.imu.robotOnField.y = -63;//initial y position on field in inches
           robotUG.driveTrain.imu.robotOnField.theta = 90;//initial robot angle orientation on field in degrees from EAST CCW +
           robotUG.driveTrain.imu.priorAngle = robotUG.driveTrain.imu.robotOnField.theta;//initial robot angle orientation on field in degrees from EAST CCW +
           robotUG.driveTrain.imu.fakeAngle = (float) robotUG.driveTrain.imu.robotOnField.theta;//initial robot angle orientation on field in degrees from EAST CCW +
           robotUG.driveTrain.robotHeading = -robotUG.driveTrain.imu.fakeAngle;//initial robot angle orientation on field in degrees from EAST, CW +

           robotUG.driveTrain.robotFieldLocation = robotUG.driveTrain.imu.robotOnField;

//           robotUG.driveTrain.targetPoint.setPoint(robotUG.driveTrain.imu.robotOnField.x,robotUG.driveTrain.imu.robotOnField.y);


       }

       if(robotNumber == 2) {

//           cons.DRIVE_POWER_LIMIT = 0.75;
           robotUG.driveTrain.frontLeft.motorTol=1.0;
           robotUG.driveTrain.frontRight.motorTol=1.0;
           robotUG.driveTrain.backRight.motorTol=1.0;
           robotUG.driveTrain.backLeft.motorTol=1.0;
           //field angle orientation is + = CCW , while robot frame is + = CW
           robotUG.driveTrain.imu.robotOnField.x = -48;//initial x position on field in inches (Added 2 inches for robot 7" to wheel center vs. 9")
           robotUG.driveTrain.imu.robotOnField.y = -60;//initial y position on field in inches (WAS 48)
           robotUG.driveTrain.imu.robotOnField.theta = 45;//initial robot angle orientation on field in degrees from EAST (WAS 180 for backing to foundation)
           robotUG.driveTrain.imu.priorAngle = robotUG.driveTrain.imu.robotOnField.theta;//initial robot angle orientation on field in degrees from EAST (WAS 180 for backing to foundation)
           robotUG.driveTrain.imu.fakeAngle = (float) robotUG.driveTrain.imu.robotOnField.theta;//initial robot angle orientation on field in degrees from EAST (WAS 180 for backing to foundation)
           robotUG.driveTrain.robotHeading = -robotUG.driveTrain.imu.fakeAngle;//initial robot angle orientation on field in degrees from EAST

           robotUG.driveTrain.robotFieldLocation = robotUG.driveTrain.imu.robotOnField;


       }

       if(robotNumber == 3) {
//           cons.DRIVE_POWER_LIMIT = 0.75;
           robotUG.driveTrain.frontLeft.motorTol=1.0;
           robotUG.driveTrain.frontRight.motorTol=1.0;
           robotUG.driveTrain.backRight.motorTol=1.0;
           robotUG.driveTrain.backLeft.motorTol=1.0;
           //field angle orientation is + = CCW , while robot frame is + = CW
           robotUG.driveTrain.imu.robotOnField.x = 24;//initial x position on field in inches (Added 2 inches for robot 7" to wheel center vs. 9")
           robotUG.driveTrain.imu.robotOnField.y = -60;//initial y position on field in inches
           robotUG.driveTrain.imu.robotOnField.theta = 135;//initial robot angle orientation on field in degrees from EAST
           robotUG.driveTrain.imu.priorAngle = robotUG.driveTrain.imu.robotOnField.theta;//initial robot angle orientation on field in degrees from EAST
           robotUG.driveTrain.imu.fakeAngle = (float) robotUG.driveTrain.imu.robotOnField.theta;//initial robot angle orientation on field in degrees from EAST
           robotUG.driveTrain.robotHeading = -robotUG.driveTrain.imu.fakeAngle;//initial robot angle orientation on field in degrees from EAST

           robotUG.driveTrain.robotFieldLocation = robotUG.driveTrain.imu.robotOnField;


       }

       if(robotNumber == 4) {
//           cons.DRIVE_POWER_LIMIT = 0.75;
           robotUG.driveTrain.frontLeft.motorTol=1.0;
           robotUG.driveTrain.frontRight.motorTol=1.0;
           robotUG.driveTrain.backRight.motorTol=1.0;
           robotUG.driveTrain.backLeft.motorTol=1.0;
           robotUG.driveTrain.imu.robotOnField.x = 48;//initial x position on field in inches (Added 2 inches for robot 7" to wheel center vs. 9")
           robotUG.driveTrain.imu.robotOnField.y = -60;//initial y position on field in inches (WAS 48)
           robotUG.driveTrain.imu.robotOnField.theta = 45;//initial robot angle orientation on field in degrees from EAST (WAS 0 for backing to foundation)
           robotUG.driveTrain.imu.priorAngle = robotUG.driveTrain.imu.robotOnField.theta;//initial robot angle orientation on field in degrees from EAST (WAS 0 for backing to foundation)
           robotUG.driveTrain.imu.fakeAngle = (float) robotUG.driveTrain.imu.robotOnField.theta;//initial robot angle orientation on field in degrees from EAST (WAS 0 for backing to foundation)
           robotUG.driveTrain.robotHeading = -robotUG.driveTrain.imu.fakeAngle;//initial robot angle orientation on field in degrees from EAST

           robotUG.driveTrain.robotFieldLocation = robotUG.driveTrain.imu.robotOnField;

       }
       robotUG.driveTrain.initIMUtoAngle(-robotUG.driveTrain.robotFieldLocation.theta);//ADDED HERE FOR OFFLINE, NEEDS TO BE IN initialize() method in OpMode
       telemetry.addData("Robot Number ", "%d",robotNumber);
       telemetry.addData("drivePowerLimit ", "%.2f",cons.DRIVE_POWER_LIMIT);
       telemetry.addData("Robot Field Location", "X = %.2f inch, Y = %.2f inch, Theta = %.2f degrees",robotUG.driveTrain.robotFieldLocation.x, robotUG.driveTrain.robotFieldLocation.y, robotUG.driveTrain.robotFieldLocation.theta);
       telemetry.update();
       // COMMENTED OUT BECAUSE CAUSING ERRORS IN ROBOT IMU POSITION
//       robotUG.driveTrain.angleUnWrap();
//       robotUG.driveTrain.offset = robotUG.driveTrain.robotHeading;
//       robotUG.driveTrain.robotHeading-=robotUG.driveTrain.offset;//set robotHeading = 0 for all opModes regardless of position, but track actual angle in IMU
       // ** END COMMENTED ****
       fc.updateField(this);
       //Initialize starting position on field, field center is assumed (0,0), 0 field angle is pointing EAST
//       robotUG.driveTrain.imu.fieldXArray[0] = robotUG.driveTrain.imu.fieldX; //initial x position on field in inches
//       robotUG.driveTrain.imu.fieldYArray[0] = robotUG.driveTrain.imu.fieldY; //initial y position on field in inches
//       robotUG.driveTrain.imu.robotAngleArray[0] = robotUG.driveTrain.imu.priorAngle; //initial robot angle orientation on field in degrees from EAST
//       robotUG.driveTrain.imu.RobotPoints.add(new FieldLocation(robotUG.driveTrain.imu.robotOnField.x, robotUG.driveTrain.imu.robotOnField.y, robotUG.driveTrain.imu.robotOnField.theta));

   }


   public enum computer{KARL,PC,MAC,WILL};
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    // INSERT ACTUAL CODE TO BE TESTED IN METHODS
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //----------------------------------------------------------------------------------------------
    // TEST MODE METHOD runAutonomous is populated with actual program code in runOpMode
    //----------------------------------------------------------------------------------------------

    @Override
    public void initialize() {


    }
    @Override
    public void runOpMode(){



        if (robotNumber ==1) {
                runtime.reset();
                 cons.DRIVE_POWER_LIMIT = 0.7;
                cons.STEERING_POWER_LIMIT = 0.6;//was somewhere between 0.60 and 0.72 X DRIVE_POWER_LIMIT
                cons.STEERING_POWER_GAIN = 0.03;//was 0.05
                robotUG.driveTrain.setGearRatio(40.0);

                fieldPoints.add(new PursuitPoint(robotUG.driveTrain.robotFieldLocation.x  ,robotUG.driveTrain.robotFieldLocation.y));

                robotUG.wobbleArm.wobbleGoalServo.setPosition(0.9);
                // Add point for driving to view ring stack
                fieldPoints.add(new PursuitPoint(-36, -43));

                // Simple set of points - diamond or square
//                fieldPoints.add(new PursuitPoint(-30,-30));
//                fieldPoints.add(new PursuitPoint(-60,-30));
//                fieldPoints.add(new PursuitPoint(-60,-60));
//                fieldPoints.add(new PursuitPoint(-38,-60));

                // Slalom course - doesn't get to 180 so should be good
//                fieldPoints.add(new PursuitPoint(12,0));
//                fieldPoints.add(new PursuitPoint(12,60));
//                fieldPoints.add(new PursuitPoint(24,60));
//                fieldPoints.add(new PursuitPoint(24,0));
//                fieldPoints.add(new PursuitPoint(36, 0));
//                fieldPoints.add(new PursuitPoint(36,60));
//                fieldPoints.add(new PursuitPoint(48,60));
//                fieldPoints.add(new PursuitPoint(48,0));
//                fieldPoints.add(new PursuitPoint(60, 0));
//                fieldPoints.add(new PursuitPoint(60,60));


                // angled line
//                fieldPoints.add(new PursuitPoint(-24,-36));
//                fieldPoints.add(new PursuitPoint(-24,-24));
//                fieldPoints.add(new PursuitPoint(-30,-24));
//                fieldPoints.add(new PursuitPoint(-30,-12));
//                fieldPoints.add(new PursuitPoint(-60,-12));
//                fieldPoints.add(new PursuitPoint(-60,12));

                // Circle
//                fieldPoints.add(new PursuitPoint(34,10.0));
//                ArrayList<PursuitPoint> circlePoints;
//                circlePoints = PursuitPath.defineArc(new PursuitPoint(12,12), 24.0, 0.0, 330*Math.PI/180,  25, PursuitPath.pathDirection.POSITIVE);
//                fieldPoints.addAll(circlePoints);
//                ArrayList<PursuitPoint> rectPoints;
//                rectPoints = PursuitPath.defineRectangle(36, 0, 24, -60, 10);
//                fieldPoints.addAll(rectPoints);

            // Display the robot points on the screen to confirm what was entered - needed for troubleshooting only
            for(int h=0;h<fieldPoints.size();h++) {
                telemetry.addData("Point", "%d: %.2f, %.2f", h, fieldPoints.get(h).x, fieldPoints.get(h).y);
            }
            telemetry.update();

            telemetry.addData("Drive Power Limit Updated", cons.DRIVE_POWER_LIMIT);
            telemetry.addData("Steering Power Limit Updated", cons.STEERING_POWER_LIMIT);
            telemetry.addData("Steering Power Gain Updated", cons.STEERING_POWER_GAIN);
            telemetry.addData("Forward Scale Factor", cons.adjForward);
            telemetry.addData("Right Scale Factor", cons.adjRight);
            telemetry.addData("Rotation Scale Factor", cons.adjRotate);
            telemetry.update();
            telemetry.addData("Robot Field Location", "X = %.2f inch, Y = %.2f inch, Theta = %.2f degrees",robotUG.driveTrain.robotFieldLocation.x, robotUG.driveTrain.robotFieldLocation.y, robotUG.driveTrain.robotFieldLocation.theta);
            telemetry.addData("Drive Train Gear Ratio", " %.1f : 1 & deg to Counts %.1f",robotUG.driveTrain.gearRatio, robotUG.driveTrain.gearRatioDegToCounts);
            telemetry.update();

            haveBlueWobble1 = true;

            robotUG.driveTrain.drivePursuit(fieldPoints,this,"To The RING STACK");

            // Add point for driving to wobble goal location

            /** Choose Where to go Next and Pick up Wobble Goal
             * case "None": Zone A pursuit points
             * case "Single": Zone B pursuit points
             * case "Quad": Zone C pursuit points
             * case "Multiple": error - seeing multiple objects, no selection
             */
            if(ringSelect == 0){// Options are 0, 1, 4 rings on the field
                 decideWobbleGoalZone("None");
            }
            else if(ringSelect == 1){// Options are 0, 1, 4 rings on the field
                decideWobbleGoalZone("Single");
            }
            else {// Options are 0, 1, 4 rings on the field
                decideWobbleGoalZone("Quad");
            }
            for(int h=0;h<fieldPoints.size();h++) {
                telemetry.addData("Point", "%d: %.2f, %.2f", h, fieldPoints.get(h).x, fieldPoints.get(h).y);
            }
            telemetry.update(); //Display points
            /* TEST CODE TO DRAW LINES FOR VIZ */
            for(int h=0;h<fieldPoints.size()-1;h++) {
                lines.add(new PursuitLines(fieldPoints.get(h).x, fieldPoints.get(h).y, fieldPoints.get(h+1).x, fieldPoints.get(h+1).y));
            }
            robotUG.driveTrain.drivePursuit(fieldPoints,this,"To Wobble Goal drop zone");

            /* COACH ADDITIONS: added helpful telemetry
             *  - Add telemetry before every "pressAToContinue" to provide updates on the robot's progress
             *  - Report the step complete, robot position, arm position, etc.
             */

            telemetry.addLine("Drive to Wobble Goal Drop Zone Completed");
            telemetry.addData("Desired Position (X, Y)", " \t\t( %1.1f, %1.1f)", robotUG.driveTrain.targetPoint.x, robotUG.driveTrain.targetPoint.y);
            telemetry.addData("Robot Position (X, Y)", " \t\t( %1.1f, %1.1f)", robotUG.driveTrain.robotFieldLocation.x, robotUG.driveTrain.robotFieldLocation.y);
            telemetry.addData("Robot Angles", " \t Desired: %1.1f, \t Actual: %1.1f", robotUG.driveTrain.targetHeading, robotUG.driveTrain.robotHeading);
            telemetry.update();// Review robot's motion

            /** Rotate 180*, Drop the Wobble Goal and Rotation 180 */
            robotUG.driveTrain.IMUDriveRotate(90, "Rotate 180*", this);

            telemetry.addLine("Rotate to Drop Goal");
            telemetry.addData("Desired Position (X, Y)", " \t\t( %1.1f, %1.1f)", robotUG.driveTrain.targetPoint.x, robotUG.driveTrain.targetPoint.y);
            telemetry.addData("Robot Position (X, Y)", " \t\t( %1.1f, %1.1f)", robotUG.driveTrain.robotFieldLocation.x, robotUG.driveTrain.robotFieldLocation.y);
            telemetry.addData("Robot Angles", " \t Desired: %1.1f, \t Actual: %1.1f", 90.0, robotUG.driveTrain.robotHeading);

            telemetry.update();// Review rotation

            robotUG.wobbleArm.dropWobble(this);
            haveBlueWobble1 = false;

            telemetry.addLine("Drop Goal");
            telemetry.addData("Wobble Goal Arm", " Command: %1.2f, Actual: %d", robotUG.wobbleArm.wobbleArmTargetAngle, robotUG.wobbleArm.wobbleArmTarget);
            telemetry.addData("Wobble Goal Servo", " \t Desired: %1.1f, \t Actual: %1.1f", 90.0, robotUG.driveTrain.robotHeading);
            telemetry.addLine("Check that Wobble Goal Has Been Dropped ...");

            telemetry.update();//Review wobble goal drop

            /* Coach Note: changing to go to high goal and to clear points and avoid rotation
             *
             */
            fieldPoints.clear();
            fieldPoints.add(new PursuitPoint(robotUG.driveTrain.robotFieldLocation.x, robotUG.driveTrain.robotFieldLocation.y));
            fieldPoints.add(new PursuitPoint(-48, -8));/* COACH CHANGED - for high goal - allow all options to align */
            fieldPoints.add(new PursuitPoint(-30, -8));/* COACH CHANGED - for high goal */
            //UPDATED TO MOVE BACK (more negative) to help shooter go in

            /* TEST CODE TO DRAW LINES FOR VIZ */
            for(int h=0;h<fieldPoints.size()-1;h++) {
                lines.add(new PursuitLines(fieldPoints.get(h).x, fieldPoints.get(h).y, fieldPoints.get(h+1).x, fieldPoints.get(h+1).y));
            }

            //TURN ON SHOOTER -- allow time to power up to full speed while driving
            robotUG.shooter.setShooter_Power(-1.0);
            updateIMU();

            /** Drive to and Shoot the Powershots */
            robotUG.driveTrain.drivePursuit(fieldPoints,this,"To PowerShot Shooting Position");

            /** Coach Note: need to rotate to face the PowerShot targets
             * see added lines below
             */


            robotUG.driveTrain.IMUDriveRotate(-90, "Rotate to Face Targets", this);/* COACH ADDED */

            // make sure you are at -90 angle
            telemetry.addLine(" VERIFY robot is aligned Shoot Target #1");
            telemetry.addData("Heading", " %1.1f",  robotUG.driveTrain.robotHeading);
            telemetry.addData("Location", " (%1.1f, %1.1f)",  robotUG.driveTrain.robotFieldLocation.x,robotUG.driveTrain.robotFieldLocation.y);
            telemetry.update();

            // shoot high goal
            //TURN ON CONVEYOR
            robotUG.conveyor.setMotion(Conveyor.motionType.UP);
            robotUG.collector.collectorWheel.setPower(-1.0);//need negative power to collector rings
            updateIMU();

            int counts = 0;
            while(counts < 50) {
                telemetry.addLine("Shoot High Goal x3");
                telemetry.addData("Counts", " %d", counts);
                telemetry.addData("Shooter Power", "  %1.2f",  robotUG.shooter.shooter_Power);
                telemetry.addData("Conveyor Power", " %1.1f",  robotUG.conveyor.conveyor_Power);
                telemetry.addLine("Press GamePad2 'BACK' once shooter fires ...");
                telemetry.update();
                robotUG.driveTrain.robotNavigator(this);
                updateIMU();
                counts+=1;
            }
            //TURN OFF CONVEYOR & COLLECTOR OFF
            robotUG.conveyor.setMotion(Conveyor.motionType.OFF);
            robotUG.collector.collectorWheel.setPower(0.0);

            updateIMU();

//            robotUG.driveTrain.IMUDriveFwdRight(DriveTrain.moveDirection.RightLeft, 7.5, -90, "Move Right 7.5 inch to shot", this);
//
//            //Make sure that robot is lined up for 2nd shot
//            telemetry.addLine(" VERIFY robot is aligned Shoot Target #2");
//            telemetry.addData("Heading", " %1.1f",  robotUG.driveTrain.robotHeading);
//            telemetry.addData("Location", " (%1.1f, %1.1f)",  robotUG.driveTrain.robotFieldLocation.x,robotUG.driveTrain.robotFieldLocation.y);
//            telemetry.update();
//            // shoot powershot
//            //TURN ON CONVEYOR
//            robotUG.conveyor.setMotion(Conveyor.motionType.UP);
//            updateIMU();
//
//            counts = 0;
//            while(counts < 10) {
//                telemetry.addLine("Shoot Target #2");
//                telemetry.addData("Counts", " %d", counts);
//                telemetry.addData("Shooter Power", "  %1.2f",  robotUG.shooter.shooter_Power);
//                telemetry.addData("Conveyor Power", " %1.1f",  robotUG.conveyor.conveyor_Power);
//                telemetry.addLine("Press GamePad2 'BACK' once shooter fires ...");
//                telemetry.update();
//                updateIMU();
//                counts+=1;
//            }
//            //TURN OFF CONVEYOR
//            robotUG.conveyor.setMotion(Conveyor.motionType.OFF);
//            updateIMU();
//            robotUG.driveTrain.IMUDriveFwdRight(DriveTrain.moveDirection.RightLeft, 7.5, -90, "Move Right 7.5 inch to shot", this);
//            //Make sure that robot is lined up for 2nd shot
//            telemetry.addLine(" VERIFY robot is aligned Shoot Target #3");
//            telemetry.addData("Heading", " %1.1f",  robotUG.driveTrain.robotHeading);
//            telemetry.addData("Location", " (%1.1f, %1.1f)",  robotUG.driveTrain.robotFieldLocation.x,robotUG.driveTrain.robotFieldLocation.y);
//            telemetry.update();
//            //TURN ON CONVEYOR
//            robotUG.conveyor.setMotion(Conveyor.motionType.UP);
//            updateIMU();
//            counts = 0;
//            while(counts < 10) {
//                telemetry.addLine("Shoot Target #3");
//                telemetry.addData("Counts", " %d", counts);
//                telemetry.addData("Shooter Power", "  %1.2f",  robotUG.shooter.shooter_Power);
//                telemetry.addData("Conveyor Power", " %1.1f",  robotUG.conveyor.conveyor_Power);
//                telemetry.addLine("Press GamePad2 'BACK' once shooter fires ...");
//                telemetry.update();
//                updateIMU();
//                counts+=1;
//            }
//            //TURN OFF CONVEYOR & SHOOTER
//            robotUG.conveyor.setMotion(Conveyor.motionType.OFF);
//            robotUG.shooter.setShooter_Power(0.0);
//            updateIMU();
            robotUG.driveTrain.IMUDriveFwdRight(DriveTrain.moveDirection.FwdBack, 12, -90, "Move Fwd ~6 in. to score points", this);
            /* INCREASED DRIVING DISTANCE BASED ON SHOOTING LOCATION*/
            //Telemetry output after driving completed
            telemetry.addData("Driving Completed", "...successfully?!?");

            telemetry.addLine("----------------------------------");

            telemetry.addData("Robot Heading", " Desired: %.2f, FieldNav: %.2f, RobotHeading: %.2f", robotUG.driveTrain.targetHeading, robotUG.driveTrain.robotFieldLocation.theta, robotUG.driveTrain.robotHeading);
            telemetry.addData("Robot Location", " Desired(X,Y): (%.2f,%.2f), Navigator(X,Y): (%.2f,%.2f)",
                    robotUG.driveTrain.targetPoint.x,robotUG.driveTrain.targetPoint.y, robotUG.driveTrain.robotFieldLocation.x, robotUG.driveTrain.robotFieldLocation.y);

            telemetry.addData("Motor Counts", "FL (%d) FR (%d) BR (%d) BL (%d)",
                    robotUG.driveTrain.flPrevious, robotUG.driveTrain.frPrevious, robotUG.driveTrain.brPrevious, robotUG.driveTrain.blPrevious);

            telemetry.addData("Final Pursuit Point", " (%.2f, %.2f)", fieldPoints.get(fieldPoints.size()-1).x,fieldPoints.get(fieldPoints.size()-1).y);
            telemetry.addLine("----------------------------------");
            telemetry.addLine("Observe telemetry and Press A to shutdown");
            telemetry.update();

            robotUG.shutdownAll();

            for(int h=0;h<fieldPoints.size()-1;h++) {
                lines.add(new PursuitLines(fieldPoints.get(h).x, fieldPoints.get(h).y, fieldPoints.get(h+1).x, fieldPoints.get(h+1).y));
            }
//  ADD LINES TO DISPLAY

            }
        if(robotNumber ==2){
            runtime.reset();


//            cons.DRIVE_POWER_LIMIT = 0.4;
//            cons.STEERING_POWER_LIMIT = 0.4;//was somewhere between 0.60 and 0.72 X DRIVE_POWER_LIMIT
//            cons.STEERING_POWER_GAIN = 0.05;//was 0.05
//
//            fieldPoints.clear();
//            lines.clear();
//            fieldPoints.add(new PursuitPoint(robotUG.driveTrain.robotFieldLocation.x  ,robotUG.driveTrain.robotFieldLocation.y));

            // Simple set of points - diamond or square
//
//            fieldPoints.add(new PursuitPoint(-36,-36));
//            fieldPoints.add(new PursuitPoint(-36,-24));
//            fieldPoints.add(new PursuitPoint(-40,-24));
//            fieldPoints.add(new PursuitPoint(-40,-12));
//            fieldPoints.add(new PursuitPoint(-60,-12));
//            fieldPoints.add(new PursuitPoint(-60,24));
//
//            for(int h=0;h<fieldPoints.size()-1;h++) {
//                lines.add(new PursuitLines(fieldPoints.get(h).x, fieldPoints.get(h).y, fieldPoints.get(h+1).x, fieldPoints.get(h+1).y));
//            }
//
//            telemetry.addData("Drive Power Limit Updated", cons.DRIVE_POWER_LIMIT);
//            telemetry.addData("Steering Power Limit Updated", cons.STEERING_POWER_LIMIT);
//            telemetry.addData("Steering Power Gain Updated", cons.STEERING_POWER_GAIN);
//            telemetry.addData("Forward Scale Factor", cons.adjForward);
//            telemetry.addData("Right Scale Factor", cons.adjRight);
//            telemetry.addData("Rotation Scale Factor", cons.adjRotate);

//            haveBlueWobble2 = true;
//            haveBlueRing = true;

//            robotUG.driveTrain.drivePursuit(fieldPoints, this, "Drive multi-lines");

//            haveBlueWobble2 = false;
            writeBR = true;
            writeBW2 = true;
        }
        if(robotNumber ==3) {
                runtime.reset();
            cons.DRIVE_POWER_LIMIT = 0.4;
            cons.STEERING_POWER_LIMIT = 0.4;//was somewhere between 0.60 and 0.72 X DRIVE_POWER_LIMIT
            cons.STEERING_POWER_GAIN = 0.03;//was 0.05

                fieldPoints.clear();
                lines.clear();
                fieldPoints.add(new PursuitPoint(robotUG.driveTrain.robotFieldLocation.x  ,robotUG.driveTrain.robotFieldLocation.y));
                // angled line
                fieldPoints.add(new PursuitPoint(24,-36));
                fieldPoints.add(new PursuitPoint(24,-24));
                fieldPoints.add(new PursuitPoint(30,-24));
                fieldPoints.add(new PursuitPoint(30,-12));
                fieldPoints.add(new PursuitPoint(60,-12));
                fieldPoints.add(new PursuitPoint(60,12));


                for(int h=0;h<fieldPoints.size()-1;h++) {
                    lines.add(new PursuitLines(fieldPoints.get(h).x, fieldPoints.get(h).y, fieldPoints.get(h+1).x, fieldPoints.get(h+1).y));
                }
                haveRedWobble1 = true;
                robotUG.driveTrain.drivePursuit(fieldPoints, this, "Drive multi-lines");

                telemetry.addData("Drive Power Limit Updated", cons.DRIVE_POWER_LIMIT);
                telemetry.addData("Steering Power Limit Updated", cons.STEERING_POWER_LIMIT);
                telemetry.addData("Steering Power Gain Updated", cons.STEERING_POWER_GAIN);
                telemetry.addData("Forward Scale Factor", cons.adjForward);
                telemetry.addData("Right Scale Factor", cons.adjRight);
                telemetry.addData("Rotation Scale Factor", cons.adjRotate);

                writeRR = true;
                writeRW1 = true;
            }
        if(robotNumber ==4 ) {
            runtime.reset();

            // for tests and smaller field trials the robot is initialized to (0,0) and 0.0 degrees
//                robotUG.driveTrain.initIMUtoAngle(-90.0); // NEEDS TO BE IN ACTUAL OpMode
//            cons.DRIVE_POWER_LIMIT = 0.4;
//            cons.STEERING_POWER_LIMIT = 0.4;//was somewhere between 0.60 and 0.72 X DRIVE_POWER_LIMIT
//            cons.STEERING_POWER_GAIN = 0.05;//was 0.05


//            fieldPoints.clear();
//            lines.clear();
//            fieldPoints.add(new PursuitPoint(robotUG.driveTrain.robotFieldLocation.x  ,robotUG.driveTrain.robotFieldLocation.y));

            // Simple set of points - diamond or square

//            fieldPoints.add(new PursuitPoint(36,-36));
//            fieldPoints.add(new PursuitPoint(36,-24));
//            fieldPoints.add(new PursuitPoint(40,-24));
//            fieldPoints.add(new PursuitPoint(40,-12));
//            fieldPoints.add(new PursuitPoint(60,-12));
//            fieldPoints.add(new PursuitPoint(60,24));

//            for(int h=0;h<fieldPoints.size()-1;h++) {
//                lines.add(new PursuitLines(fieldPoints.get(h).x, fieldPoints.get(h).y, fieldPoints.get(h+1).x, fieldPoints.get(h+1).y));
//            }
//
//            telemetry.addData("Drive Power Limit Updated", cons.DRIVE_POWER_LIMIT);
//            telemetry.addData("Steering Power Limit Updated", cons.STEERING_POWER_LIMIT);
//            telemetry.addData("Steering Power Gain Updated", cons.STEERING_POWER_GAIN);
//            telemetry.addData("Forward Scale Factor", cons.adjForward);
//            telemetry.addData("Right Scale Factor", cons.adjRight);
//            telemetry.addData("Rotation Scale Factor", cons.adjRotate);
//
//            haveRedWobble2 = true;
//
//            robotUG.driveTrain.drivePursuit(fieldPoints, this, "Drive multi-lines");
//
//            haveRedWobble2 = false;
            writeRW2 = true;
        }

    } //MAIN OpMode PROGRAM END


    //##############################################################################################
    // END ACTUAL CODE TO BE TESTED AS METHODS
    //##############################################################################################

    //Run Calculations like Autonomous OpMode
    public static void main(String []args)throws IOException {

// Code to setup the main program that runs offline, none of this is robot code
        OfflineOpModeLibs OffLibs = new OfflineOpModeLibs();
        OffLibs.testModeActive = true;
        // Prepare robot class for offline operation, must be run prior to copied runOpMode or init
        // Sets initial position and counters and initial array variables


        OffLibs.location = computer.KARL;//For Karl on Mac
        //OffLibs.location =  PC; for the HP Windows PC
//        OffLibs.location = computer.MAC;//For Caleb
//        OffLibs.location = computer.WILL;//For William


        for(int h = 1; h<5;h++) {

            OffLibs.robotNumber = h;//!!!!!!!!!!!!!1!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            OffLibs.foundationPosChange = 0;// 26 for unmoved FoundationOpMode, 0 for moved FoundationOpMode
            OffLibs.insideOutside = 0;// 0 for Inside, 24 for Outside
            OffLibs.foundationInOut = 26;// 0 for Inside, 26 for Outside
            if(h ==1 || h==2) {
                OffLibs.sideColor = 1;// + for Blue for robots 1 & 2
            }
            else {
                OffLibs.sideColor = -1;// - for Red for robots 3 & 4

            }
            OffLibs.prepOpMode();

//calls code input by programmer into runAutonomous method that comes from main runOpMode
            OffLibs.runOpMode();
// Lines below are to capture the array data and output
            OffLibs.extractArrayData();
            int countVar = Math.max(size, OffLibs.IMUCounter);

            fos = new FileOutputStream(OffLibs.fileLocation + String.format("Robot%dOnField.txt", OffLibs.robotNumber));// Path to directory for IntelliJ code
            OffLibs.fc.writeFieldAsText(fos, OffLibs.robotUG.driveTrain.imu.RobotPoints, countVar);

            fos = new FileOutputStream(OffLibs.fileLocation + String.format("Robot%dAccessories.txt", OffLibs.robotNumber));// Path to directory for IntelliJ code
            OffLibs.writeExtrasToFile(fos);

            fos = new FileOutputStream(OffLibs.fileLocation + String.format("Robot%dPath.txt", OffLibs.robotNumber));// Path to directory for IntelliJ code
            OffLibs.writePath(fos, OffLibs.lines, OffLibs.lines.size());

            if(OffLibs.robotNumber == 1) {
                fos = new FileOutputStream(OffLibs.fileLocation + String.format("Robot%dPursuit.txt", OffLibs.robotNumber));// Path to directory for IntelliJ code
                OffLibs.fc.writeFieldAsText(fos, OffLibs.fc.PursuitPoints, countVar);
                fos = new FileOutputStream(OffLibs.fileLocation + String.format("Robot%dNav.txt", OffLibs.robotNumber));// Path to directory for IntelliJ code
                OffLibs.fc.writeFieldAsText(fos, OffLibs.fc.NavPoints, countVar);

            }

            if (OffLibs.writeRR) {
                fos = new FileOutputStream(fileLocation + "RedRing.txt");// Path to directory for IntelliJ code
                OffLibs.fc.writeFieldAsText(fos, OffLibs.fc.RedRingPoints, countVar);
                fos = new FileOutputStream(fileLocation + "RingSet.txt");// Path to directory for IntelliJ code
                OffLibs.fc.writeRingType(fos);

            }
            if (OffLibs.writeBR) {
                fos = new FileOutputStream(fileLocation + "BlueRing.txt");// Path to directory for IntelliJ code
                OffLibs.fc.writeFieldAsText(fos, OffLibs.fc.BlueRingPoints, countVar);
                fos = new FileOutputStream(fileLocation + "RingSet.txt");// Path to directory for IntelliJ code
                OffLibs.fc.writeRingType(fos);
            }
            if (OffLibs.writeRW1) {
                fos = new FileOutputStream(fileLocation + "RedWobble1.txt");// Path to directory for IntelliJ code
                OffLibs.fc.writeFieldAsText(fos, OffLibs.fc.RedWobble1Points, countVar);
            }
            if (OffLibs.writeRW2){
                fos = new FileOutputStream(fileLocation + "RedWobble2.txt");// Path to directory for IntelliJ code
                OffLibs.fc.writeFieldAsText(fos, OffLibs.fc.RedWobble2Points, countVar);
            }
            if (OffLibs.writeBW1 ) {
                fos = new FileOutputStream(fileLocation + "BlueWobble1.txt");// Path to directory for IntelliJ code
                OffLibs.fc.writeFieldAsText(fos, OffLibs.fc.BlueWobble1Points, countVar);
            }
            if ( OffLibs.writeBW2){
                fos = new FileOutputStream(fileLocation + "BlueWobble2.txt");// Path to directory for IntelliJ code
                OffLibs.fc.writeFieldAsText(fos, OffLibs.fc.BlueWobble2Points, countVar);
            }

            OffLibs.telemetry.addData("Total Number of Time Steps", "%d", OffLibs.IMUCounter);
            OffLibs.telemetry.addData("Completed", "Robot: %d, side: %.0f", h, OffLibs.sideColor);
                        OffLibs.telemetry.addLine("===================================");
            OffLibs.telemetry.addLine(" ");
            OffLibs.telemetry.update();

        }
    }

}