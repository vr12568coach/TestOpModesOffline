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

    boolean writeBF = false;
    boolean writeRF = false;
    boolean writeBS1 = false;
    boolean writeBS2 = false;
    boolean writeRS1 = false;
    boolean writeRS2 = false;

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
    int[] jackIMU = new int[size];
    double[] gripIMU = new double[size];
    double[] blueStoneServoIMU = new double[size];
    double[] redStoneServoIMU = new double[size];


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
    public double timeStep = 135;//determined a fixed time step (in milliseconds) so that faster speeds will show shorter time to distance
    // relly not the time step but the speed of the motor at maxpower in counts/second
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
        jackIMU = robotUG.driveTrain.imu.jackDirection;
        gripIMU  = robotUG.driveTrain.imu.gripperWidth;
        blueStoneServoIMU  = robotUG.driveTrain.imu.blueServoArray;
        redStoneServoIMU  = robotUG.driveTrain.imu.redServoArray;


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
        Arrays.fill(jackIMU,IMUCounter,(size),jackIMU[IMUCounter-1]);
        Arrays.fill(gripIMU,IMUCounter,(size),gripIMU[IMUCounter-1]);
        Arrays.fill(blueStoneServoIMU,IMUCounter,(size),blueStoneServoIMU[IMUCounter-1]);
        Arrays.fill(redStoneServoIMU,IMUCounter,(size),redStoneServoIMU[IMUCounter-1]);

        double deltaTime = (timeArray[1] - timeArray[0]);
        for(int k = IMUCounter-1; k < size;k++){// needed to reduce counter by 1 -- means there is an extra count somewhere
            timeArray[k] = timeArray[k-1] + deltaTime;
            robotUG.driveTrain.imu.RobotPoints.add(robotUG.driveTrain.imu.RobotPoints.get(k-1));
            robotUG.driveTrain.imu.GripperPoints.add(robotUG.driveTrain.imu.GripperPoints.get(k-1));


            fc.BlueFoundationPoints.add(fc.BlueFoundationPoints.get(k-1));
            fc.RedFoundationPoints.add(fc.RedFoundationPoints.get(k-1));

            fc.RedSkyStone1Points.add(fc.RedSkyStone1Points.get(k-1));
            fc.BlueSkyStone1Points.add(fc.BlueSkyStone1Points.get(k-1));

            fc.BlueSkyStone2Points.add(fc.BlueSkyStone2Points.get(k-1));
            fc.RedSkyStone2Points.add(fc.RedSkyStone2Points.get(k-1));
            fc.PursuitPoints.add(fc.PursuitPoints.get(k-1));
            fc.NavPoints1.add(fc.NavPoints1.get(k-1));
            fc.NavPoints2.add(fc.NavPoints2.get(k-1));

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
                        timeArray[i], flIMU[i], frIMU[i], brIMU[i], blIMU[i], jackIMU[i],
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
//          Only writing out the FieldLocation X,Y, Theta as formatted for reading in, Used for foundations,stones, robot, and gripper
            for (int j = 0; j < size; j++) {
                // writes the data as text for each value in the array
                osw.write(Integer.toString(jackIMU[j])+"\t");   // Jack motion
                osw.write(Double.toString(blueStoneServoIMU[j])+"\t");   // Blue Stone Servo
                osw.write(Double.toString(redStoneServoIMU[j])+"\t");   // Red Stone Servo
                osw.write(Double.toString(gripIMU[j])+"\n");   // gripper motion
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
                osw.write(Double.toString(lines.get(j).y1)+"\t");   // FPath Y1 position on field in inches
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

//        robotUG.driveTrain.imu.blueStoneServoPos = robotUG.driveTrain.stoneServoArm.getPosition();
//        robotUG.driveTrain.imu.redStoneServoPos = robotUG.driveTrain.servoRedStoneGrab.getPosition();

        fc.updateField(this);

        robotSeeStone= fc.stoneFound;


        if(haveBlueFoundation){writeBF = true;}
        if(haveRedFoundation){writeRF = true;}
        if(haveBlueSkyStone1){writeBS1 = true;}
        if(haveBlueSkyStone2){writeBS2 = true;}
        if(haveRedSkyStone1){writeRS1 = true;}
        if(haveRedSkyStone2){writeRS2 = true;}

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


       stoneSelect = 2;
       fc = new FieldConfiguration(stoneSelect);//KS added 12/20 to set stone position

       haveBlueFoundation = false;
       haveRedFoundation= false;
       haveBlueSkyStone1= false;
       haveBlueSkyStone2= false;
       haveRedSkyStone1= false;
       haveRedSkyStone2= false;

       writeBF = false;
       writeRF = false;
       writeBS1 = false;
       writeBS2 = false;
       writeRS1 = false;
       writeRS2 = false;

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

//       initializeMiniBot();
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
        * items that are 1 = true will be configured to the robot
        */
       // HW ELEMENTS *****************    DriveTrain  Shooter  Conveyor	WobbleArm	Collector
       boolean[] configArray = new boolean[]{ true, 	false, 	false, 		false, 		false};

       robotUG = new HardwareRobotMulti(this, configArray, testModeActive);

           //***********************************************************
       //Code that needs to be Kept in init to initialize functions
       //***********************************************************

       robotUG.driveTrain.imu.timeStep = timeStep;
       robotUG.driveTrain.frontLeft.timeStep = timeStep;
       robotUG.driveTrain.frontRight.timeStep = timeStep;
       robotUG.driveTrain.backRight.timeStep = timeStep;
       robotUG.driveTrain.backLeft.timeStep = timeStep;

//       robotUG.driveTrain.jack.timeStep = timeStep;
//       robotUG.driveTrain.gripper.timeStep = timeStep;

       fc.RedFoundationPoints.clear();
       fc.BlueFoundationPoints.clear();
       fc.BlueSkyStone1Points.clear();
       fc.RedSkyStone1Points.clear();
       fc.BlueSkyStone2Points.clear();
       fc.RedSkyStone2Points.clear();
       fc.updateField(this);

       robotUG.driveTrain.imu.GripperPoints.clear();
//       robotUG.driveTrain.imu.GripperPoints.add(new FieldLocation(robotUG.driveTrain.imu.gripperX, robotUG.driveTrain.imu.gripperY, robotUG.driveTrain.imu.gripperTheta));

       robotUG.driveTrain.imu.RobotPoints.clear();
//       robotUG.driveTrain.imu.RobotPoints.add(new FieldLocation(robotUG.driveTrain.imu.robotOnField.x, robotUG.driveTrain.imu.robotOnField.y, robotUG.driveTrain.imu.robotOnField.theta));

       //Setting counter to capture array data is unique to offline running of code
       counter = 1;
       robotUG.driveTrain.imu.counter = counter;
//
//       robotNumber = 1;//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//       foundationPosChange = 0;// 26 for unmoved FoundationOpMode, 0 for moved FoundationOpMode
//       insideOutside = 0;// 0 for Inside, 24 for Outside
//       foundationInOut = 26;// 0 for Inside, 26 for Outside
//       sideColor = 1;// + for Blue, - for Red

       if(robotNumber == 1) {
           cons.DRIVE_POWER_LIMIT = 1.0;
           robotUG.driveTrain.frontLeft.motorTol=1.0;
           robotUG.driveTrain.frontRight.motorTol=1.0;
           robotUG.driveTrain.backRight.motorTol=1.0;
           robotUG.driveTrain.backLeft.motorTol=1.0;
           //field angle orientation is + = CCW , while robot frame is + = CW
           robotUG.driveTrain.imu.robotOnField.x = 0;//initial x position on field in inches (Added 2 inches for robot 7" to wheel center vs. 9")
           robotUG.driveTrain.imu.robotOnField.y = 0;//initial y position on field in inches
           robotUG.driveTrain.imu.robotOnField.theta = 0;//initial robot angle orientation on field in degrees from EAST
           robotUG.driveTrain.imu.priorAngle = 0;//initial robot angle orientation on field in degrees from EAST
           robotUG.driveTrain.imu.fakeAngle = 0;//initial robot angle orientation on field in degrees from EAST
           robotUG.driveTrain.robotHeading = -robotUG.driveTrain.imu.fakeAngle;//initial robot angle orientation on field in degrees from EAST


//           robotUG.driveTrain.robotLocationNav1 = robotUG.driveTrain.imu.robotOnField;
           robotUG.driveTrain.robotLocationNav2 = robotUG.driveTrain.imu.robotOnField;

           robotUG.driveTrain.robotFieldLocationNav2 = robotUG.driveTrain.imu.robotOnField;
//           robotUG.driveTrain.robotFieldLocationNav1 = robotUG.driveTrain.imu.robotOnField;

           robotUG.driveTrain.targetPoint.setPoint(robotUG.driveTrain.imu.robotOnField.x ,robotUG.driveTrain.imu.robotOnField.y);

           telemetry.addData("Robot Number ", "%d",robotNumber);
           telemetry.addData("drivePowerLimit ", "%.2f",cons.DRIVE_POWER_LIMIT);

           telemetry.update();
       }

       if(robotNumber == 2) {

//           cons.DRIVE_POWER_LIMIT = 0.75;
           robotUG.driveTrain.frontLeft.motorTol=1.0;
           robotUG.driveTrain.frontRight.motorTol=1.0;
           robotUG.driveTrain.backRight.motorTol=1.0;
           robotUG.driveTrain.backLeft.motorTol=1.0;
           //field angle orientation is + = CCW , while robot frame is + = CW
           robotUG.driveTrain.imu.robotOnField.x = -65;//initial x position on field in inches (Added 2 inches for robot 7" to wheel center vs. 9")
           robotUG.driveTrain.imu.robotOnField.y = 58;//initial y position on field in inches (WAS 48)
           robotUG.driveTrain.imu.robotOnField.theta = 0;//initial robot angle orientation on field in degrees from EAST (WAS 180 for backing to foundation)
           robotUG.driveTrain.imu.priorAngle = 0;//initial robot angle orientation on field in degrees from EAST (WAS 180 for backing to foundation)
           robotUG.driveTrain.imu.fakeAngle = 0;//initial robot angle orientation on field in degrees from EAST (WAS 180 for backing to foundation)
           robotUG.driveTrain.robotHeading = -robotUG.driveTrain.imu.fakeAngle;//initial robot angle orientation on field in degrees from EAST
           telemetry.addData("Robot Number ", "%d",robotNumber);
           telemetry.addData("drivePowerLimit ", "%.2f",cons.DRIVE_POWER_LIMIT);

           telemetry.update();
       }

       if(robotNumber == 3) {
//           cons.DRIVE_POWER_LIMIT = 0.75;
           robotUG.driveTrain.frontLeft.motorTol=1.0;
           robotUG.driveTrain.frontRight.motorTol=1.0;
           robotUG.driveTrain.backRight.motorTol=1.0;
           robotUG.driveTrain.backLeft.motorTol=1.0;
           //field angle orientation is + = CCW , while robot frame is + = CW
           robotUG.driveTrain.imu.robotOnField.x = 65;//initial x position on field in inches (Added 2 inches for robot 7" to wheel center vs. 9")
           robotUG.driveTrain.imu.robotOnField.y = -36;//initial y position on field in inches
           robotUG.driveTrain.imu.robotOnField.theta = 180;//initial robot angle orientation on field in degrees from EAST
           robotUG.driveTrain.imu.priorAngle = 180;//initial robot angle orientation on field in degrees from EAST
           robotUG.driveTrain.imu.fakeAngle = 180;//initial robot angle orientation on field in degrees from EAST
           robotUG.driveTrain.robotHeading = -robotUG.driveTrain.imu.fakeAngle;//initial robot angle orientation on field in degrees from EAST
           telemetry.addData("Robot Number ", "%d",robotNumber);
           telemetry.addData("drivePowerLimit ", "%.2f",cons.DRIVE_POWER_LIMIT);

           telemetry.update();

       }

       if(robotNumber == 4) {
//           cons.DRIVE_POWER_LIMIT = 0.75;
           robotUG.driveTrain.frontLeft.motorTol=1.0;
           robotUG.driveTrain.frontRight.motorTol=1.0;
           robotUG.driveTrain.backRight.motorTol=1.0;
           robotUG.driveTrain.backLeft.motorTol=1.0;
           robotUG.driveTrain.imu.robotOnField.x = 65;//initial x position on field in inches (Added 2 inches for robot 7" to wheel center vs. 9")
           robotUG.driveTrain.imu.robotOnField.y = 58;//initial y position on field in inches (WAS 48)
           robotUG.driveTrain.imu.robotOnField.theta = 180;//initial robot angle orientation on field in degrees from EAST (WAS 0 for backing to foundation)
           robotUG.driveTrain.imu.priorAngle = 180;//initial robot angle orientation on field in degrees from EAST (WAS 0 for backing to foundation)
           robotUG.driveTrain.imu.fakeAngle = 180;//initial robot angle orientation on field in degrees from EAST (WAS 0 for backing to foundation)
           robotUG.driveTrain.robotHeading = -robotUG.driveTrain.imu.fakeAngle;//initial robot angle orientation on field in degrees from EAST
           telemetry.addData("Robot Number ", "%d",robotNumber);
           telemetry.addData("drivePowerLimit ", "%.2f",cons.DRIVE_POWER_LIMIT);

           telemetry.update();
       }

       robotUG.driveTrain.angleUnWrap();
       robotUG.driveTrain.offset = robotUG.driveTrain.robotHeading;
       robotUG.driveTrain.robotHeading-=robotUG.driveTrain.offset;//set robotHeading = 0 for all opModes regardless of position, but track actual angle in IMU
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

//        initialize();
//        runtime.reset();
//
//        if(robotNumber ==1 || robotNumber ==3) {

            if (robotNumber == 1) {
                runtime.reset();


                // for tests and smaller field trials the robot is initialized to (0,0) and 0.0 degrees
//                robotUG.driveTrain.robotLocationNav1 = robotUG.driveTrain.imu.robotOnField;
                robotUG.driveTrain.robotLocationNav2 = robotUG.driveTrain.imu.robotOnField;
//                robotUG.driveTrain.robotFieldLocationNav1 = robotUG.driveTrain.imu.robotOnField;
                robotUG.driveTrain.robotFieldLocationNav2 = robotUG.driveTrain.imu.robotOnField;

                robotUG.driveTrain.robotHeading = -robotUG.driveTrain.imu.fakeAngle;
                robotUG.driveTrain.priorAngle = robotUG.driveTrain.robotHeading;
                cons.DRIVE_POWER_LIMIT = 0.05;
                cons.STEERING_POWER_LIMIT = cons.DRIVE_POWER_LIMIT*0.70;//was somewhere between 0.60 and 0.72 X DRIVE_POWER_LIMIT
                cons.STEERING_POWER_GAIN = 0.05;//was 0.05

//                ArrayList<PursuitPoint> pathPoints = new ArrayList<>();
//                pathPoints= fieldPoints;
//
//                pathPoints.add(new PursuitPoint(robotUG.driveTrain.robotLocationNav2.x  ,robotUG.driveTrain.robotLocationNav2.y));
                // Simple set of points - diamond
//                pathPoints.add(new PursuitPoint(30,30));
//                pathPoints.add(new PursuitPoint(0,60));
//                pathPoints.add(new PursuitPoint(-30,30));
//                pathPoints.add(new PursuitPoint(-6,6));

                // Slalom course - doesn't get to 180 so should be good
//                pathPoints.add(new PursuitPoint(18,0));
//                pathPoints.add(new PursuitPoint(18,72));
//                pathPoints.add(new PursuitPoint(36,72));
//                pathPoints.add(new PursuitPoint(36,0));
//                pathPoints.add(new PursuitPoint(54, 0));
//                pathPoints.add(new PursuitPoint(54,72));
//                pathPoints.add(new PursuitPoint(72,72));
//                pathPoints.add(new PursuitPoint(72,0));

                // angled line
//                pathPoints.add(new PursuitPoint(30,30));
//                pathPoints.add(new PursuitPoint(45,35));
//                pathPoints.add(new PursuitPoint(60,70));




//                for(int h=0;h<pathPoints.size()-1;h++) {
//                    lines.add(new PursuitLines(pathPoints.get(h).x, pathPoints.get(h).y, pathPoints.get(h+1).x, pathPoints.get(h+1).y));
//                }
                telemetry.addData("Drive Power Limit Updated", cons.DRIVE_POWER_LIMIT);
                telemetry.addData("Steering Power Limit Updated", cons.STEERING_POWER_LIMIT);
                telemetry.addData("Steering Power Gain Updated", cons.STEERING_POWER_GAIN);
                telemetry.addData("Forward Scale Factor", cons.adjForward);
                telemetry.addData("Right Scale Factor", cons.adjRight);
                telemetry.addData("Rotation Scale Factor", cons.adjRotate);

//                robotUG.driveTrain.drivePursuit(pathPoints, this, "Drive multi-lines");

                //Drive set distances and report Navigation
                robotUG.driveTrain.IMUDriveFwdRight(DriveTrain.moveDirection.FwdBack,6,0,"FWD 60",this);
                robotUG.driveTrain.IMUDriveRotate(-90,"CCW 90",this);
                robotUG.driveTrain.IMUDriveFwdRight(DriveTrain.moveDirection.FwdBack,6,-90,"FWD 60",this);
                robotUG.driveTrain.IMUDriveRotate(-180,"CCW 90",this);
                robotUG.driveTrain.IMUDriveFwdRight(DriveTrain.moveDirection.FwdBack,12,-180,"FWD 120",this);
                robotUG.driveTrain.IMUDriveRotate(-270,"CCW 90",this);
                robotUG.driveTrain.IMUDriveFwdRight(DriveTrain.moveDirection.FwdBack,12,-270,"FWD 120",this);


            }
//
//            if(robotNumber ==1) {
                writeBS1 = true;
//            }
            if(robotNumber ==3) {
                runtime.reset();
                insideOutside = 0;// 0 for Inside, 24 for Outside

                robotUG.driveTrain.initIMU(this);

//                fwdToTwoStone();
//
//                vuforiaStoneLocateOffline(stoneSelect); //REPLACES THE ACTUAL VUFORIA CODE
//
//                goToStone();
//
//                takeStone1();
//
//                getStone2();
//
//                takeStone2();
//
//                twoStonePark();

                telemetry.addLine("OpMode Complete");
                telemetry.update();
                writeRS1 = true;
//                // Added if statements to write foundation files to clear old data
            }
//        }
//
       if(robotNumber ==2){
            if(foundationPosChange == 26) {
//               robotUG.driveTrain.IMUDriveFwdRight(HardwarerobotUG.driveTrain.moveDirection.RightLeft,50*sideColor, 0, "RIGHT/LEFT 50 inches",this);

            }
            if(foundationPosChange != 26) {
                runtime.reset();
                insideOutside = 24;// 0 for Inside, 24 for Outside

                robotUG.driveTrain.initIMU(this);

//                grabFoundation();
//
//                foundationInCorner();

                telemetry.addLine("OpMode Complete");
                telemetry.update();

            }
            writeBF = true;
        }
        if(robotNumber ==4 ) {
            if(foundationPosChange == 26){
//                robotUG.driveTrain.IMUDriveFwdRight(HardwarerobotUG.driveTrain.moveDirection.RightLeft,50*sideColor, 0, "RIGHT/LEFT 50 inches",this);
            }

            if(foundationPosChange != 26) {
                runtime.reset();
                insideOutside = 24;// 0 for Inside, 24 for Outside

                robotUG.driveTrain.initIMU(this);

//                grabFoundation();
//
//                foundationInCorner();

                telemetry.addLine("OpMode Complete");
                telemetry.update();
            }
            writeRF = true;

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

            fos = new FileOutputStream(OffLibs.fileLocation + String.format("Robot%dGripper.txt", OffLibs.robotNumber));// Path to directory for IntelliJ code
            OffLibs.fc.writeFieldAsText(fos, OffLibs.robotUG.driveTrain.imu.GripperPoints, countVar);

            if(OffLibs.robotNumber == 1) {
                fos = new FileOutputStream(OffLibs.fileLocation + String.format("Robot%dPursuit.txt", OffLibs.robotNumber));// Path to directory for IntelliJ code
                OffLibs.fc.writeFieldAsText(fos, OffLibs.fc.PursuitPoints, countVar);
                fos = new FileOutputStream(OffLibs.fileLocation + String.format("Robot%dNav1.txt", OffLibs.robotNumber));// Path to directory for IntelliJ code
                OffLibs.fc.writeFieldAsText(fos, OffLibs.fc.NavPoints1, countVar);
                fos = new FileOutputStream(OffLibs.fileLocation + String.format("Robot%dNav2.txt", OffLibs.robotNumber));// Path to directory for IntelliJ code
                OffLibs.fc.writeFieldAsText(fos, OffLibs.fc.NavPoints2, countVar);
                fos = new FileOutputStream(OffLibs.fileLocation + String.format("Robot%dPath.txt", OffLibs.robotNumber));// Path to directory for IntelliJ code
                OffLibs.writePath(fos, OffLibs.lines, OffLibs.lines.size());
            }

            if (OffLibs.writeRF) {
                fos = new FileOutputStream(fileLocation + "RedFoundation.txt");// Path to directory for IntelliJ code
                OffLibs.fc.writeFieldAsText(fos, OffLibs.fc.RedFoundationPoints, countVar);
            }
            if (OffLibs.writeBF) {
                fos = new FileOutputStream(fileLocation + "BlueFoundation.txt");// Path to directory for IntelliJ code
                OffLibs.fc.writeFieldAsText(fos, OffLibs.fc.BlueFoundationPoints, countVar);
            }
            if (OffLibs.writeRS1 || OffLibs.writeRS2) {
                fos = new FileOutputStream(fileLocation + "RedSkyStone1.txt");// Path to directory for IntelliJ code
                OffLibs.fc.writeFieldAsText(fos, OffLibs.fc.RedSkyStone1Points, countVar);

                fos = new FileOutputStream(fileLocation + "RedSkyStone2.txt");// Path to directory for IntelliJ code
                OffLibs.fc.writeFieldAsText(fos, OffLibs.fc.RedSkyStone2Points, countVar);
            }
            if (OffLibs.writeBS1 || OffLibs.writeBS2) {
                fos = new FileOutputStream(fileLocation + "BlueSkyStone1.txt");// Path to directory for IntelliJ code
                OffLibs.fc.writeFieldAsText(fos, OffLibs.fc.BlueSkyStone1Points, countVar);

                fos = new FileOutputStream(fileLocation + "BlueSkyStone2.txt");// Path to directory for IntelliJ code
                OffLibs.fc.writeFieldAsText(fos, OffLibs.fc.BlueSkyStone2Points, countVar);
            }

            OffLibs.telemetry.addData("Total Number of Time Steps", "%d", OffLibs.IMUCounter);
            OffLibs.telemetry.addData("Completed", "Robot: %d, side: %.0f", h, OffLibs.sideColor);
                        OffLibs.telemetry.addLine("===================================");
            OffLibs.telemetry.addLine(" ");
            OffLibs.telemetry.update();

        }
    }

}