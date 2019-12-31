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

package TestOpModesOffline;


import java.io.DataOutputStream;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.ArrayList;
import java.util.Arrays;

import Skystone_14999.HarwareConfig.HardwareBilly;
import Skystone_14999.OpModes.Autonomous.AutoGenerated.BlueFoundOut;
import Skystone_14999.OpModes.Autonomous.AutoGenerated.BlueStoneIn;
import Skystone_14999.OpModes.Autonomous.AutoGenerated.RedFoundOut;
import Skystone_14999.OpModes.Autonomous.AutoGenerated.RedStoneIn;
import Skystone_14999.OpModes.Autonomous.BasicAuto;
import Skystone_14999.OpModes.Autonomous.DoubleSkyStoneDP_InB;

//import com.qualcomm.robotcore.hardware.HardwareMap;

//************************************************************************************************
//** THIS FILE INSTANTIATES OpModes FROM THE OpModeCSV.csv FILE AND RUNS THEM FOR VISUALIZATION **
//************************************************************************************************

public class OfflineOpModeRunFile extends BasicAuto {


//****************************************
//DECLARE VARIABLES FROM CODE BEING TESTED
//****************************************

//****************************************
// DECLARE VARIABLES NEEDED FOR TEST CODE
//****************************************
    private FieldConfiguration fc = new FieldConfiguration();
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

    public BasicAuto autoOpMode = new BasicAuto();

    /* Constructor */
    public OfflineOpModeRunFile(BasicAuto inputOpMode){
        autoOpMode = inputOpMode;
    };
    public OfflineOpModeRunFile(){

    };


    //----------------------------------------------------------------------------------------------
    // TEST MODE CODE ONLY - EXTRACTS ARRAY DATA FROM classes
    //----------------------------------------------------------------------------------------------
    public void extractArrayData(){


        flIMU = autoOpMode.Billy.imu.flArray;
        frIMU = autoOpMode.Billy.imu.frArray;
        brIMU = autoOpMode.Billy.imu.brArray;
        blIMU = autoOpMode.Billy.imu.blArray;
        jackIMU = autoOpMode.Billy.imu.jackDirection;
        gripIMU  = autoOpMode.Billy.imu.gripperWidth;
        blueStoneServoIMU  = autoOpMode.Billy.imu.blueServoArray;
        redStoneServoIMU  = autoOpMode.Billy.imu.redServoArray;


        timeArray = autoOpMode.Billy.imu.timeArray;

        arrayRobotX = autoOpMode.Billy.imu.robotXArray;
        arrayRobotY = autoOpMode.Billy.imu.robotYArray;
        arrayRobotDist = autoOpMode.Billy.imu.robotDistArray;
        arrayRobotAngle = autoOpMode.Billy.imu.robotAngleArray;
        IMUCounter = autoOpMode.Billy.imu.counter;

        arrayFLBR = autoOpMode.Billy.imu.FLBRArray;
        arrayFRBL = autoOpMode.Billy.imu.FRBLArray;

//        robotX = autoOpMode.Billy.imu.robotX;
//        robotY = autoOpMode.Billy.imu.robotY;
//        robotDist = autoOpMode.Billy.imu.robotDist;
//        robotAngle = (double) autoOpMode.Billy.imu.fakeAngle;
//        arrayFieldX = autoOpMode.Billy.imu.fieldXArray;
//        arrayFieldY= autoOpMode.Billy.imu.fieldYArray;
//        arrayFieldDist = autoOpMode.Billy.imu.fieldDistArray;
//
//        Arrays.fill(flIMU,IMUCounter,(size), flIMU[IMUCounter-1]);
//        Arrays.fill(frIMU,IMUCounter,(size), frIMU[IMUCounter-1]);
//        Arrays.fill(brIMU,IMUCounter,(size), brIMU[IMUCounter-1]);
//        Arrays.fill(blIMU,IMUCounter,(size), blIMU[IMUCounter-1]);
//
//
//        Arrays.fill(arrayFLBR,IMUCounter,(size),arrayFLBR[IMUCounter-1]);
//        Arrays.fill(arrayFRBL,IMUCounter,(size),arrayFRBL[IMUCounter-1]);
//
//        Arrays.fill(arrayRobotX,IMUCounter,(size),arrayRobotX[IMUCounter-1]);
//        Arrays.fill(arrayRobotY,IMUCounter,(size),arrayRobotY[IMUCounter-1]);
        Arrays.fill(arrayRobotDist,IMUCounter,(size),arrayRobotDist[IMUCounter-1]);
        Arrays.fill(arrayRobotAngle,IMUCounter,(size),arrayRobotAngle[IMUCounter-1]);

//        Arrays.fill(RX,counter,(size),RX[counter-1]);
//        Arrays.fill(RY,counter,(size),RY[counter-1]);
//
//        Arrays.fill(arrayFieldX,IMUCounter,(size),arrayFieldX[IMUCounter-1]);
//        Arrays.fill(arrayFieldY,IMUCounter,(size),arrayFieldY[IMUCounter-1]);
        Arrays.fill(arrayFieldDist,IMUCounter,(size),arrayFieldDist[IMUCounter-1]);
        Arrays.fill(jackIMU,IMUCounter,(size),jackIMU[IMUCounter-1]);
        Arrays.fill(gripIMU,IMUCounter,(size),gripIMU[IMUCounter-1]);
        Arrays.fill(blueStoneServoIMU,IMUCounter,(size),blueStoneServoIMU[IMUCounter-1]);
        Arrays.fill(redStoneServoIMU,IMUCounter,(size),redStoneServoIMU[IMUCounter-1]);

        double deltaTime = (timeArray[1] - timeArray[0]);
        for(int k = IMUCounter-1; k < size;k++){// needed to reduce counter by 1 -- means there is an extra count somewhere
            timeArray[k] = timeArray[k-1] + deltaTime;
            autoOpMode.Billy.imu.RobotPoints.add(autoOpMode.Billy.imu.RobotPoints.get(k-1));
            autoOpMode.Billy.imu.GripperPoints.add(autoOpMode.Billy.imu.GripperPoints.get(k-1));


            autoOpMode.fc.BlueFoundationPoints.add(autoOpMode.fc.BlueFoundationPoints.get(k-1));
            autoOpMode.fc.RedFoundationPoints.add(autoOpMode.fc.RedFoundationPoints.get(k-1));

            autoOpMode.fc.RedSkyStone1Points.add(autoOpMode.fc.RedSkyStone1Points.get(k-1));
            autoOpMode.fc.BlueSkyStone1Points.add(autoOpMode.fc.BlueSkyStone1Points.get(k-1));

            autoOpMode.fc.BlueSkyStone2Points.add(autoOpMode.fc.BlueSkyStone2Points.get(k-1));
            autoOpMode.fc.RedSkyStone2Points.add(autoOpMode.fc.RedSkyStone2Points.get(k-1));
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


    //----------------------------------------------------------------------------------------------
    // TEST MODE CODE ONLY - UPDATES FAKE IMU
    //----------------------------------------------------------------------------------------------
    @Override
    public void updateIMU(){
        //Move this imu portion to robot update method
        autoOpMode.Billy.imu.flCnt = autoOpMode.Billy.frontLeft.getCurrentPosition();
        autoOpMode.Billy.imu.frCnt = autoOpMode.Billy.frontRight.getCurrentPosition();
        autoOpMode.Billy.imu.brCnt = autoOpMode.Billy.backRight.getCurrentPosition();
        autoOpMode.Billy.imu.blCnt = autoOpMode.Billy.backLeft.getCurrentPosition();

        autoOpMode.Billy.imu.blueStoneServoPos = autoOpMode.Billy.stoneServoArm.getPosition();
//        autoOpMode.Billy.imu.redStoneServoPos = autoOpMode.Billy.servoRedStoneGrab.getPosition();

        autoOpMode.fc.updateField(autoOpMode);

        robotSeeStone= autoOpMode.fc.stoneFound;

        if(autoOpMode.haveBlueFoundation){writeBF = true;}
        if(autoOpMode.haveRedFoundation){writeRF = true;}
        if(autoOpMode.haveBlueSkyStone1){writeBS1 = true;}
        if(autoOpMode.haveBlueSkyStone2){writeBS2 = true;}
        if(autoOpMode.haveRedSkyStone1){writeRS1 = true;}
        if(autoOpMode.haveRedSkyStone2){writeRS2 = true;}

        try {
//
                if (IMUCounter >= size) {
                    int a = 1 / 0;
                }
        } catch (ArithmeticException e) {
            telemetry.addData("Exception","Exceeded %d counter steps", size);
            telemetry.addData("IMU Counter","%d", IMUCounter);
        }


    }



    //----------------------------------------------------------------------------------------------
    // TEST MODE METHOD prepOpMode is used to initialize offline items - instead of initOpMode
    //----------------------------------------------------------------------------------------------
   public void prepOpMode() {

   //************* BELOW IS TEST CODE ********************************


       autoOpMode.testModeActive = true;// set for each OpMode


       autoOpMode.stoneSelect = 2;
       autoOpMode.fc = new FieldConfiguration(autoOpMode.stoneSelect);//KS added 12/20 to set stone position

       autoOpMode.haveBlueFoundation = false;
       autoOpMode.haveRedFoundation= false;
       autoOpMode.haveBlueSkyStone1= false;
       autoOpMode.haveBlueSkyStone2= false;
       autoOpMode.haveRedSkyStone1= false;
       autoOpMode.haveRedSkyStone2= false;

       writeBF = false;
       writeRF = false;
       writeBS1 = false;
       writeBS2 = false;
       writeRS1 = false;
       writeRS2 = false;

       switch(location) {
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

       autoOpMode.initialize();
       //***********************************************************
       //Code that needs to be Kept in prepOpMode to initialize functions
       //***********************************************************

       autoOpMode.Billy.imu.timeStep = timeStep;
       autoOpMode.Billy.frontLeft.timeStep = timeStep;
       autoOpMode.Billy.frontRight.timeStep = timeStep;
       autoOpMode.Billy.backRight.timeStep = timeStep;
       autoOpMode.Billy.backLeft.timeStep = timeStep;

       autoOpMode.Billy.jack.timeStep = timeStep;
//       Billy.gripper.timeStep = timeStep;

       autoOpMode.fc.RedFoundationPoints.clear();
       autoOpMode.fc.BlueFoundationPoints.clear();
       autoOpMode.fc.BlueSkyStone1Points.clear();
       autoOpMode.fc.RedSkyStone1Points.clear();
       autoOpMode.fc.BlueSkyStone2Points.clear();
       autoOpMode.fc.RedSkyStone2Points.clear();
       autoOpMode.fc.updateField(autoOpMode);

       autoOpMode.Billy.imu.GripperPoints.clear();
//       Billy.imu.GripperPoints.add(new FieldLocation(Billy.imu.gripperX, Billy.imu.gripperY, Billy.imu.gripperTheta));

       autoOpMode.Billy.imu.RobotPoints.clear();
//       Billy.imu.RobotPoints.add(new FieldLocation(Billy.imu.robotOnField.x, Billy.imu.robotOnField.y, Billy.imu.robotOnField.theta));

       //Setting counter to capture array data is unique to offline running of code
       counter = 1;
       autoOpMode.Billy.imu.counter = counter;
//
//       robotNumber = 1;//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//       foundationPosChange = 0;// 26 for unmoved FoundationOpMode, 0 for moved FoundationOpMode
//       insideOutside = 0;// 0 for Inside, 24 for Outside
//       foundationInOut = 26;// 0 for Inside, 26 for Outside
//       sideColor = 1;// + for Blue, - for Red

       if(robotNumber == 1) {
           autoOpMode.Billy.frontLeft.motorTol=1.0;
           autoOpMode.Billy.frontRight.motorTol=1.0;
           autoOpMode. Billy.backRight.motorTol=1.0;
           autoOpMode.Billy.backLeft.motorTol=1.0;
           //field angle orientation is + = CCW , while robot frame is + = CW
           autoOpMode.Billy.imu.robotOnField.x = -65;//initial x position on field in inches (Added 2 inches for robot 7" to wheel center vs. 9")
           autoOpMode.Billy.imu.robotOnField.y = -36;//initial y position on field in inches
           autoOpMode.Billy.imu.robotOnField.theta = 0;//initial robot angle orientation on field in degrees from EAST
           autoOpMode.Billy.imu.priorAngle = 0;//initial robot angle orientation on field in degrees from EAST
           autoOpMode.Billy.imu.fakeAngle = 0;//initial robot angle orientation on field in degrees from EAST
           autoOpMode.Billy.robotHeading = -autoOpMode.Billy.imu.fakeAngle;//initial robot angle orientation on field in degrees from EAST
           autoOpMode.telemetry.addData("Robot Number ", "%d",robotNumber);
           autoOpMode.telemetry.update();
       }

       if(robotNumber == 2) {
           autoOpMode.Billy.frontLeft.motorTol=1.0;
           autoOpMode.Billy.frontRight.motorTol=1.0;
           autoOpMode.Billy.backRight.motorTol=1.0;
           autoOpMode.Billy.backLeft.motorTol=1.0;
           //field angle orientation is + = CCW , while robot frame is + = CW
           autoOpMode.Billy.imu.robotOnField.x = -65;//initial x position on field in inches (Added 2 inches for robot 7" to wheel center vs. 9")
           autoOpMode.Billy.imu.robotOnField.y = 58;//initial y position on field in inches (WAS 48)
           autoOpMode.Billy.imu.robotOnField.theta = 0;//initial robot angle orientation on field in degrees from EAST (WAS 180 for backing to foundation)
           autoOpMode.Billy.imu.priorAngle = 0;//initial robot angle orientation on field in degrees from EAST (WAS 180 for backing to foundation)
           autoOpMode.Billy.imu.fakeAngle = 0;//initial robot angle orientation on field in degrees from EAST (WAS 180 for backing to foundation)
           autoOpMode.Billy.robotHeading = -autoOpMode.Billy.imu.fakeAngle;//initial robot angle orientation on field in degrees from EAST
           autoOpMode.telemetry.addData("Robot Number ", "%d",robotNumber);
           autoOpMode.telemetry.update();
       }

       if(robotNumber == 3) {
           autoOpMode.Billy.frontLeft.motorTol=1.0;
           autoOpMode.Billy.frontRight.motorTol=1.0;
           autoOpMode.Billy.backRight.motorTol=1.0;
           autoOpMode.Billy.backLeft.motorTol=1.0;
           //field angle orientation is + = CCW , while robot frame is + = CW
           autoOpMode.Billy.imu.robotOnField.x = 65;//initial x position on field in inches (Added 2 inches for robot 7" to wheel center vs. 9")
           autoOpMode.Billy.imu.robotOnField.y = -36;//initial y position on field in inches
           autoOpMode.Billy.imu.robotOnField.theta = 180;//initial robot angle orientation on field in degrees from EAST
           autoOpMode.Billy.imu.priorAngle = 180;//initial robot angle orientation on field in degrees from EAST
           autoOpMode.Billy.imu.fakeAngle = 180;//initial robot angle orientation on field in degrees from EAST
           autoOpMode.Billy.robotHeading = -autoOpMode.Billy.imu.fakeAngle;//initial robot angle orientation on field in degrees from EAST
           autoOpMode.telemetry.addData("Robot Number ", "%d",robotNumber);
           autoOpMode.telemetry.update();
       }

       if(robotNumber == 4) {
           autoOpMode.Billy.frontLeft.motorTol=1.0;
           autoOpMode.Billy.frontRight.motorTol=1.0;
           autoOpMode.Billy.backRight.motorTol=1.0;
           autoOpMode.Billy.backLeft.motorTol=1.0;
           autoOpMode.Billy.imu.robotOnField.x = 65;//initial x position on field in inches (Added 2 inches for robot 7" to wheel center vs. 9")
           autoOpMode.Billy.imu.robotOnField.y = 58;//initial y position on field in inches (WAS 48)
           autoOpMode.Billy.imu.robotOnField.theta = 180;//initial robot angle orientation on field in degrees from EAST (WAS 0 for backing to foundation)
           autoOpMode.Billy.imu.priorAngle = 180;//initial robot angle orientation on field in degrees from EAST (WAS 0 for backing to foundation)
           autoOpMode.Billy.imu.fakeAngle = 180;//initial robot angle orientation on field in degrees from EAST (WAS 0 for backing to foundation)
           autoOpMode.Billy.robotHeading = -autoOpMode.Billy.imu.fakeAngle;//initial robot angle orientation on field in degrees from EAST
           autoOpMode.telemetry.addData("Robot Number ", "%d",robotNumber);
           autoOpMode.telemetry.update();
       }

       autoOpMode.Billy.angleUnWrap();
       autoOpMode.Billy.offset = autoOpMode.Billy.robotHeading;
       autoOpMode.Billy.robotHeading-=autoOpMode.Billy.offset;//set robotHeading = 0 for all opModes regardless of position, but track actual angle in IMU
       autoOpMode.fc.updateField(autoOpMode);
       //Initialize starting position on field, field center is assumed (0,0), 0 field angle is pointing EAST
//       Billy.imu.fieldXArray[0] = Billy.imu.fieldX; //initial x position on field in inches
//       Billy.imu.fieldYArray[0] = Billy.imu.fieldY; //initial y position on field in inches
//       Billy.imu.robotAngleArray[0] = Billy.imu.priorAngle; //initial robot angle orientation on field in degrees from EAST
//       Billy.imu.RobotPoints.add(new FieldLocation(Billy.imu.robotOnField.x, Billy.imu.robotOnField.y, Billy.imu.robotOnField.theta));

   }


   public enum computer{PC,MAC,WILL};

    //Run Calculations like Autonomous OpMode
    public static void main(String []args)throws IOException {

        //Instantiate a static class to run the code with
        OfflineOpModeRunFile OffRunFiles = new OfflineOpModeRunFile();

        OffRunFiles.location = computer.PC;//For Karl on HP
//        OffRunFiles.location = computer.MAC;//For Caleb
//        OffRunFiles.location = computer.WILL;//For William

        //Load the opMode HashMap from the CSV file
        OffRunFiles.ompf.checkoutFiles("OpModeCSV.csv",OffRunFiles);
        //Create a loop to call all the opModes based on the names in the CSV file
        boolean writeData = false;
        String opModeName = "unknown";
        OpModeParam currentOpParam;
        for(String s : OffRunFiles.ompf.omp.keySet()) {
            opModeName = s;
            writeData = false;
            currentOpParam = OffRunFiles.ompf.omp.get(s);
            if (s.equals("BlueStoneIn")) {
                BlueStoneIn inputOpMode = new BlueStoneIn();
                inputOpMode.testModeActive = true;
                OffRunFiles = new OfflineOpModeRunFile(inputOpMode);
                OffRunFiles.robotNumber = 1;
                OffRunFiles.prepOpMode();
                OffRunFiles.autoOpMode.runCode();
                writeData = true;
                OffRunFiles.writeBS1=true;
                OffRunFiles.writeBS2=true;
            }
            if (s.equals("BlueFoundOut")) {
                BlueFoundOut inputOpMode = new BlueFoundOut();
                inputOpMode.testModeActive = true;
                OffRunFiles = new OfflineOpModeRunFile(inputOpMode);
                OffRunFiles.robotNumber = 2;
                OffRunFiles.prepOpMode();
                OffRunFiles.autoOpMode.runCode();
                writeData = true;
                OffRunFiles.writeBF=true;
            }
            if (s.equals("RedStoneIn")) {
                RedStoneIn inputOpMode = new RedStoneIn();
                inputOpMode.testModeActive = true;
                OffRunFiles = new OfflineOpModeRunFile(inputOpMode);
                OffRunFiles.robotNumber = 3;
                OffRunFiles.prepOpMode();
                OffRunFiles.autoOpMode.runCode();
                writeData = true;
                OffRunFiles.writeRS1=true;
                OffRunFiles.writeRS2=true;
            }
            if (s.equals("RedFoundOut")) {
                RedFoundOut inputOpMode = new RedFoundOut();
                inputOpMode.testModeActive = true;
                OffRunFiles = new OfflineOpModeRunFile(inputOpMode);
                OffRunFiles.robotNumber = 4;
                OffRunFiles.prepOpMode();
                OffRunFiles.autoOpMode.runCode();
                writeData = true;
                OffRunFiles.writeRF=true;
            }


            if (writeData) {
//        for(int h = 1; h<5;h++) {//removed for HashMap for loop
//            OffRunFiles.robotNumber = h;
//            if(currentOpParam.teamColor.equals("Red")){
//                if(currentOpParam.startSide.equals("Front")){OffRunFiles.robotNumber = 3;}
//                else{OffRunFiles.robotNumber = 4;}
//            }
//            else if(currentOpParam.teamColor.equals("Blue")){
//                if(currentOpParam.startSide.equals("Front")){OffRunFiles.robotNumber = 1;}
//                else{OffRunFiles.robotNumber = 2;}
//            }

                // items below are already set in the opModes
//            OffRunFiles.autoOpMode.foundationPosChange = 0;// 26 for unmoved FoundationOpMode, 0 for moved FoundationOpMode
//            OffRunFiles.autoOpMode.insideOutside = 0;// 0 for Inside, 24 for Outside
//            OffRunFiles.autoOpMode.foundationInOut = 26;// 0 for Inside, 26 for Outside
//            if(h ==1 || h==2) {
//                OffRunFiles.autoOpMode.sideColor = 1;// + for Blue for robots 1 & 2
//            }
//            else {
//                OffRunFiles.autoOpMode.sideColor = -1;// - for Red for robots 3 & 4
//
//            }


// Lines below are to capture the array data and output
                OffRunFiles.extractArrayData();
                int countVar = Math.max(size, OffRunFiles.IMUCounter);

                fos = new FileOutputStream(OffRunFiles.fileLocation + String.format("Robot%dOnField.txt", OffRunFiles.robotNumber));// Path to directory for IntelliJ code
                OffRunFiles.fc.writeFieldAsText(fos, OffRunFiles.autoOpMode.Billy.imu.RobotPoints, countVar);

                fos = new FileOutputStream(OffRunFiles.fileLocation + String.format("Robot%dAccessories.txt", OffRunFiles.robotNumber));// Path to directory for IntelliJ code
                OffRunFiles.writeExtrasToFile(fos);

                fos = new FileOutputStream(OffRunFiles.fileLocation + String.format("Robot%dGripper.txt", OffRunFiles.robotNumber));// Path to directory for IntelliJ code
                OffRunFiles.fc.writeFieldAsText(fos, OffRunFiles.autoOpMode.Billy.imu.GripperPoints, countVar);

                if (OffRunFiles.writeRF) {
                    fos = new FileOutputStream(fileLocation + "RedFoundation.txt");// Path to directory for IntelliJ code
                    OffRunFiles.fc.writeFieldAsText(fos, OffRunFiles.autoOpMode.fc.RedFoundationPoints, countVar);
                }
                if (OffRunFiles.writeBF) {
                    fos = new FileOutputStream(fileLocation + "BlueFoundation.txt");// Path to directory for IntelliJ code
                    OffRunFiles.fc.writeFieldAsText(fos, OffRunFiles.autoOpMode.fc.BlueFoundationPoints, countVar);
                }
                if (OffRunFiles.writeRS1 || OffRunFiles.writeRS2) {
                    fos = new FileOutputStream(fileLocation + "RedSkyStone1.txt");// Path to directory for IntelliJ code
                    OffRunFiles.fc.writeFieldAsText(fos, OffRunFiles.autoOpMode.fc.RedSkyStone1Points, countVar);

                    fos = new FileOutputStream(fileLocation + "RedSkyStone2.txt");// Path to directory for IntelliJ code
                    OffRunFiles.fc.writeFieldAsText(fos, OffRunFiles.autoOpMode.fc.RedSkyStone2Points, countVar);
                }
                if (OffRunFiles.writeBS1 || OffRunFiles.writeBS2) {
                    fos = new FileOutputStream(fileLocation + "BlueSkyStone1.txt");// Path to directory for IntelliJ code
                    OffRunFiles.fc.writeFieldAsText(fos, OffRunFiles.autoOpMode.fc.BlueSkyStone1Points, countVar);

                    fos = new FileOutputStream(fileLocation + "BlueSkyStone2.txt");// Path to directory for IntelliJ code
                    OffRunFiles.fc.writeFieldAsText(fos, OffRunFiles.autoOpMode.fc.BlueSkyStone2Points, countVar);
                }
                OffRunFiles.autoOpMode.telemetry.addLine("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
                OffRunFiles.autoOpMode.telemetry.addLine(" ");
                OffRunFiles.autoOpMode.telemetry.addData("Total Number of Time Steps", "%d", OffRunFiles.autoOpMode.Billy.imu.counter);
                OffRunFiles.autoOpMode.telemetry.addData("Completed", "Robot: %d", OffRunFiles.robotNumber);
                OffRunFiles.autoOpMode.telemetry.addLine(" ");
                OffRunFiles.autoOpMode.telemetry.addLine("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
                OffRunFiles.autoOpMode.telemetry.addLine(" ");
                OffRunFiles.autoOpMode.telemetry.update();

            }
        }
    }

}