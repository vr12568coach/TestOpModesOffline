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


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import Skystone_14999.HarwareConfig.HardwareBilly;
import Skystone_14999.OpModes.Autonomous.BasicAuto;
import Skystone_14999.OpModes.Autonomous.DoubleSkyStoneDP_InB;
//import com.qualcomm.robotcore.hardware.HardwareMap;

import java.io.DataOutputStream;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.ArrayList;
import java.util.Arrays;


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
    public double timeStep = 150;//determined a fixed time step (in milliseconds) so that faster speeds will show shorter time to distance
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


        flIMU = Billy.imu.flArray;
        frIMU = Billy.imu.frArray;
        brIMU = Billy.imu.brArray;
        blIMU = Billy.imu.blArray;
        jackIMU = Billy.imu.jackDirection;
        gripIMU  = Billy.imu.gripperWidth;
        blueStoneServoIMU  = Billy.imu.blueServoArray;
        redStoneServoIMU  = Billy.imu.redServoArray;


        timeArray = Billy.imu.timeArray;

        arrayRobotX = Billy.imu.robotXArray;
        arrayRobotY = Billy.imu.robotYArray;
        arrayRobotDist = Billy.imu.robotDistArray;
        arrayRobotAngle = Billy.imu.robotAngleArray;
        IMUCounter = Billy.imu.counter;

        arrayFLBR = Billy.imu.FLBRArray;
        arrayFRBL = Billy.imu.FRBLArray;

//        robotX = Billy.imu.robotX;
//        robotY = Billy.imu.robotY;
//        robotDist = Billy.imu.robotDist;
//        robotAngle = (double) Billy.imu.fakeAngle;
//        arrayFieldX = Billy.imu.fieldXArray;
//        arrayFieldY= Billy.imu.fieldYArray;
//        arrayFieldDist = Billy.imu.fieldDistArray;
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
            Billy.imu.RobotPoints.add(Billy.imu.RobotPoints.get(k-1));
            Billy.imu.GripperPoints.add(Billy.imu.GripperPoints.get(k-1));


            fc.BlueFoundationPoints.add(fc.BlueFoundationPoints.get(k-1));
            fc.RedFoundationPoints.add(fc.RedFoundationPoints.get(k-1));

            fc.RedSkyStone1Points.add(fc.RedSkyStone1Points.get(k-1));
            fc.BlueSkyStone1Points.add(fc.BlueSkyStone1Points.get(k-1));

            fc.BlueSkyStone2Points.add(fc.BlueSkyStone2Points.get(k-1));
            fc.RedSkyStone2Points.add(fc.RedSkyStone2Points.get(k-1));
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
        Billy.imu.flCnt = Billy.frontLeft.getCurrentPosition();
        Billy.imu.frCnt = Billy.frontRight.getCurrentPosition();
        Billy.imu.brCnt = Billy.backRight.getCurrentPosition();
        Billy.imu.blCnt = Billy.backLeft.getCurrentPosition();

        Billy.imu.blueStoneServoPos = Billy.stoneServoArm.getPosition();
//        Billy.imu.redStoneServoPos = Billy.servoRedStoneGrab.getPosition();

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

//    @Override
//    public void opModeIsActive(){//Need a way to easily have OpMode is Active
//        //change methods to "condition" && (om.opModeIsActive || om.testModeActive)
//        //set testModeActive = false in BasicAuto and only override in the Test Offline code
//
//    }

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

       initializeMiniBot();
       //***********************************************************
       //Code that needs to be Kept in init to initialize functions
       //***********************************************************

       Billy.imu.timeStep = timeStep;
       Billy.frontLeft.timeStep = timeStep;
       Billy.frontRight.timeStep = timeStep;
       Billy.backRight.timeStep = timeStep;
       Billy.backLeft.timeStep = timeStep;

       Billy.jack.timeStep = timeStep;
//       Billy.gripper.timeStep = timeStep;

       fc.RedFoundationPoints.clear();
       fc.BlueFoundationPoints.clear();
       fc.BlueSkyStone1Points.clear();
       fc.RedSkyStone1Points.clear();
       fc.BlueSkyStone2Points.clear();
       fc.RedSkyStone2Points.clear();
       fc.updateField(this);

       Billy.imu.GripperPoints.clear();
//       Billy.imu.GripperPoints.add(new FieldLocation(Billy.imu.gripperX, Billy.imu.gripperY, Billy.imu.gripperTheta));

       Billy.imu.RobotPoints.clear();
//       Billy.imu.RobotPoints.add(new FieldLocation(Billy.imu.robotOnField.x, Billy.imu.robotOnField.y, Billy.imu.robotOnField.theta));

       //Setting counter to capture array data is unique to offline running of code
       counter = 1;
       Billy.imu.counter = counter;
//
//       robotNumber = 1;//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//       foundationPosChange = 0;// 26 for unmoved FoundationOpMode, 0 for moved FoundationOpMode
//       insideOutside = 0;// 0 for Inside, 24 for Outside
//       foundationInOut = 26;// 0 for Inside, 26 for Outside
//       sideColor = 1;// + for Blue, - for Red

       if(robotNumber == 1) {
//           cons.pHM.get("drivePowerLimit").setParameter(1.0);
           Billy.frontLeft.motorTol=1.0;
           Billy.frontRight.motorTol=1.0;
           Billy.backRight.motorTol=1.0;
           Billy.backLeft.motorTol=1.0;
           //field angle orientation is + = CCW , while robot frame is + = CW
           Billy.imu.robotOnField.x = -63;//initial x position on field in inches
           Billy.imu.robotOnField.y = -36;//initial y position on field in inches
           Billy.imu.robotOnField.theta = 0;//initial robot angle orientation on field in degrees from EAST
           Billy.imu.priorAngle = 0;//initial robot angle orientation on field in degrees from EAST
           Billy.imu.fakeAngle = 0;//initial robot angle orientation on field in degrees from EAST
           Billy.robotHeading = -Billy.imu.fakeAngle;//initial robot angle orientation on field in degrees from EAST

       }

       if(robotNumber == 2) {

//           cons.pHM.get("drivePowerLimit").setParameter(0.75);
           Billy.frontLeft.motorTol=1.0;
           Billy.frontRight.motorTol=1.0;
           Billy.backRight.motorTol=1.0;
           Billy.backLeft.motorTol=1.0;
           //field angle orientation is + = CCW , while robot frame is + = CW
           Billy.imu.robotOnField.x = -63;//initial x position on field in inches
           Billy.imu.robotOnField.y = 48;//initial y position on field in inches (WAS 48)
           Billy.imu.robotOnField.theta = 0;//initial robot angle orientation on field in degrees from EAST (WAS 180 for backing to foundation)
           Billy.imu.priorAngle = 0;//initial robot angle orientation on field in degrees from EAST (WAS 180 for backing to foundation)
           Billy.imu.fakeAngle = 0;//initial robot angle orientation on field in degrees from EAST (WAS 180 for backing to foundation)
           Billy.robotHeading = -Billy.imu.fakeAngle;//initial robot angle orientation on field in degrees from EAST
           telemetry.addData("Robot Number ", "%d",robotNumber);
           telemetry.addData("drivePowerLimit ", "%.2f",cons.pHM.get("drivePowerLimit").value);

           telemetry.update();
       }

       if(robotNumber == 3) {
//           cons.pHM.get("drivePowerLimit").setParameter(0.75);
           Billy.frontLeft.motorTol=1.0;
           Billy.frontRight.motorTol=1.0;
           Billy.backRight.motorTol=1.0;
           Billy.backLeft.motorTol=1.0;
           //field angle orientation is + = CCW , while robot frame is + = CW
           Billy.imu.robotOnField.x = 63;//initial x position on field in inches
           Billy.imu.robotOnField.y = -36;//initial y position on field in inches
           Billy.imu.robotOnField.theta = 180;//initial robot angle orientation on field in degrees from EAST
           Billy.imu.priorAngle = 180;//initial robot angle orientation on field in degrees from EAST
           Billy.imu.fakeAngle = 180;//initial robot angle orientation on field in degrees from EAST
           Billy.robotHeading = -Billy.imu.fakeAngle;//initial robot angle orientation on field in degrees from EAST
       }

       if(robotNumber == 4) {
//           cons.pHM.get("drivePowerLimit").setParameter(0.75);
           Billy.frontLeft.motorTol=1.0;
           Billy.frontRight.motorTol=1.0;
           Billy.backRight.motorTol=1.0;
           Billy.backLeft.motorTol=1.0;
           Billy.imu.robotOnField.x = 63;//initial x position on field in inches
           Billy.imu.robotOnField.y = 48;//initial y position on field in inches (WAS 48)
           Billy.imu.robotOnField.theta = 180;//initial robot angle orientation on field in degrees from EAST (WAS 0 for backing to foundation)
           Billy.imu.priorAngle = 180;//initial robot angle orientation on field in degrees from EAST (WAS 0 for backing to foundation)
           Billy.imu.fakeAngle = 180;//initial robot angle orientation on field in degrees from EAST (WAS 0 for backing to foundation)
           Billy.robotHeading = -Billy.imu.fakeAngle;//initial robot angle orientation on field in degrees from EAST
           telemetry.addData("Robot Number ", "%d",robotNumber);
           telemetry.addData("drivePowerLimit ", "%.2f",cons.pHM.get("drivePowerLimit").value);

           telemetry.update();
       }

       Billy.angleUnWrap();
       Billy.offset = Billy.robotHeading;
       Billy.robotHeading-=Billy.offset;//set robotHeading = 0 for all opModes regardless of position, but track actual angle in IMU
       fc.updateField(this);
       //Initialize starting position on field, field center is assumed (0,0), 0 field angle is pointing EAST
//       Billy.imu.fieldXArray[0] = Billy.imu.fieldX; //initial x position on field in inches
//       Billy.imu.fieldYArray[0] = Billy.imu.fieldY; //initial y position on field in inches
//       Billy.imu.robotAngleArray[0] = Billy.imu.priorAngle; //initial robot angle orientation on field in degrees from EAST
//       Billy.imu.RobotPoints.add(new FieldLocation(Billy.imu.robotOnField.x, Billy.imu.robotOnField.y, Billy.imu.robotOnField.theta));

   }


   public enum computer{PC,MAC,WILL};
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    // INSERT ACTUAL CODE TO BE TESTED IN METHODS
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //----------------------------------------------------------------------------------------------
    // TEST MODE METHOD runAutonomous is populated with actual program code in runOpMode
    //----------------------------------------------------------------------------------------------

    @Override
    public void initialize() {


        //************** BELOW IS initializeMiniBot() FROM BasicAuto ************************
//        readOrWriteHashMap();
//
//        Billy.initMiniBot(hardwareMap, cons);
//
//        //Motor configuration, recommend Not Changing - Set all motors to forward direction, positive = clockwise when viewed from shaft side
//        Billy.frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
//        Billy.frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
//        Billy.backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
//        Billy.backRight.setDirection(DcMotorSimple.Direction.FORWARD);
//
//        Billy.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        Billy.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        Billy.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        Billy.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        //Reset all motor encoders
//        Billy.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        Billy.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        Billy.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        Billy.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        Billy.frontLeft.setTargetPosition(0);
//        Billy.frontRight.setTargetPosition(0);
//        Billy.backLeft.setTargetPosition(0);
//        Billy.backRight.setTargetPosition(0);
//
//        //Set all motors to position mode (assumes that all motors have encoders on them)
//        Billy.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        Billy.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        Billy.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        Billy.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        Billy.frontLeft.setPower(0);
//        Billy.frontRight.setPower(0);
//        Billy.backLeft.setPower(0);
//        Billy.backRight.setPower(0);
//
//        Billy.stoneServoArm.setPosition(0);
//
//        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
//        stoneTarget.setName("Stone Target");
//        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
//        blueRearBridge.setName("Blue Rear Bridge");
//        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
//        redRearBridge.setName("Red Rear Bridge");
//        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
//        redFrontBridge.setName("Red Front Bridge");
//        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
//        blueFrontBridge.setName("Blue Front Bridge");
//        VuforiaTrackable red1 = targetsSkyStone.get(5);
//        red1.setName("Red Perimeter 1");
//        VuforiaTrackable red2 = targetsSkyStone.get(6);
//        red2.setName("Red Perimeter 2");
//        VuforiaTrackable front1 = targetsSkyStone.get(7);
//        front1.setName("Front Perimeter 1");
//        VuforiaTrackable front2 = targetsSkyStone.get(8);
//        front2.setName("Front Perimeter 2");
//        VuforiaTrackable blue1 = targetsSkyStone.get(9);
//        blue1.setName("Blue Perimeter 1");
//        VuforiaTrackable blue2 = targetsSkyStone.get(10);
//        blue2.setName("Blue Perimeter 2");
//        VuforiaTrackable rear1 = targetsSkyStone.get(11);
//        rear1.setName("Rear Perimeter 1");
//        VuforiaTrackable rear2 = targetsSkyStone.get(12);
//        rear2.setName("Rear Perimeter 2");
//
//        allTrackables.addAll(targetsSkyStone);
//
//        //Indicate initialization complete and provide telemetry
//        telemetry.addData("Status: ", "Initialized");
//        telemetry.addData("Drive Motors", "FL (%.2f), FR (%.2f), BL (%.2f), BR (%.2f)", Billy.frontLeft.getPower(), Billy.frontRight.getPower(), Billy.backLeft.getPower(), Billy.backRight.getPower());
//        telemetry.addData("Target Positions", "Forward (%d), Right (%d), Rotate (%d)", forwardPosition, rightPosition, clockwisePosition);
//        telemetry.addData(">", "Press Play to start");
//        telemetry.update();//Update telemetry to update display

        //************** ABOVE IS initializeMiniBot() FROM BasicAuto ************************

    }
    @Override
    public void runOpMode(){

//        initialize();

        runtime.reset();

        if(robotNumber ==1 || robotNumber ==3){
            runtime.reset();

            Billy.initIMU(this);

            fwdToTwoStone();

            vuforiaStoneLocateOffline(stoneSelect); //REPLACES THE ACTUAL VUFORIA CODE

            goToStone();

            takeStone1();

            getStone2();

            takeStone2();

            twoStonePark();

            telemetry.addLine("OpMode Complete");
            telemetry.update();
            if(robotNumber ==1) {
                writeBS1 = true;
            }
            if(robotNumber ==3) {
                writeRS1 = true;
//                // Added if statements to write foundation files to clear old data
            }
        }

        if(robotNumber ==2){
            if(foundationPosChange == 26) {
               Billy.IMUDriveFwdRight(HardwareBilly.moveDirection.RightLeft,50*sideColor, 0, "RIGHT/LEFT 50 inches",this);

            }
            if(foundationPosChange != 26) {
                runtime.reset();

                Billy.initIMU(this);

                grabFoundation();

                pullFoundation();

                awayFromFoundation();

                telemetry.addLine("OpMode Complete");
                telemetry.update();

            }
            writeBF = true;
        }
        if(robotNumber ==4 ) {
            if(foundationPosChange == 26){
                Billy.IMUDriveFwdRight(HardwareBilly.moveDirection.RightLeft,50*sideColor, 0, "RIGHT/LEFT 50 inches",this);
            }

            if(foundationPosChange != 26) {
                runtime.reset();

                Billy.initIMU(this);

                grabFoundation();

                pullFoundation();

                awayFromFoundation();

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

        OffLibs.location = computer.PC;//For Karl on HP
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
            OffLibs.fc.writeFieldAsText(fos, OffLibs.Billy.imu.RobotPoints, countVar);

            fos = new FileOutputStream(OffLibs.fileLocation + String.format("Robot%dAccessories.txt", OffLibs.robotNumber));// Path to directory for IntelliJ code
            OffLibs.writeExtrasToFile(fos);

            fos = new FileOutputStream(OffLibs.fileLocation + String.format("Robot%dGripper.txt", OffLibs.robotNumber));// Path to directory for IntelliJ code
            OffLibs.fc.writeFieldAsText(fos, OffLibs.Billy.imu.GripperPoints, countVar);


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