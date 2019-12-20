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


import com.qualcomm.robotcore.hardware.DcMotorSimple;

import Skystone_14999.DriveMotion.DriveMethods;
import Skystone_14999.OpModes.Autonomous.BasicAuto;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.robot.Robot;

import java.io.DataOutputStream;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
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

    int counter;
    final static int size = 300;
//    int[] totalCounts = new int[size];
    int[] flCounts = new int[size];
    int[] frCounts = new int[size];
    int[] brCounts = new int[size];
    int[] blCounts = new int[size];
    int[] flIMU = new int[size];
    int[] frIMU = new int[size];
    int[] brIMU = new int[size];
    int[] blIMU = new int[size];
    int[] lsIMU = new int[size];
    double[] arrayRobotX = new double[size];
    double[] arrayRobotY= new double[size];
    double[] arrayRobotDist = new double[size];
    double[] arrayRobotAngle = new double[size];

    public double[] arrayFLBR=new double[size];
    public double[] arrayFRBL=new double[size];

    double[] arrayFieldX = new double[size];
    double[] arrayFieldY= new double[size];
    double[] arrayFieldDist = new double[size];

//    double distanceTraveledInchAlt;
//    static double[] altDistArray = new double[size];
    static double[] distanceTraveledArray= new double[size];;
    double[] timeArray= new double[size];
//    static int maxCounts = (int) Math.round(driveDistance*ROBOT_INCH_TO_MOTOR_DEG*DEGREES_TO_COUNTS);
    public double timeStep = 1000 * 30/size;//determined a fixed time step (in milliseconds) so that faster speeds will show shorter time to distance
    //timeStep was 100 in seconds to fill 30 seconds / size of array (counts to 30
    static FileReader in = null;
    static FileWriter out = null;
    static FileOutputStream fileOutStream = null;
    static DataOutputStream dataOutStream = null;
    static DataOutputStream dos = null;

    int IMUCounter =0;

    /* Constructor */
    public OfflineOpModeLibs(){

    };


    //----------------------------------------------------------------------------------------------
    // TEST MODE CODE ONLY - EXTRACTS ARRAY DATA FROM classes
    //----------------------------------------------------------------------------------------------
    public void extractArrayData(){

//        telemetry.addData("Distance", " Command(%.2f), Current(%.2f)", commandInch, distanceTraveledInch);
//        telemetry.addData("control loop:", "Current Error(%.1f), Sum Error(%.1f), Steering Power(%.1f)", error, sumError, steerInput);
//        telemetry.update();
        flIMU = Billy.imu.flArray;
        frIMU = Billy.imu.frArray;
        brIMU = Billy.imu.brArray;
        blIMU = Billy.imu.blArray;
        lsIMU = Billy.imu.lsArray;
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
        arrayFieldX = Billy.imu.fieldXArray;
        arrayFieldY= Billy.imu.fieldYArray;
        arrayFieldDist = Billy.imu.fieldDistArray;

        Arrays.fill(flIMU,IMUCounter,(size), flIMU[IMUCounter-1]);
        Arrays.fill(frIMU,IMUCounter,(size), frIMU[IMUCounter-1]);
        Arrays.fill(brIMU,IMUCounter,(size), brIMU[IMUCounter-1]);
        Arrays.fill(blIMU,IMUCounter,(size), blIMU[IMUCounter-1]);


        Arrays.fill(arrayFLBR,IMUCounter,(size),arrayFLBR[IMUCounter-1]);
        Arrays.fill(arrayFRBL,IMUCounter,(size),arrayFRBL[IMUCounter-1]);

        Arrays.fill(arrayRobotX,IMUCounter,(size),arrayRobotX[IMUCounter-1]);
        Arrays.fill(arrayRobotY,IMUCounter,(size),arrayRobotY[IMUCounter-1]);
        Arrays.fill(arrayRobotDist,IMUCounter,(size),arrayRobotDist[IMUCounter-1]);
        Arrays.fill(arrayRobotAngle,IMUCounter,(size),arrayRobotAngle[IMUCounter-1]);

//        Arrays.fill(RX,counter,(size),RX[counter-1]);
//        Arrays.fill(RY,counter,(size),RY[counter-1]);

        Arrays.fill(arrayFieldX,IMUCounter,(size),arrayFieldX[IMUCounter-1]);
        Arrays.fill(arrayFieldY,IMUCounter,(size),arrayFieldY[IMUCounter-1]);
        Arrays.fill(arrayFieldDist,IMUCounter,(size),arrayFieldDist[IMUCounter-1]);
//        Arrays.fill(FX,counter,(size),FX[counter-1]);
//        Arrays.fill(FY,counter,(size),FY[counter-1]);
//        Arrays.fill(FDist,counter,(size),FDist[counter-1]);

        Arrays.fill(distanceTraveledArray,IMUCounter, size, distanceTraveledArray[IMUCounter-1]);
//        Arrays.fill(altDistArray,counter,(size),altDistArray[counter-1]);
        double deltaTime = (timeArray[1] - timeArray[0]);
        for(int k = IMUCounter; k < size;k++){
            timeArray[k] = timeArray[k-1] + deltaTime;
            Billy.imu.RedFoundationPoints.add(Billy.imu.RedFoundationPoints.get(k-1));
            Billy.imu.BlueFoundationPoints.add(Billy.imu.BlueFoundationPoints.get(k-1));
            Billy.imu.SkyStonePoints.add(Billy.imu.SkyStonePoints.get(k-1));

        }

    }

    //----------------------------------------------------------------------------------------------
    // TEST MODE CODE ONLY - WRITES DATA TO FILE
    //----------------------------------------------------------------------------------------------
    public void writeToFile( FileWriter out, DataOutputStream fileData){
        int countVar = Math.max(size, IMUCounter);

        try {
            out.write(String.format("Calculated IMU Counts\r"));
            out.write(String.format("Time,FL_IMU,FR_IMU,BR_IMU,BL_IMU,LS_IMU,FLBR_cnt,FRBL_cnt,"+
                    "RobotY, RobotX, RobotAngle, Field_Y, Field_X\r"));

            for (int i = 0; i < countVar; i++) {
//                out.write(String.format("%d,%d,%d,%d,%d,", i,  flCounts[i], frCounts[i], brCounts[i], blCounts[i]));
                out.write(String.format("%.3f,%d,%d,%d,%d,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f",
                        timeArray[i], flIMU[i], frIMU[i], brIMU[i], blIMU[i], lsIMU[i],
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
    // TEST MODE CODE ONLY - WRITES DATA TO SCREEN
    //----------------------------------------------------------------------------------------------
    public void writeToScreen(){

        System.out.println(String.format("Calculated Counts\r"));
        System.out.println(String.format("Time,FLcalc,FRcalc,BRcalc,BLcalc,FLBR_cnt, FRBL_cnt\r"));
        int countVar = Math.max(size, IMUCounter);
        for(int i=0; i<countVar; i++) {
//            System.out.print(String.format("%d,%d,%d,%d,%d,", i, flCounts[i], frCounts[i], brCounts[i], blCounts[i]));
            System.out.print(String.format("%.3f,%d,%d,%d,%d,%.1f,%.1f",timeArray[i], flIMU[i], frIMU[i], brIMU[i], blIMU[i],
                    arrayFLBR[i],arrayFRBL[i]));
            System.out.println(String.format(" "));
        }
        System.out.println(String.format("-----------------------"));
        System.out.println("Robot Motion Final Values");
        System.out.println(String.format("%.3f Robot final rotation angle (deg.)",arrayRobotAngle[size-1]));
        System.out.println(String.format("%.3f Robot final X distance (in.)",arrayRobotX[size-1]));
        System.out.println(String.format("%.3f Robot final Y distance (in.)",arrayRobotY[size-1]));
        System.out.println(String.format("%.3f Robot final total distance (in.)",arrayRobotDist[size-1]));
        System.out.println(String.format("-----------------------"));

//        System.out.println(String.format("Final Distance: %.3f",distanceTraveledArray[distanceTraveledArray.length-1]));
//        System.out.println(String.format("Final Error: %.3f", arrayRobotDist[size-1] - distanceTraveledArray[distanceTraveledArray.length-1]));
//        System.out.println(String.format("-----------------------"));
        System.out.println(String.format(" "));

        System.out.println("Distance Traveled Array Values");
        System.out.println("Time,Distance Travel, FieldX, FieldY, RobotDist, Angle");

        for(int j =0; j<distanceTraveledArray.length; j++){
            System.out.println(String.format("%.3f,%.3f,%.3f,%.3f,%.3f,%.1f\r",timeArray[j],distanceTraveledArray[j],arrayFieldX[j],arrayFieldY[j],arrayRobotDist[j],arrayRobotAngle[j]));

        }
    }
    //----------------------------------------------------------------------------------------------
    // TEST MODE CODE ONLY - UPDATES FAKE IMU
    //----------------------------------------------------------------------------------------------
    @Override
    public void updateIMU(){
        Billy.imu.flCnt = Billy.frontLeft.getCurrentPosition();
        Billy.imu.frCnt = Billy.frontRight.getCurrentPosition();
        Billy.imu.brCnt = Billy.backRight.getCurrentPosition();
        Billy.imu.blCnt = Billy.backLeft.getCurrentPosition();

        Billy.imu.haveBlueFoundation = haveBlueFoundation;
        Billy.imu.haveRedFoundation = haveRedFoundation;
        Billy.imu.haveSkyStone = haveSkyStone;

        try {
//
                if (IMUCounter >= size) {
                    int a = 1 / 0;
                }
            } catch (ArithmeticException e) {
                System.out.println(String.format("Exceeded %d counter steps", size));
                System.out.println(String.format("IMU Counter = %d", IMUCounter));
//                distanceTraveledInch = driveDistance;
            }


    }

    //----------------------------------------------------------------------------------------------
    // TEST MODE METHOD prepOpMode is used to initialize offline items - instead of initOpMode
    //----------------------------------------------------------------------------------------------
   public void prepOpMode() {

       //This is the offline version of init that has additional items
       //Needs to be run before runOpMode or inherited init
       //Added code to try to use library version of hardware
       Billy.isTestMode = true;
       activeOpMode = true;
       // Map all of hardware to program from robot using HardwareVincent script
       //Will refer all defined configuration items - motors, servos, sensors- to this defined Billy class
//        createRobot();

       Billy.init(hardwareMap);

       //Motor configuration, recommend Not Changing - Set all motors to forward direction, positive = clockwise when viewed from shaft side
       Billy.frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
       Billy.frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
       Billy.backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
       Billy.backRight.setDirection(DcMotorSimple.Direction.FORWARD);

       Billy.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       Billy.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       Billy.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       Billy.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       //Reset all motor encoders
       Billy.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       Billy.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       Billy.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       Billy.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

       Billy.frontLeft.setTargetPosition(0);
       Billy.frontRight.setTargetPosition(0);
       Billy.backLeft.setTargetPosition(0);
       Billy.backRight.setTargetPosition(0);

       //Set all motors to position mode (assumes that all motors have encoders on them)
       Billy.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       Billy.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       Billy.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       Billy.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

       Billy.frontLeft.setPower(0);
       Billy.frontRight.setPower(0);
       Billy.backLeft.setPower(0);
       Billy.backRight.setPower(0);

//       targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");
//
//
//       VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
//       stoneTarget.setName("Stone Target");
//       VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
//       blueRearBridge.setName("Blue Rear Bridge");
//       VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
//       redRearBridge.setName("Red Rear Bridge");
//       VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
//       redFrontBridge.setName("Red Front Bridge");
//       VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
//       blueFrontBridge.setName("Blue Front Bridge");
//       VuforiaTrackable red1 = targetsSkyStone.get(5);
//       red1.setName("Red Perimeter 1");
//       VuforiaTrackable red2 = targetsSkyStone.get(6);
//       red2.setName("Red Perimeter 2");
//       VuforiaTrackable front1 = targetsSkyStone.get(7);
//       front1.setName("Front Perimeter 1");
//       VuforiaTrackable front2 = targetsSkyStone.get(8);
//       front2.setName("Front Perimeter 2");
//       VuforiaTrackable blue1 = targetsSkyStone.get(9);
//       blue1.setName("Blue Perimeter 1");
//       VuforiaTrackable blue2 = targetsSkyStone.get(10);
//       blue2.setName("Blue Perimeter 2");
//       VuforiaTrackable rear1 = targetsSkyStone.get(11);
//       rear1.setName("Rear Perimeter 1");
//       VuforiaTrackable rear2 = targetsSkyStone.get(12);
//       rear2.setName("Rear Perimeter 2");
//
//       allTrackables.addAll(targetsSkyStone);

       readOrWriteHashMapOffline();

       //Indicate initialization complete and provide telemetry
       telemetry.addData("Status: ", "Initialized");
       telemetry.addData("Drive Motors", "FL (%.2f), FR (%.2f), BL (%.2f), BR (%.2f)", Billy.frontLeft.getPower(), Billy.frontRight.getPower(), Billy.backLeft.getPower(), Billy.backRight.getPower());
       telemetry.addData("Target Positions", "Forward (%d), Right (%d), Rotate (%d)", forwardPosition, rightPosition, clockwisePosition);
       telemetry.update();//Update telemetry to update display

       //***********************************************************
       //Code that needs to be Kept in init to initialize functions
       //***********************************************************

       Billy.imu.timeStep = timeStep;
       Billy.frontLeft.timeStep = timeStep;
       Billy.frontRight.timeStep = timeStep;
       Billy.backRight.timeStep = timeStep;
       Billy.backLeft.timeStep = timeStep;

       Billy.imu.RedFoundationPoints.add(new FieldLocation(Billy.imu.redFoundX,Billy.imu.redFoundY,Billy.imu.redFoundTheta));
       Billy.imu.BlueFoundationPoints.add(new FieldLocation(Billy.imu.blueFoundX,Billy.imu.blueFoundY,Billy.imu.blueFoundTheta));
       Billy.imu.SkyStonePoints.add(new FieldLocation(Billy.imu.stoneX,Billy.imu.stoneY,Billy.imu.stoneTheta));

       //Setting counter to capture array data is unique to offline running of code
       counter = 1;
       Billy.imu.counter = counter;
       Billy.robotNumber = 1;//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
       foundationPosChange = 26;// 26 for unmoved Foundation, 0 for moved Foundation
       insideOutside = 0;// 0 for Inside, 24 for Outside
       foundationInOut = 26;// 0 for Inside, 26 for Outside
       sideColor = 1;// + for Blue, - for Red

       if(Billy.robotNumber == 1) {
           cons.pHM.get("drivePowerLimit").setParameter(0.75);
//           Billy.frontLeft.motorTol=1.2;
//           Billy.backLeft.motorTol=1.2;
           //field angle orientation is + = CCW , while robot frame is + = CW
           Billy.imu.fieldX = -63;//initial x position on field in inches
           Billy.imu.fieldY = -28;//initial y position on field in inches
           Billy.imu.priorAngle = 0;//initial robot angle orientation on field in degrees from EAST
           Billy.imu.fakeAngle = 0;//initial robot angle orientation on field in degrees from EAST
       }

       if(Billy.robotNumber == 2) {

           cons.pHM.get("drivePowerLimit").setParameter(0.75);
//           Billy.frontLeft.motorTol=1.2;
//           Billy.backLeft.motorTol=1.2;
//
           //field angle orientation is + = CCW , while robot frame is + = CW
           Billy.imu.fieldX = -63;//initial x position on field in inches
           Billy.imu.fieldY = 48;//initial y position on field in inches
           Billy.imu.priorAngle = 180;//initial robot angle orientation on field in degrees from EAST
           Billy.imu.fakeAngle = 180;//initial robot angle orientation on field in degrees from EAST
           telemetry.addData("Robot Number ", "%d",Billy.robotNumber);
           telemetry.addData("drivePowerLimit ", "%.2f",cons.pHM.get("drivePowerLimit").value);

           telemetry.update();
       }

       //Initialize starting position on field, field cent eris assumed (0,0), 0 field angle is pointing EAST
       Billy.imu.fieldXArray[0] = Billy.imu.fieldX; //initial x position on field in inches
       Billy.imu.fieldYArray[0] = Billy.imu.fieldY; //initial y position on field in inches
       Billy.imu.robotAngleArray[0] = Billy.imu.priorAngle; //initial robot angle orientation on field in degrees from EAST

   }

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

        initialize();

        runtime.reset();

        if(Billy.robotNumber ==1) {

            fwdToStone();

            findSkyStone();

            crossDropStoneFor2();

            getSecondStone();

        }
        if(Billy.robotNumber ==2) {
            if(foundationPosChange == 26){

                drv.driveGeneral(DriveMethods.moveDirection.RightLeft,-50, cons.pHM.get("drivePowerLimit").value, "Left 30 inches",this);
            }

            if(foundationPosChange != 26) {
            grabFoundation();

            pullFoundation();

            aroundFoundation();

            pushFoundation();

            awayFromFoundation();

            }
        }

    } //MAIN OpMode PROGRAM END


    //##############################################################################################
    // END ACTUAL CODE TO BE TESTED AS METHODS
    //##############################################################################################

    //Run Calculations like Autonomous OpMode
    public static void main(String []args)throws IOException {

// Code to setup the main program that runs offline, none of this is robot code
        OfflineOpModeLibs OffLibs = new OfflineOpModeLibs();
        OffLibs.testMode = true;//Declare Test Mode
        OffLibs.activeOpMode = true;
        // Prepare robot class for offline operation, must be run prior to copied runOpMode or init
        // Sets initial position and counters and initial array variables
        OffLibs.prepOpMode();

        StringBuilder fileName = new StringBuilder();
        fileName.append("RMO_");
        fileName.append(String.format("%s.csv","RuckusAutonomous01"));
        out = new FileWriter(fileName.toString());
        //============= SET ROBOT 1 or 2 to WRITE DATA ==============
//        fileOutStream = new FileOutputStream("C:/Users/Spiessbach/Documents/GitHub/OfflineViz/RobotOnField.dat");// Path to directory for IntelliJ code
//        fileOutStream = new FileOutputStream("C:/Users/Spiessbach/Documents/GitHub/OfflineCode/Robot1OnField.dat");// Path to directory for IntelliJ code
        fileOutStream = new FileOutputStream("/Users/caleb/Documents/FTC/IntelliJ/RobotVisualization/"+String.format("Robot%dOnField.dat",OffLibs.Billy.robotNumber));// Path to directory for IntelliJ code

        dataOutStream = new DataOutputStream(fileOutStream);
//calls code input by programmer into runAutonomous method that comes from main runOpMode
        OffLibs.runOpMode();
// Lines below are to capture the array data and output
        OffLibs.extractArrayData();
        OffLibs.writeToFile(out, dataOutStream);
//        OffLibs.writeToScreen();
        fileOutStream.close();

        dos = new DataOutputStream(new FileOutputStream("/Users/caleb/Documents/FTC/IntelliJ/RobotVisualization/SkyStone.dat"));// Path to directory for IntelliJ code
        OffLibs.writeListToFile(dos, OffLibs.Billy.imu.SkyStonePoints);

        dos = new DataOutputStream(new FileOutputStream("/Users/caleb/Documents/FTC/IntelliJ/RobotVisualization/RedFoundation.dat"));// Path to directory for IntelliJ code
        OffLibs.writeListToFile(dos, OffLibs.Billy.imu.RedFoundationPoints);

        dos = new DataOutputStream(new FileOutputStream("/Users/caleb/Documents/FTC/IntelliJ/RobotVisualization/BlueFoundation.dat"));// Path to directory for IntelliJ code
        OffLibs.writeListToFile(dos, OffLibs.Billy.imu.BlueFoundationPoints);

        OffLibs.telemetry.addData("Total Number of Time Steps", "%d",OffLibs.IMUCounter);
        OffLibs.telemetry.update();
    }

}