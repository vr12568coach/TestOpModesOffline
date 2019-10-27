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
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import CoachCode.CoachOpMode.CoachBasicAuto;
import CoachCode.CoachOpMode.CoachBasicOpMode;


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

public class OfflineOpModeLibs extends CoachBasicAuto{

//    public CoachDriveFunction CoachDrive = new CoachDriveFunction();

//    public Telemetry telemetry = new Telemetry();
//    public FakeTelemetry telemetry = new FakeTelemetry();


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
    int[] jackIMU = new int[size];
    double[] gripIMU = new double[size];
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

    int IMUCounter =0;

    public boolean opModeIsRunning = true;

    static DataOutputStream dos = null;

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
        flIMU = RobotVincent.imu.flArray;
        frIMU = RobotVincent.imu.frArray;
        brIMU = RobotVincent.imu.brArray;
        blIMU = RobotVincent.imu.blArray;
        jackIMU = RobotVincent.imu.jackDirection;
        gripIMU = RobotVincent.imu.gripperWidth;
        timeArray = RobotVincent.imu.timeArray;
        arrayRobotX = RobotVincent.imu.robotXArray;
        arrayRobotY = RobotVincent.imu.robotYArray;
        arrayRobotDist = RobotVincent.imu.robotDistArray;
        arrayRobotAngle = RobotVincent.imu.robotAngleArray;
        IMUCounter = RobotVincent.imu.counter;

        arrayFLBR = RobotVincent.imu.FLBRArray;
        arrayFRBL = RobotVincent.imu.FRBLArray;

        arrayFieldX = RobotVincent.imu.fieldXArray;
        arrayFieldY= RobotVincent.imu.fieldYArray;
        arrayFieldDist = RobotVincent.imu.fieldDistArray;

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

        Arrays.fill(arrayFieldX,IMUCounter,(size),arrayFieldX[IMUCounter-1]);
        Arrays.fill(arrayFieldY,IMUCounter,(size),arrayFieldY[IMUCounter-1]);
        Arrays.fill(arrayFieldDist,IMUCounter,(size),arrayFieldDist[IMUCounter-1]);


        Arrays.fill(distanceTraveledArray,IMUCounter, size, distanceTraveledArray[IMUCounter-1]);
//        Arrays.fill(altDistArray,counter,(size),altDistArray[counter-1]);
        double deltaTime = (timeArray[1] - timeArray[0]);
        for(int k = IMUCounter; k < size;k++){
            timeArray[k] = timeArray[k-1] + deltaTime;
            RobotVincent.imu.RedFoundationPoints.add(RobotVincent.imu.RedFoundationPoints.get(k-1));
            RobotVincent.imu.BlueFoundationPoints.add(RobotVincent.imu.BlueFoundationPoints.get(k-1));
            //%%%%%%%%%%%%%%%%%% UPDATED BELOW 10/27/19 FOR 4 ROBOTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            RobotVincent.imu.BlueSkyStonePoints.add(RobotVincent.imu.BlueSkyStonePoints.get(k-1));
            RobotVincent.imu.RedSkyStonePoints.add(RobotVincent.imu.RedSkyStonePoints.get(k-1));
            RobotVincent.imu.GripperPoints.add(RobotVincent.imu.GripperPoints.get(k-1));

            //%%%%%%%%%%%%%%%%%% UPDATED ABOVE 10/27/19 FOR 4 ROBOTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


        }

    }

    //----------------------------------------------------------------------------------------------
    // TEST MODE CODE ONLY - WRITES DATA TO FILE
    //----------------------------------------------------------------------------------------------
    public void writeToFile( FileWriter out, DataOutputStream fileData){
        int countVar = Math.max(size, IMUCounter);

        try {
            out.write(String.format("Calculated IMU Counts\r"));
            out.write(String.format("Time,FL_IMU,FR_IMU,BR_IMU,BL_IMU,LS_IMU,FLBR_cnt,FRBL_cnt, RobotY, RobotX, RobotAngle, Field_Y, Field_X\r"));

            for (int i = 0; i < countVar; i++) {
//                out.write(String.format("%d,%d,%d,%d,%d,", i,  flCounts[i], frCounts[i], brCounts[i], blCounts[i]));
                out.write(String.format("%.3f,%d,%d,%d,%d,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f",
                        timeArray[i], flIMU[i], frIMU[i], brIMU[i], blIMU[i], jackIMU[i],
                        arrayFLBR[i],arrayFRBL[i],arrayRobotY[i], arrayRobotX[i], arrayRobotAngle[i],
                        arrayFieldY[i], arrayFieldX[i]));
                out.write(String.format("\n"));
            }


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
    public void writeExtrasToFile(DataOutputStream fileData){
        int countVar = Math.max(size, IMUCounter);

        try {
//          Write the data in binary format that can be read back in by the Java plotting programs in IntelliJ
//          Only writing out the FieldLocation X,Y, Theta as formatted for reading in, Used for foundations, and stones so could also add robot

            for (int j = 0; j < countVar; j++) {
                // writes the bytes for each double in the array
                fileData.writeInt(jackIMU[j]);   // FieldLocation X position on field in inches
                fileData.writeDouble(gripIMU[j]);   // FieldLocation Y position on field in inches
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
        RobotVincent.imu.flCnt = RobotVincent.frontLeft.getCurrentPosition();
        RobotVincent.imu.frCnt = RobotVincent.frontRight.getCurrentPosition();
        RobotVincent.imu.brCnt = RobotVincent.backRight.getCurrentPosition();
        RobotVincent.imu.blCnt = RobotVincent.backLeft.getCurrentPosition();

        RobotVincent.imu.jackCnt = RobotVincent.jack.getCurrentPosition();
        RobotVincent.imu.gripCnt = RobotVincent.gripper.getCurrentPosition();


        RobotVincent.imu.haveBlueFoundation = haveBlueFoundation;
        RobotVincent.imu.haveRedFoundation = haveRedFoundation;
        //%%%%%%%%%%%%%%%%%% UPDATED BELOW 10/27/19 FOR 4 ROBOTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        RobotVincent.imu.jackCnt = RobotVincent.jack.getCurrentPosition();
        RobotVincent.imu.gripCnt = RobotVincent.gripper.getCurrentPosition();
        RobotVincent.imu.haveBlueSkyStone = haveBlueSkyStone;
        RobotVincent.imu.haveRedSkyStone = haveRedSkyStone;
        //%%%%%%%%%%%%%%%%%% UPDATED ABOVE 10/27/19 FOR 4 ROBOTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


        try {
//                distanceTraveledArray[counter] = distanceTraveledInch;
               counter += 1;
//               RobotVincent.imu.counter = counter;
//                RobotVincent.imu.updateCounter(counter);
                if (counter >= size) {
                    int a = 1 / 0;
                }
            } catch (ArithmeticException e) {
                System.out.println(String.format("Exceeded %d counter steps", size));
//                distanceTraveledInch = driveDistance;
            }


    }

    //----------------------------------------------------------------------------------------------
    // TEST MODE METHOD prepOpMode is used to initialize offline items
    //----------------------------------------------------------------------------------------------
   public void prepOpMode() {
       //This is the offline version of init that has additional items
       //Needs to be run before runOpMode or inherited init
       //Added code to try to use library version of hardware
       RobotVincent.TestMode = true;
       RobotVincent.init(hardwareMap);


       //***************** MODIFIED TO LOAD FROM PC ********************
       telemetry.addData("Loading from","%s",fileNameEdited);
       telemetry.addData("loadFile ","%s",loadFile);
       telemetry.update();

       //Change method to loadfromPC
       constants.phm = constants.loadFromPC(fileNameEdited, constants.phm,this);
       if(!loadFile) {
//           HashMap hasn't been created, need to create file
           telemetry.addLine("HashMap File did not Exist");
           telemetry.addLine(String.format("loadFile =  %s",loadFile));
           telemetry.update();
           telemetry.addLine(String.format("Defining HashMap and Writing file to %s", fileNameEdited));
           constants.defineParams();
           //Change to savetoFile
           constants.saveToFile(fileNameEdited, constants.phm);
           telemetry.addLine(String.format("loadFile =  %s",loadFile));
           telemetry.update();

       }
       else {
           telemetry.addLine("HashMap File Successfully Loaded");
           telemetry.addLine(String.format("loadFile =  %s",loadFile));
           telemetry.update();
       }
       telemetry.addLine("HashMap Load Completed");
       telemetry.addLine("-------------------------------------------");

       telemetry.update();

       //*************** END MODIFIED FOR LOAD ***************************************
       //Motor configuration, recommend Not Changing - Set all motors to forward direction, positive = clockwise when viewed from shaft side
       RobotVincent.frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
       RobotVincent.frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
       RobotVincent.backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
       RobotVincent.backRight.setDirection(DcMotorSimple.Direction.FORWARD);

       RobotVincent.jack.setDirection(DcMotorSimple.Direction.FORWARD);
       RobotVincent.gripper.setDirection(DcMotorSimple.Direction.FORWARD);


       RobotVincent.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       RobotVincent.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       RobotVincent.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       RobotVincent.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       RobotVincent.jack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       RobotVincent.gripper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


       RobotVincent.frontLeft.setTargetPosition(0);
       RobotVincent.frontRight.setTargetPosition(0);
       RobotVincent.backLeft.setTargetPosition(0);
       RobotVincent.backRight.setTargetPosition(0);

       RobotVincent.jack.setTargetPosition(0);
       RobotVincent.gripper.setTargetPosition(0);

       //Reset all motor encoders
       RobotVincent.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       RobotVincent.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       RobotVincent.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       RobotVincent.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

       RobotVincent.jack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       RobotVincent.gripper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

       //Set all motors to position mode (assumes that all motors have encoders on them)
       RobotVincent.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       RobotVincent.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       RobotVincent.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       RobotVincent.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

       RobotVincent.jack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       RobotVincent.gripper.setMode(DcMotor.RunMode.RUN_TO_POSITION);

       RobotVincent.frontLeft.setPower(0);
       RobotVincent.frontRight.setPower(0);
       RobotVincent.backRight.setPower(0);
       RobotVincent.backLeft.setPower(0);

       RobotVincent.jack.setPower(0);
       RobotVincent.gripper.setPower(0);

       //***********************************************************
       //Code that needs to be Kept in init to initialize functions
       //***********************************************************

       RobotVincent.imu.timeStep = timeStep;
       RobotVincent.frontLeft.timeStep = timeStep;
       RobotVincent.frontRight.timeStep = timeStep;
       RobotVincent.backRight.timeStep = timeStep;
       RobotVincent.backLeft.timeStep = timeStep;

       RobotVincent.jack.timeStep = timeStep;
       RobotVincent.gripper.timeStep = timeStep;


       RobotVincent.imu.RedFoundationPoints.add(new FieldLocation(RobotVincent.imu.redFoundX,RobotVincent.imu.redFoundY,RobotVincent.imu.redFoundTheta));
       RobotVincent.imu.BlueFoundationPoints.add(new FieldLocation(RobotVincent.imu.blueFoundX,RobotVincent.imu.blueFoundY,RobotVincent.imu.blueFoundTheta));
       //%%%%%%%%%%%%%%%%%% UPDATED BELOW 10/27/19 FOR 4 ROBOTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       RobotVincent.imu.BlueSkyStonePoints.add(new FieldLocation(RobotVincent.imu.blueStoneX,RobotVincent.imu.blueStoneY,RobotVincent.imu.blueStoneTheta));
       RobotVincent.imu.RedSkyStonePoints.add(new FieldLocation(RobotVincent.imu.redStoneX,RobotVincent.imu.redStoneY,RobotVincent.imu.redStoneTheta));
       RobotVincent.imu.GripperPoints.add(new FieldLocation(RobotVincent.imu.gripperX,RobotVincent.imu.gripperY,RobotVincent.imu.gripperTheta));

       //Setting counter to capture array data is unique to offline running of code

       counter = 1;
       RobotVincent.imu.counter = counter;

       RobotVincent.robotNumber = 4;
       /* robotNumber sets blue or red and initial startins positions and tasks as follows:
       * 1 = Blue1 which is skystone side (LH side, -X, -Y, angle = 0)
       * 2 = Blue2 which is foundation side (LH side, -X +Y, angle = 0)
       * 3 = Red1 which is skystone side (RH side, +X, -Y, angle = 180)
       * 4 = Red2 which is foundation side (RH side, +X, +Y, angle = 180)
       */

       if(RobotVincent.robotNumber == 1) {

           //field angle orientation is + = CCW , while robot frame is + = CW

           RobotVincent.imu.fieldX = -63;//initial x position on field in inches
           RobotVincent.imu.fieldY = -28;//initial y position on field in inches
           RobotVincent.imu.priorAngle = 0;//initial robot angle orientation on field in degrees from EAST
           RobotVincent.imu.fakeAngle = 0;//initial robot angle orientation on field in degrees from EAST
       }


       if(RobotVincent.robotNumber == 2) {

           constants.phm.get("DRIVE_POWER_LIMIT").setValue(0.85);
           constants.phm.get("squareDistance").setValue(32);
//           RobotVincent.frontLeft.motorTol=1.2;
//           RobotVincent.backLeft.motorTol=1.2;
//
           //field angle orientation is + = CCW , while robot frame is + = CW
           RobotVincent.imu.fieldX = -63;//initial x position on field in inches
           RobotVincent.imu.fieldY = 48;//initial y position on field in inches
           RobotVincent.imu.priorAngle = 0;//initial robot angle orientation on field in degrees from EAST
           RobotVincent.imu.fakeAngle = 0;//initial robot angle orientation on field in degrees from EAST

       }
       if(RobotVincent.robotNumber == 3) {

           constants.phm.get("DRIVE_POWER_LIMIT").setValue(0.7);
           constants.phm.get("squareDistance").setValue(16);
//           RobotVincent.frontLeft.motorTol=1.05;
//           RobotVincent.backLeft.motorTol=1.05;
//
           //field angle orientation is + = CCW , while robot frame is + = CW
           RobotVincent.imu.fieldX = 63;//initial x position on field in inches
           RobotVincent.imu.fieldY = -28;//initial y position on field in inches
           RobotVincent.imu.priorAngle = 180;//initial robot angle orientation on field in degrees from EAST
           RobotVincent.imu.fakeAngle = 180;//initial robot angle orientation on field in degrees from EAST

       }
       if(RobotVincent.robotNumber == 4) {

           constants.phm.get("DRIVE_POWER_LIMIT").setValue(0.8);
           constants.phm.get("squareDistance").setValue(22);
//           RobotVincent.frontLeft.motorTol=1.2;
//           RobotVincent.backLeft.motorTol=1.2;
//
           //field angle orientation is + = CCW , while robot frame is + = CW
           RobotVincent.imu.fieldX = 63;//initial x position on field in inches
           RobotVincent.imu.fieldY = 48;//initial y position on field in inches
           RobotVincent.imu.priorAngle = 180;//initial robot angle orientation on field in degrees from EAST
           RobotVincent.imu.fakeAngle = 180;//initial robot angle orientation on field in degrees from EAST

       }
       telemetry.addData("Robot Number ", "%d",RobotVincent.robotNumber);
       telemetry.addData("DRIVE_POWER_LIMIT ", "%.2f",constants.phm.get("DRIVE_POWER_LIMIT").value);
       telemetry.addData("squareDistance ", "%.2f",constants.phm.get("squareDistance").value);
       telemetry.update();
       //%%%%%%%%%%%%%%%%%% UPDATED ABOVE 10/27/19 FOR 4 ROBOTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

       //Initialize starting position on field, field cent eris assumed (0,0), 0 field angle is pointing EAST
       RobotVincent.imu.fieldXArray[0] = RobotVincent.imu.fieldX; //initial x position on field in inches
       RobotVincent.imu.fieldYArray[0] = RobotVincent.imu.fieldY; //initial y position on field in inches
       RobotVincent.imu.robotAngleArray[0] = RobotVincent.imu.priorAngle; //initial robot angle orientation on field in degrees from EAST


   }

    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    // INSERT ACTUAL CODE TO BE TESTED IN METHODS
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //----------------------------------------------------------------------------------------------
    // TEST MODE METHOD runAutonomous is populated with actual program code in runOpMode
    //----------------------------------------------------------------------------------------------
    @Override
    public void initOpMode() {


    }
    @Override
    public void runOpMode(){
        initOpMode(); // run initialization from init in this OpMode

        //%%%%%%%%%%%%%%%%%% UPDATED BELOW 10/27/19 FOR 4 ROBOTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        if(RobotVincent.robotNumber ==1) {
            getBlueStone();
        }
        if(RobotVincent.robotNumber ==2) {
            getBlueFoundation();
        }
        if(RobotVincent.robotNumber ==3) {
            getRedStone();
        }
        if(RobotVincent.robotNumber ==4) {
            getRedFoundation();
        }
        //%%%%%%%%%%%%%%%%%% UPDATED ABOVE 10/27/19 FOR 4 ROBOTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




    } //MAIN OpMode PROGRAM END



    //##############################################################################################
    // END ACTUAL CODE TO BE TESTED AS METHODS
    //##############################################################################################

    //Run Calculations like Autonomous OpMode
    public static void main(String []args)throws IOException {

// Code to setup the main program that runs offline, none of this is robot code
        OfflineOpModeLibs OffLibs = new OfflineOpModeLibs();
        OffLibs.testMode = true;//Declare Test Mode
//        OffLibs.testOpMode = true;
        // Prepare robot class for offline operation, must be run prior to copied runOpMode or init
        // Sets initial position and counters and initial array variables
        OffLibs.prepOpMode();

//
        StringBuilder fileName = new StringBuilder();
        fileName.append("RMO_");
        fileName.append(String.format("%s.csv","RuckusAutonomous01"));
        out = new FileWriter(fileName.toString());
        //============= SET ROBOT 1, 2, 3,or 4 to WRITE DATA ==============
//        fileOutStream = new FileOutputStream("C:/Users/Spiessbach/Documents/GitHub/OfflineViz/RobotOnField.dat");// Path to directory for IntelliJ code
//        fileOutStream = new FileOutputStream("C:/Users/Spiessbach/Documents/GitHub/OfflineCode/Robot1OnField.dat");// Path to directory for IntelliJ code
        fileOutStream = new FileOutputStream("C:/Users/Spiessbach/Documents/FTC/IntelliJ Projects/RobotVisualization/"+String.format("Robot%dOnField.dat",OffLibs.RobotVincent.robotNumber));// Path to directory for IntelliJ code

        dataOutStream = new DataOutputStream(fileOutStream);
//calls code input by programmer into runAutonomous method that comes from main runOpMode
        OffLibs.runOpMode();

// Lines below are to capture the array data and output
        OffLibs.extractArrayData();
        OffLibs.writeToFile(out, dataOutStream);
//        OffLibs.writeToScreen();
        fileOutStream.close();


        //%%%%%%%%%%%%%%%%%% UPDATED BELOW 10/27/19 FOR 4 ROBOTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        dos = new DataOutputStream(new FileOutputStream("C:/Users/Spiessbach/Documents/FTC/IntelliJ Projects/RobotVisualization/"+String.format("Robot%dAccessories.dat",OffLibs.RobotVincent.robotNumber)));// Path to directory for IntelliJ code
        OffLibs.writeExtrasToFile(dos);
        dos = new DataOutputStream(new FileOutputStream("C:/Users/Spiessbach/Documents/FTC/IntelliJ Projects/RobotVisualization/"+String.format("Robot%dGripper.dat",OffLibs.RobotVincent.robotNumber)));// Path to directory for IntelliJ code
        OffLibs.writeListToFile(dos, OffLibs.RobotVincent.imu.GripperPoints);

        if(OffLibs.RobotVincent.robotNumber ==1) {
            dos = new DataOutputStream(new FileOutputStream("C:/Users/Spiessbach/Documents/FTC/IntelliJ Projects/RobotVisualization/BlueSkyStone.dat"));// Path to directory for IntelliJ code
            OffLibs.writeListToFile(dos, OffLibs.RobotVincent.imu.BlueSkyStonePoints);
        }
        if(OffLibs.RobotVincent.robotNumber ==2) {
            dos = new DataOutputStream(new FileOutputStream("C:/Users/Spiessbach/Documents/FTC/IntelliJ Projects/RobotVisualization/BlueFoundation.dat"));// Path to directory for IntelliJ code
            OffLibs.writeListToFile(dos, OffLibs.RobotVincent.imu.BlueFoundationPoints);
        }
        if(OffLibs.RobotVincent.robotNumber ==3) {
            dos = new DataOutputStream(new FileOutputStream("C:/Users/Spiessbach/Documents/FTC/IntelliJ Projects/RobotVisualization/RedSkyStone.dat"));// Path to directory for IntelliJ code
            OffLibs.writeListToFile(dos, OffLibs.RobotVincent.imu.RedSkyStonePoints);
        }
        if(OffLibs.RobotVincent.robotNumber ==4) {
            dos = new DataOutputStream(new FileOutputStream("C:/Users/Spiessbach/Documents/FTC/IntelliJ Projects/RobotVisualization/RedFoundation.dat"));// Path to directory for IntelliJ code
            OffLibs.writeListToFile(dos, OffLibs.RobotVincent.imu.RedFoundationPoints);
        }
        //%%%%%%%%%%%%%%%%%% UPDATED ABOVE 10/27/19 FOR 4 ROBOTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


        OffLibs.telemetry.addData("Total Number of Time Steps", "%d",OffLibs.IMUCounter);
        OffLibs.telemetry.update();

    }


}