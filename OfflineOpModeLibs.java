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
    public double timeStep = 1000 * 30/size;//determined a fixed time step (in milliseconds) so that faster speeds will show shorter time to distance
    //timeStep was 100 in seconds to fill 30 seconds / size of array (counts to 30
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

// ----------------- ADDED CODE NOT IN "coach" version ERROR? ---------------------
        //missing 1 point in field array vs robot array list
        fc.BlueFoundationPoints.add(fc.BlueFoundationPoints.get(IMUCounter-2));
        fc.RedFoundationPoints.add(fc.RedFoundationPoints.get(IMUCounter-2));

        fc.RedSkyStone1Points.add(fc.RedSkyStone1Points.get(IMUCounter-2));
        fc.BlueSkyStone1Points.add(fc.BlueSkyStone1Points.get(IMUCounter-2));

        fc.BlueSkyStone2Points.add(fc.BlueSkyStone2Points.get(IMUCounter-2));
        fc.RedSkyStone2Points.add(fc.RedSkyStone2Points.get(IMUCounter-2));

// ----------------- ADDED CODE NOT IN "coach" version ERROR? ---------------------

        double deltaTime = (timeArray[1] - timeArray[0]);
        for(int k = IMUCounter; k < size;k++){// needed to reduce counter by 1 -- means there is an extra count somewhere
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

       //This is the offline version of init that has additional items
       //Needs to be run before runOpMode or inherited init
       //Added code to try to use library version of hardware
       Billy.isTestMode = true;
//       activeOpMode = true;// already done in MAIN

       stoneSelect = 1;
       fc = new FieldConfiguration(stoneSelect);//KS added 12/20 to set stone position

       // Map all of hardware to program from robot using HardwareVincent script
       //Will refer all defined configuration items - motors, servos, sensors- to this defined Billy class
//        createRobot();

       Billy.init(hardwareMap);

       //Motor configuration, recommend Not Changing - Set all motors to forward direction, positive = clockwise when viewed from shaft side
       Billy.frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
       Billy.frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
       Billy.backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
       Billy.backRight.setDirection(DcMotorSimple.Direction.FORWARD);

       Billy.jackLeadScrew.setDirection(DcMotorSimple.Direction.FORWARD);
       Billy.gripper.setDirection(DcMotorSimple.Direction.FORWARD);


       Billy.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       Billy.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       Billy.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       Billy.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       Billy.jackLeadScrew.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       Billy.gripper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


       //Reset all motor encoders
       Billy.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       Billy.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       Billy.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       Billy.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

       Billy.frontLeft.setTargetPosition(0);
       Billy.frontRight.setTargetPosition(0);
       Billy.backLeft.setTargetPosition(0);
       Billy.backRight.setTargetPosition(0);

       Billy.jackLeadScrew.setTargetPosition(0);
       Billy.gripper.setTargetPosition(0);


       //Set all motors to position mode (assumes that all motors have encoders on them)
       Billy.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       Billy.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       Billy.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       Billy.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

       Billy.jackLeadScrew.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       Billy.gripper.setMode(DcMotor.RunMode.RUN_TO_POSITION);


       Billy.frontLeft.setPower(0);
       Billy.frontRight.setPower(0);
       Billy.backLeft.setPower(0);
       Billy.backRight.setPower(0);

       Billy.jackLeadScrew.setPower(0);
       Billy.gripper.setPower(0);


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

       Billy.jackLeadScrew.timeStep = timeStep;
       Billy.gripper.timeStep = timeStep;

       fc.RedFoundationPoints.clear();
       fc.RedFoundationPoints.add(new FieldLocation(fc.redFound.x,fc.redFound.y,fc.redFound.theta));

       fc.BlueFoundationPoints.clear();
       fc.BlueFoundationPoints.add(new FieldLocation(fc.blueFound.x,fc.blueFound.y,fc.blueFound.theta));

       fc.BlueSkyStone1Points.clear();
       fc.BlueSkyStone1Points.add(new FieldLocation(fc.blueStone1.x,fc.blueStone1.y,fc.blueStone1.theta));

       fc.RedSkyStone1Points.clear();
       fc.RedSkyStone1Points.add(new FieldLocation(fc.redStone1.x,fc.redStone1.y,fc.redStone1.theta));

       fc.BlueSkyStone2Points.clear();
       fc.BlueSkyStone2Points.add(new FieldLocation(fc.blueStone2.x,fc.blueStone2.y,fc.blueStone2.theta));

       fc.RedSkyStone2Points.clear();
       fc.RedSkyStone2Points.add(new FieldLocation(fc.redStone2.x,fc.redStone2.y,fc.redStone2.theta));

       Billy.imu.GripperPoints.clear();
       Billy.imu.GripperPoints.add(new FieldLocation(Billy.imu.gripperX, Billy.imu.gripperY, Billy.imu.gripperTheta));

       Billy.imu.RobotPoints.clear();
       Billy.imu.RobotPoints.add(new FieldLocation(Billy.imu.robotOnField.x, Billy.imu.robotOnField.y, Billy.imu.robotOnField.theta));

       //Setting counter to capture array data is unique to offline running of code
       counter = 1;
       Billy.imu.counter = counter;
//
//       Billy.robotNumber = 1;//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//       foundationPosChange = 0;// 26 for unmoved Foundation, 0 for moved Foundation
//       insideOutside = 0;// 0 for Inside, 24 for Outside
//       foundationInOut = 26;// 0 for Inside, 26 for Outside
//       sideColor = 1;// + for Blue, - for Red

       if(Billy.robotNumber == 1) {
           cons.pHM.get("drivePowerLimit").setParameter(0.75);
//           Billy.frontLeft.motorTol=1.2;
//           Billy.backLeft.motorTol=1.2;
           //field angle orientation is + = CCW , while robot frame is + = CW
           Billy.imu.robotOnField.x = -63;//initial x position on field in inches
           Billy.imu.robotOnField.y = -28;//initial y position on field in inches
           Billy.imu.robotOnField.theta = 0;//initial robot angle orientation on field in degrees from EAST
           Billy.imu.priorAngle = 0;//initial robot angle orientation on field in degrees from EAST
           Billy.imu.fakeAngle = 0;//initial robot angle orientation on field in degrees from EAST
       }

       if(Billy.robotNumber == 2) {

           cons.pHM.get("drivePowerLimit").setParameter(0.75);
//           Billy.frontLeft.motorTol=1.2;
//           Billy.backLeft.motorTol=1.2;
//
           //field angle orientation is + = CCW , while robot frame is + = CW
           Billy.imu.robotOnField.x = -63;//initial x position on field in inches
           Billy.imu.robotOnField.y = 48;//initial y position on field in inches
           Billy.imu.robotOnField.theta = 180;//initial robot angle orientation on field in degrees from EAST
           Billy.imu.priorAngle = 180;//initial robot angle orientation on field in degrees from EAST
           Billy.imu.fakeAngle = 180;//initial robot angle orientation on field in degrees from EAST
           telemetry.addData("Robot Number ", "%d",Billy.robotNumber);
           telemetry.addData("drivePowerLimit ", "%.2f",cons.pHM.get("drivePowerLimit").value);

           telemetry.update();
       }

       if(Billy.robotNumber == 3) {
           cons.pHM.get("drivePowerLimit").setParameter(0.75);
//           Billy.frontLeft.motorTol=1.2;
//           Billy.backLeft.motorTol=1.2;
           //field angle orientation is + = CCW , while robot frame is + = CW
           Billy.imu.robotOnField.x = 63;//initial x position on field in inches
           Billy.imu.robotOnField.y = -28;//initial y position on field in inches
           Billy.imu.robotOnField.theta = 180;//initial robot angle orientation on field in degrees from EAST
           Billy.imu.priorAngle = 180;//initial robot angle orientation on field in degrees from EAST
           Billy.imu.fakeAngle = 180;//initial robot angle orientation on field in degrees from EAST
       }

       if(Billy.robotNumber == 4) {

           Billy.imu.robotOnField.x = 63;//initial x position on field in inches
           Billy.imu.robotOnField.y = 48;//initial y position on field in inches
           Billy.imu.robotOnField.theta = 0;//initial robot angle orientation on field in degrees from EAST
           Billy.imu.priorAngle = 0;//initial robot angle orientation on field in degrees from EAST
           Billy.imu.fakeAngle = 0;//initial robot angle orientation on field in degrees from EAST
           telemetry.addData("Robot Number ", "%d",Billy.robotNumber);
           telemetry.addData("drivePowerLimit ", "%.2f",cons.pHM.get("drivePowerLimit").value);

           telemetry.update();
       }

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

        switch(location){
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
    }
    @Override
    public void runOpMode(){

        initialize();

        runtime.reset();

        if(Billy.robotNumber ==1 || Billy.robotNumber ==3) {

            fwdToStone();

            findSkyStone();

            crossDropStoneFor2();

            getSecondStone();

        }
        if(Billy.robotNumber ==2 || Billy.robotNumber ==4 ) {
            if(foundationPosChange == 26){

                drv.driveGeneral(DriveMethods.moveDirection.RightLeft,-50*sideColor, cons.pHM.get("drivePowerLimit").value, "Left 30 inches",this);
                //KS COMMENT 12/20/19 - need to add sideColor to correct motion for blue/red
                if(Billy.robotNumber ==2){writeBF = true;}
                if(Billy.robotNumber ==4){writeRF = true;}
                // Added if statements to write foundation files to clear old data
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

        OffLibs.location = computer.PC;//For Karl on HP
//        OffLibs.location = computer.MAC;//For Caleb
//        OffLibs.location = computer.WILL;//For William


        for(int h = 1; h<5;h++) {

            OffLibs.Billy.robotNumber = h;//!!!!!!!!!!!!!1!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            OffLibs.foundationPosChange = 26;// 26 for unmoved Foundation, 0 for moved Foundation
            OffLibs.insideOutside = 24;// 0 for Inside, 24 for Outside
            OffLibs.foundationInOut = 0;// 0 for Inside, 26 for Outside
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

            fos = new FileOutputStream(OffLibs.fileLocation + String.format("Robot%dOnField.txt", OffLibs.Billy.robotNumber));// Path to directory for IntelliJ code
            OffLibs.fc.writeFieldAsText(fos, OffLibs.Billy.imu.RobotPoints, countVar);

            fos = new FileOutputStream(OffLibs.fileLocation + String.format("Robot%dAccessories.txt", OffLibs.Billy.robotNumber));// Path to directory for IntelliJ code
            OffLibs.writeExtrasToFile(fos);

            fos = new FileOutputStream(OffLibs.fileLocation + String.format("Robot%dGripper.txt", OffLibs.Billy.robotNumber));// Path to directory for IntelliJ code
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