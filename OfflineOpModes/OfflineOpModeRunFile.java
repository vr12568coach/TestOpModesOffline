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


import java.io.DataOutputStream;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.ArrayList;
import java.util.Arrays;


import OfflineCode.Field.FieldConfiguration;
import UltimateGoal_RobotTeam.HarwareConfig.HardwareRobotMulti;
import UltimateGoal_RobotTeam.OpModes.Autonomous.BasicAuto;
import UltimateGoal_RobotTeam.OpModes.Autonomous.CompleteAutonomousBlueExterior;
import UltimateGoal_RobotTeam.OpModes.Autonomous.CompleteAutonomousBlueInterior;
//import UltimateGoal_RobotTeam.OpModes.Autonomous.DoNothingBlueExt;
//import UltimateGoal_RobotTeam.OpModes.Autonomous.DoNothingBlueInt;
//import UltimateGoal_RobotTeam.OpModes.Autonomous.DoNothingRedExt;
//import UltimateGoal_RobotTeam.OpModes.Autonomous.DoNothingRedInt;
//import UltimateGoal_RobotTeam.OpModes.Autonomous.RedExteriorWobbleHighGoal;
//import UltimateGoal_RobotTeam.OpModes.Autonomous.RedInteriorWobbleHighGoal;
import UltimateGoal_RobotTeam.Utilities.PursuitLines;

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
    //********** REMOVED from OfflineOpMode files to BasicAuto for OfflineOpModeRunFile ******************

//    private FieldConfiguration fc = new FieldConfiguration();
//
//    boolean writeBR = false;
//    boolean writeRR = false;
//    boolean writeBW1 = false;
//    boolean writeBW2 = false;
//    boolean writeRW1 = false;
//    boolean writeRW2 = false;
//    boolean robotSeeRing = false;
//    int IMUCounter =0;
    final static int size = 300;
    //********** REMOVED from OfflineOpMode files to BasicAuto for OfflineOpModeRunFile ******************

    private int robotNumber = 1;

//Instantiate the Autonomous OpMode you wish to Run - can be multiple for each of 4 robots
// Below being used as the surrogate robot so multiple autonomous OpModes can be contained in a single run
  public BasicAuto autoOpMode = new BasicAuto();// Needed for Offline Code

    /* DON'T NEED TO INSTANTIATE THE OPMODES HERE, DONE WHEN RUNNING */
//    CompleteAutonomousBlueExterior BlueExt = new CompleteAutonomousBlueExterior();
//    CompleteAutonomousBlueInterior BlueInt = new CompleteAutonomousBlueInterior();//Needs to be updated before use
    /* CREATE RED OpModes for Future */

    // Finish instantiations

    int counter;
    int[] flCounts = new int[size];
    int[] frCounts = new int[size];
    int[] brCounts = new int[size];
    int[] blCounts = new int[size];
    int[] flIMU = new int[size];
    int[] frIMU = new int[size];
    int[] brIMU = new int[size];
    int[] blIMU = new int[size];

//    int[] collectorArray = new int[size];
//    int[] conveyorArray = new int[size];
//    int[] shooterArray = new int[size];
//    double[] wgaAngleArray = new double[size];

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
    /* Moved time step to BasicOpMode */



    static FileReader in = null;
    static FileWriter out = null;
    static FileOutputStream fileOutStream = null;
    static DataOutputStream dataOutStream = null;
    static DataOutputStream dos = null;
    static FileOutputStream fos = null;
    public String SelectedOpMode = null;

    static String fileLocation;
    static computer location;

    public boolean opModeIsRunning = true;


    //Constructor
    public OfflineOpModeRunFile(BasicAuto inputOpMode){
        autoOpMode = inputOpMode;
    };
    public OfflineOpModeRunFile(){

    };


    //----------------------------------------------------------------------------------------------
    // TEST MODE CODE ONLY - EXTRACTS ARRAY DATA FROM classes
    //----------------------------------------------------------------------------------------------
    public void extractArrayData(){


        flIMU = autoOpMode.robotUG.driveTrain.imu.flArray;
        frIMU = autoOpMode.robotUG.driveTrain.imu.frArray;
        brIMU = autoOpMode.robotUG.driveTrain.imu.brArray;
        blIMU = autoOpMode.robotUG.driveTrain.imu.blArray;

        timeArray = autoOpMode.robotUG.driveTrain.imu.timeArray;

        arrayRobotX = autoOpMode.robotUG.driveTrain.imu.robotXArray;
        arrayRobotY = autoOpMode.robotUG.driveTrain.imu.robotYArray;
        arrayRobotDist = autoOpMode.robotUG.driveTrain.imu.robotDistArray;
        arrayRobotAngle = autoOpMode.robotUG.driveTrain.imu.robotAngleArray;
        IMUCounter = autoOpMode.robotUG.driveTrain.imu.counter;

        arrayFLBR = autoOpMode.robotUG.driveTrain.imu.FLBRArray;
        arrayFRBL = autoOpMode.robotUG.driveTrain.imu.FRBLArray;


        Arrays.fill(arrayRobotDist,IMUCounter,(size),arrayRobotDist[IMUCounter-1]);
        Arrays.fill(arrayRobotAngle,IMUCounter,(size),arrayRobotAngle[IMUCounter-1]);

        Arrays.fill(arrayFieldDist,IMUCounter,(size),arrayFieldDist[IMUCounter-1]);
        Arrays.fill(autoOpMode.collectorArray,IMUCounter,(size), autoOpMode.collectorArray[IMUCounter-1]);
        Arrays.fill(autoOpMode.conveyorArray,IMUCounter,(size), autoOpMode.conveyorArray[IMUCounter-1]);
        Arrays.fill(autoOpMode.shooterArray,IMUCounter,(size), autoOpMode.shooterArray[IMUCounter-1]);
        Arrays.fill(autoOpMode.wgaAngleArray,IMUCounter,(size), autoOpMode.wgaAngleArray[IMUCounter-1]);

        double deltaTime = (timeArray[1] - timeArray[0]);
//        for(int k = IMUCounter-1; k < size;k++){// needed to reduce counter by 1 -- means there is an extra count somewhere
        for(int k = IMUCounter+1; k < size;k++){// updated to only add points
            timeArray[k] = timeArray[k-1] + deltaTime;
            autoOpMode.fc.RobotPoints.add(autoOpMode.fc.RobotPoints.get(k-1));

            autoOpMode.fc.BlueRingPoints.add(autoOpMode.fc.BlueRingPoints.get(k-1));
            autoOpMode.fc.RedRingPoints.add(autoOpMode.fc.RedRingPoints.get(k-1));

            autoOpMode.fc.RedWobble1Points.add(autoOpMode.fc.RedWobble1Points.get(k-1));
            autoOpMode.fc.BlueWobble1Points.add(autoOpMode.fc.BlueWobble1Points.get(k-1));

            autoOpMode.fc.BlueWobble2Points.add(autoOpMode.fc.BlueWobble2Points.get(k-1));
            autoOpMode.fc.RedWobble2Points.add(autoOpMode.fc.RedWobble2Points.get(k-1));
            autoOpMode.fc.PursuitPoints.add(autoOpMode.fc.PursuitPoints.get(k-1));
            autoOpMode.fc.NavPoints.add(autoOpMode.fc.NavPoints.get(k-1));

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
                osw.write(Integer.toString(autoOpMode.collectorArray[j])+"\t");   // collector Power - use to determine if ON
                osw.write(Integer.toString(autoOpMode.conveyorArray[j])+"\t");   // conveyor Power - use to determine if ON
                osw.write(Integer.toString(autoOpMode.shooterArray[j])+"\t");   // shooter Power - use to determine if ON
                osw.write(Double.toString(autoOpMode.wgaAngleArray[j])+"\n");   // Wobble Goal Arm angle / motion
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
//    @Override
//    public void updateIMU(){
//        //Moved this method content to BasicopMode so that it is inherited in all BasicOpModes
//        //Otherwise the OpModes run the blank BasicOpMode code
//        if(testModeActive) {
//            autoOpMode.robotUG.driveTrain.imu.flCnt = autoOpMode.robotUG.driveTrain.frontLeft.getCurrentPosition();
//            autoOpMode.robotUG.driveTrain.imu.frCnt = autoOpMode.robotUG.driveTrain.frontRight.getCurrentPosition();
//            autoOpMode.robotUG.driveTrain.imu.brCnt = autoOpMode.robotUG.driveTrain.backRight.getCurrentPosition();
//            autoOpMode.robotUG.driveTrain.imu.blCnt = autoOpMode.robotUG.driveTrain.backLeft.getCurrentPosition();
//
//            IMUCounter = autoOpMode.robotUG.driveTrain.imu.counter;
//
//            collectorArray[IMUCounter] = (int) Math.round(robotUG.collector.collectorPower);
//            conveyorArray[IMUCounter] = (int) Math.round(robotUG.conveyor.conveyor_Power);
//            shooterArray[IMUCounter] = (int) Math.round(robotUG.shooter.getShooter_Power());
//            wgaAngleArray[IMUCounter] = robotUG.wobbleArm.getArmAngleDegrees() * Math.PI / 180.0;
//
//            fc.updateField(this);
//            robotSeeRing = fc.ringFound;
//
//
//            if (autoOpMode.haveBlueRing) {
//                writeBR = true;
//            }
//            if (autoOpMode.haveRedRing) {
//                writeRR = true;
//            }
//            if (autoOpMode.haveBlueWobble1) {
//                writeBW1 = true;
//            }
//            if (autoOpMode.haveBlueWobble2) {
//                writeBW2 = true;
//            }
//            if (autoOpMode.haveRedWobble1) {
//                writeRW1 = true;
//            }
//            if (autoOpMode.haveRedWobble2) {
//                writeRW2 = true;
//            }
//
//            try {
////
//                if (IMUCounter >= size) {
//                    int a = 1 / 0;
//                }
//            } catch (ArithmeticException e) {
//                System.out.println(String.format("Exceeded %d counter steps", size));
//                System.out.println(String.format("IMU Counter = %d", IMUCounter));
//            }
//        }
//
//    }

    //----------------------------------------------------------------------------------------------
    // TEST MODE METHOD prepOpMode is used to initialize offline items - instead of initOpMode
    //----------------------------------------------------------------------------------------------
   public void prepOpMode(int ringNumber) {

   //************* BELOW IS TEST CODE ********************************


       autoOpMode.testModeActive = true;// set for each OpMode


       autoOpMode.ringSelect = ringNumber;// Options are 0, 1, 4 rings on the field
       autoOpMode.fc = new FieldConfiguration(ringSelect);//KS added 12/20 to set stone position

       autoOpMode.haveBlueRing = false;
       autoOpMode.haveRedRing = false;
       autoOpMode.haveBlueWobble1 = false;
       autoOpMode.haveBlueWobble2 = false;
       autoOpMode.haveRedWobble1 = false;
       autoOpMode.haveRedWobble2 = false;

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

//       // --------- INVESTIGATE MOVING SOME CODE TO INITIALIZE METHOD TO USE ACTUAL OpMode CODE ------
//       // %%%%%% UPDATED BELOW FOR HardwareRobotMulti class %%%%%%%%%
//
//       // configure the robot needed - for this demo only need DriveTrain
//       // configArray has True or False values for each subsystem HW element
//       //
//       /** configArray is arranged as
//        * [0] = DriveTrain
//        * [1] = Shooter
//        * [2] = Conveyor
//        * [3] = WobbleArm
//        * [4] = Collector
//        * [5] = ImageRecog
//        * items that are 1 = true will be configured to the robot
//        */
//       // HW ELEMENTS *****************    DriveTrain  Shooter  Conveyor	WobbleArm	Collector	ImageRecog
//       boolean[] configArray = new boolean[]{ true, 	true, 	true, 		true, 		true,		false};
//       /* DON'T START ImageRecog Offline -- Need to troubleshoot */
//       // READ HASHMAP FILE OFFLINE
//       readOrWriteHashMapOffline();
//
//       autoOpMode.robotUG = new HardwareRobotMulti(this, configArray,autoOpMode.testModeActive);
//
//       // Update telemetry to tell driver than robot is ready
//       telemetry.addData("STATUS", "MultiRobot Hardware Configured!!");
//
//       telemetry.addData("Robot Field Location", "X = %.2f inch, Y = %.2f inch, Theta = %.2f degrees",
//               autoOpMode.robotUG.driveTrain.robotFieldLocation.x, autoOpMode.robotUG.driveTrain.robotFieldLocation.y, autoOpMode.robotUG.driveTrain.robotFieldLocation.theta);
//       telemetry.addLine(" ");
//       telemetry.addLine("*********************************************");
//       telemetry.addData("WARNING", "VERIFY THAT DRIVE POWER LIMIT IS LOW FOR INITIAL TESTS");
//       telemetry.addLine("*********************************************");
//       telemetry.addLine(" ");
//       telemetry.addData(">", "Press Play to start");
//       telemetry.update();
//       telemetry.setAutoClear(true);//revert back to telemetry.update clearing prior display
//       //***********************************************************
//       //Code that needs to be Kept in init to initialize functions
//       //***********************************************************

//       MOVED timeStep to constructors on HW items for test mode

       autoOpMode.fc.RedRingPoints.clear();
       autoOpMode.fc.BlueRingPoints.clear();
       autoOpMode.fc.BlueWobble1Points.clear();
       autoOpMode.fc.RedWobble1Points.clear();
       autoOpMode.fc.BlueWobble2Points.clear();
       autoOpMode.fc.RedWobble2Points.clear();

       collectorArray= new int[size];;
       conveyorArray= new int[size];
       shooterArray= new int[size];
       wgaAngleArray= new double[size];

//       autoOpMode.fc.updateField(autoOpMode);



//
//       if(robotNumber == 1) {
//           autoOpMode.cons.DRIVE_POWER_LIMIT = 1.0;
//           autoOpMode.robotUG.driveTrain.frontLeft.motorTol=1.0;
//           autoOpMode.robotUG.driveTrain.frontRight.motorTol=1.0;
//           autoOpMode.robotUG.driveTrain.backRight.motorTol=1.0;
//           autoOpMode.robotUG.driveTrain.backLeft.motorTol=1.0;
//           //field angle orientation is + = CCW , while robot frame is + = CW
////           robotUG.driveTrain.robotFieldLocation.setLocation(-36,-63,90);// FROM CompleteAutonomous
//           autoOpMode.robotUG.driveTrain.imu.robotOnField.x = -36;//initial x position on field in inches
//           autoOpMode.robotUG.driveTrain.imu.robotOnField.y = -63;//initial y position on field in inches
//           autoOpMode.robotUG.driveTrain.imu.robotOnField.theta = 90;//initial robot angle orientation on field in degrees from EAST CCW +
//           autoOpMode.robotUG.driveTrain.imu.priorAngle = autoOpMode.robotUG.driveTrain.imu.robotOnField.theta;//initial robot angle orientation on field in degrees from EAST CCW +
//           autoOpMode.robotUG.driveTrain.imu.fakeAngle = (float) autoOpMode.robotUG.driveTrain.imu.robotOnField.theta;//initial robot angle orientation on field in degrees from EAST CCW +
//           autoOpMode.robotUG.driveTrain.robotHeading = -autoOpMode.robotUG.driveTrain.imu.fakeAngle;//initial robot angle orientation on field in degrees from EAST, CW +
//
//           autoOpMode.robotUG.driveTrain.robotFieldLocation = autoOpMode.robotUG.driveTrain.imu.robotOnField;
//       }
//
//       if(robotNumber == 2) {
//           //           autoOpMode.cons.DRIVE_POWER_LIMIT = 0.75;
//           autoOpMode.robotUG.driveTrain.frontLeft.motorTol=1.0;
//           autoOpMode.robotUG.driveTrain.frontRight.motorTol=1.0;
//           autoOpMode.robotUG.driveTrain.backRight.motorTol=1.0;
//           autoOpMode.robotUG.driveTrain.backLeft.motorTol=1.0;
//           //field angle orientation is + = CCW , while robot frame is + = CW
//           autoOpMode.robotUG.driveTrain.imu.robotOnField.x = -48;//initial x position on field in inches (Added 2 inches for robot 7" to wheel center vs. 9")
//           autoOpMode.robotUG.driveTrain.imu.robotOnField.y = -60;//initial y position on field in inches (WAS 48)
//           autoOpMode.robotUG.driveTrain.imu.robotOnField.theta = 45;//initial robot angle orientation on field in degrees from EAST (WAS 180 for backing to foundation)
//           autoOpMode.robotUG.driveTrain.imu.priorAngle = autoOpMode.robotUG.driveTrain.imu.robotOnField.theta;//initial robot angle orientation on field in degrees from EAST (WAS 180 for backing to foundation)
//           autoOpMode.robotUG.driveTrain.imu.fakeAngle = (float) autoOpMode.robotUG.driveTrain.imu.robotOnField.theta;//initial robot angle orientation on field in degrees from EAST (WAS 180 for backing to foundation)
//           autoOpMode.robotUG.driveTrain.robotHeading = -autoOpMode.robotUG.driveTrain.imu.fakeAngle;//initial robot angle orientation on field in degrees from EAST
//
//           autoOpMode.robotUG.driveTrain.robotFieldLocation = autoOpMode.robotUG.driveTrain.imu.robotOnField;
//       }
//
//       if(robotNumber == 3) {
//           //           autoOpMode.cons.DRIVE_POWER_LIMIT = 0.75;
//           autoOpMode.robotUG.driveTrain.frontLeft.motorTol=1.0;
//           autoOpMode.robotUG.driveTrain.frontRight.motorTol=1.0;
//           autoOpMode.robotUG.driveTrain.backRight.motorTol=1.0;
//           autoOpMode.robotUG.driveTrain.backLeft.motorTol=1.0;
//           //field angle orientation is + = CCW , while robot frame is + = CW
//           autoOpMode.robotUG.driveTrain.imu.robotOnField.x = 24;//initial x position on field in inches (Added 2 inches for robot 7" to wheel center vs. 9")
//           autoOpMode.robotUG.driveTrain.imu.robotOnField.y = -60;//initial y position on field in inches
//           autoOpMode.robotUG.driveTrain.imu.robotOnField.theta = 135;//initial robot angle orientation on field in degrees from EAST
//           autoOpMode.robotUG.driveTrain.imu.priorAngle = autoOpMode.robotUG.driveTrain.imu.robotOnField.theta;//initial robot angle orientation on field in degrees from EAST
//           autoOpMode.robotUG.driveTrain.imu.fakeAngle = (float) autoOpMode.robotUG.driveTrain.imu.robotOnField.theta;//initial robot angle orientation on field in degrees from EAST
//           autoOpMode.robotUG.driveTrain.robotHeading = -autoOpMode.robotUG.driveTrain.imu.fakeAngle;//initial robot angle orientation on field in degrees from EAST
//
//           autoOpMode.robotUG.driveTrain.robotFieldLocation = autoOpMode.robotUG.driveTrain.imu.robotOnField;
//
//       }
//
//       if(robotNumber == 4) {
//           //           autoOpMode.cons.DRIVE_POWER_LIMIT = 0.75;
//           autoOpMode.robotUG.driveTrain.frontLeft.motorTol=1.0;
//           autoOpMode.robotUG.driveTrain.frontRight.motorTol=1.0;
//           autoOpMode.robotUG.driveTrain.backRight.motorTol=1.0;
//           autoOpMode.robotUG.driveTrain.backLeft.motorTol=1.0;
//           //field angle orientation is + = CCW , while robot frame is + = CW
//           autoOpMode.robotUG.driveTrain.imu.robotOnField.x = 48;//initial x position on field in inches (Added 2 inches for robot 7" to wheel center vs. 9")
//           autoOpMode.robotUG.driveTrain.imu.robotOnField.y = -60;//initial y position on field in inches
//           autoOpMode.robotUG.driveTrain.imu.robotOnField.theta = 45;//initial robot angle orientation on field in degrees from EAST
//           autoOpMode.robotUG.driveTrain.imu.priorAngle = autoOpMode.robotUG.driveTrain.imu.robotOnField.theta;//initial robot angle orientation on field in degrees from EAST
//           autoOpMode.robotUG.driveTrain.imu.fakeAngle = (float) autoOpMode.robotUG.driveTrain.imu.robotOnField.theta;//initial robot angle orientation on field in degrees from EAST
//           autoOpMode.robotUG.driveTrain.robotHeading = -autoOpMode.robotUG.driveTrain.imu.fakeAngle;//initial robot angle orientation on field in degrees from EAST
//
//           autoOpMode.robotUG.driveTrain.robotFieldLocation = autoOpMode.robotUG.driveTrain.imu.robotOnField;
//       }
//
//       autoOpMode.robotUG.driveTrain.initIMUtoAngle(-autoOpMode.robotUG.driveTrain.robotFieldLocation.theta);//ADDED HERE FOR OFFLINE, NEEDS TO BE IN initialize() method in OpMode
//       telemetry.addData("Robot Number ", "%d",robotNumber);
//       telemetry.addData("drivePowerLimit ", "%.2f",autoOpMode.cons.DRIVE_POWER_LIMIT);
//       telemetry.addData("Robot Field Location", "X = %.2f inch, Y = %.2f inch, Theta = %.2f degrees",
//               autoOpMode.robotUG.driveTrain.robotFieldLocation.x, autoOpMode.robotUG.driveTrain.robotFieldLocation.y, autoOpMode.robotUG.driveTrain.robotFieldLocation.theta);
//       telemetry.update();
//       autoOpMode.fc.updateField(autoOpMode);

   }

   public void initOfflineIMU(){
       autoOpMode.robotUG.driveTrain.imu.RobotPoints.clear();
       //Setting counter to capture array data is unique to offline running of code
       counter = 1;//Needed for Offline imu since is has compare to previous step
       autoOpMode.robotUG.driveTrain.imu.counter =  counter;//Needed robot constructed to set counter
        //Set IMU positions in Offline HW
       autoOpMode.robotUG.driveTrain.imu.robotOnField.x = autoOpMode.robotUG.driveTrain.robotFieldLocation.x;//initial x position on field in inches
       autoOpMode.robotUG.driveTrain.imu.robotOnField.y = autoOpMode.robotUG.driveTrain.robotFieldLocation.y;//initial y position on field in inches
       autoOpMode.robotUG.driveTrain.imu.robotOnField.theta = autoOpMode.robotUG.driveTrain.robotFieldLocation.theta;//initial robot angle orientation on field in degrees from EAST CCW +
       autoOpMode.robotUG.driveTrain.imu.priorAngle = autoOpMode.robotUG.driveTrain.imu.robotOnField.theta;//initial robot angle orientation on field in degrees from EAST CCW +
       autoOpMode.robotUG.driveTrain.imu.fakeAngle = (float) autoOpMode.robotUG.driveTrain.imu.robotOnField.theta;//initial robot angle orientation on field in degrees from EAST CCW +
       autoOpMode.robotUG.driveTrain.robotHeading = -autoOpMode.robotUG.driveTrain.imu.fakeAngle;//initial robot angle orientation on field in degrees from EAST, CW +

   }

   public enum computer{PC,KARL,MAC,WILL};

    //Run Calculations like Autonomous OpMode
    public static void main(String []args)throws IOException {

        //Instantiate a static class to run the code with
        OfflineOpModeRunFile OffRunFiles = new OfflineOpModeRunFile();

//        OffRunFiles.location = computer.KARL;//For Karl on HP
//  UPDATE SELECTION and PATH for your machine
        OffRunFiles.location = computer.MAC;//For Caleb
//        OffRunFiles.location = computer.WILL;//For William


        // ********** SELECT THE OpMode ************************

        int numRings = 0;//Set ring number here to be used for all 4 robots

        // Set the initial conditions for EACH ROBOT

        for(int h = 1; h<3;h++) {// After creating "Do Nothing" OpModes change to h < 5
            if(h == 1){// Define robot #1
                CompleteAutonomousBlueExterior inputOpMode = new CompleteAutonomousBlueExterior();
//                DoNothingBlueExt inputOpMode = new DoNothingBlueExt();
                OffRunFiles = new OfflineOpModeRunFile(inputOpMode);

                OffRunFiles.writeBR=true;
                OffRunFiles.writeBW1=true;
            }
            else if (h == 2){// Define robot #2
                CompleteAutonomousBlueInterior inputOpMode = new CompleteAutonomousBlueInterior();
//                DoNothingBlueInt inputOpMode = new DoNothingBlueInt();

                OffRunFiles = new OfflineOpModeRunFile(inputOpMode);

//                OffRunFiles.writeBR=true;
                OffRunFiles.writeBW2=true;
            }
            else if (h == 3){// Define robot #3
//                RedInteriorWobbleHighGoal inputOpMode = new RedInteriorWobbleHighGoal();
//                DoNothingRedInt inputOpMode = new DoNothingRedInt();

//                OffRunFiles = new OfflineOpModeRunFile(inputOpMode);

                OffRunFiles.writeRR=true;
                OffRunFiles.writeRW1=true;
            }
            else if (h == 4){// Define robot #4
//                RedExteriorWobbleHighGoal inputOpMode = new RedExteriorWobbleHighGoal();
//                DoNothingRedExt inputOpMode = new DoNothingRedExt();

//                OffRunFiles = new OfflineOpModeRunFile(inputOpMode);

//                OffRunFiles.writeRR=true;
                OffRunFiles.writeRW2=true;
            }
        // Run the code
            OffRunFiles.robotNumber = h;
            OffRunFiles.prepOpMode(numRings);//Prep field with rings
            OffRunFiles.autoOpMode.constructRobot();//Construct robot using Phone OpMode Code
            OffRunFiles.initOfflineIMU();//Pass robot location into Offline HW
            OffRunFiles.autoOpMode.initialize();//Initialize robot  using Phone OpMode Code
            OffRunFiles.autoOpMode.runCode();//Run OpMode Code

// Lines below are to capture the array data and output
            OffRunFiles.extractArrayData();
//            int countVar = Math.max(size, OffRunFiles.IMUCounter);

            fos = new FileOutputStream(OffRunFiles.fileLocation + String.format("Robot%dOnField.txt", OffRunFiles.robotNumber));// Path to directory for IntelliJ code
            OffRunFiles.fc.writeFieldAsText(fos, OffRunFiles.autoOpMode.fc.RobotPoints, size);

            fos = new FileOutputStream(OffRunFiles.fileLocation + String.format("Robot%dAccessories.txt", OffRunFiles.robotNumber));// Path to directory for IntelliJ code
            OffRunFiles.writeExtrasToFile(fos);

            fos = new FileOutputStream(OffRunFiles.fileLocation + String.format("Robot%dPath.txt", OffRunFiles.robotNumber));// Path to directory for IntelliJ code
            OffRunFiles.writePath(fos, OffRunFiles.autoOpMode.lines, OffRunFiles.autoOpMode.lines.size());

            fos = new FileOutputStream(OffRunFiles.fileLocation + String.format("Robot%dPursuit.txt", OffRunFiles.robotNumber));// Path to directory for IntelliJ code
            OffRunFiles.fc.writeFieldAsText(fos, OffRunFiles.autoOpMode.fc.PursuitPoints, size);
            fos = new FileOutputStream(OffRunFiles.fileLocation + String.format("Robot%dNav.txt", OffRunFiles.robotNumber));// Path to directory for IntelliJ code
            OffRunFiles.fc.writeFieldAsText(fos, OffRunFiles.autoOpMode.fc.NavPoints, size);

            if (OffRunFiles.autoOpMode.writeRR) {
                fos = new FileOutputStream(fileLocation + "RedRing.txt");// Path to directory for IntelliJ code
                OffRunFiles.fc.writeFieldAsText(fos, OffRunFiles.autoOpMode.fc.RedRingPoints, size);
                fos = new FileOutputStream(fileLocation + "RingSet.txt");// Path to directory for IntelliJ code
                OffRunFiles.fc.writeRingType(fos);

            }
            if (OffRunFiles.autoOpMode.writeBR) {
                fos = new FileOutputStream(fileLocation + "BlueRing.txt");// Path to directory for IntelliJ code
                OffRunFiles.fc.writeFieldAsText(fos, OffRunFiles.autoOpMode.fc.BlueRingPoints, size);
                fos = new FileOutputStream(fileLocation + "RingSet.txt");// Path to directory for IntelliJ code
                OffRunFiles.fc.writeRingType(fos);
            }
            if (OffRunFiles.autoOpMode.writeRW1) {
                fos = new FileOutputStream(fileLocation + "RedWobble1.txt");// Path to directory for IntelliJ code
                OffRunFiles.fc.writeFieldAsText(fos, OffRunFiles.autoOpMode.fc.RedWobble1Points, size);
            }
            if (OffRunFiles.autoOpMode.writeRW2){
                fos = new FileOutputStream(fileLocation + "RedWobble2.txt");// Path to directory for IntelliJ code
                OffRunFiles.fc.writeFieldAsText(fos, OffRunFiles.autoOpMode.fc.RedWobble2Points, size);
            }
            if (OffRunFiles.autoOpMode.writeBW1 ) {
                fos = new FileOutputStream(fileLocation + "BlueWobble1.txt");// Path to directory for IntelliJ code
                OffRunFiles.fc.writeFieldAsText(fos, OffRunFiles.autoOpMode.fc.BlueWobble1Points, size);
            }
            if ( OffRunFiles.autoOpMode.writeBW2){
                fos = new FileOutputStream(fileLocation + "BlueWobble2.txt");// Path to directory for IntelliJ code
                OffRunFiles.fc.writeFieldAsText(fos, OffRunFiles.autoOpMode.fc.BlueWobble2Points, size);
            }

            OffRunFiles.telemetry.addData("Total Number of Time Steps", "%d", OffRunFiles.IMUCounter);
            OffRunFiles.telemetry.addData("Completed", "Robot: %d, side: %d, 1= BLUE, -1 = RED", h, OffRunFiles.autoOpMode.sideColor);
            OffRunFiles.telemetry.addLine("===================================");
            OffRunFiles.telemetry.addLine(" ");
            OffRunFiles.telemetry.update();

        }
    }

}