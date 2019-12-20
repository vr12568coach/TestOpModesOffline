package TestOpModesOffline;


import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.TempUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.ArrayList;

import Skystone_14999.Parameters.Constants;

/**
 * Created by Spiessbach on 8/26/2018.
 */

public  class BNO055IMU{
    public Constants params = new Constants();

//    final double ROBOT_INCH_TO_MOTOR_DEG = 360 / (3.977 * Math.PI);// 360 deg. / wheel circumference (Wheel diameter x pi)
//    final int DEGREES_TO_COUNTS = 1440 / 360; //Counts per 1 revolution
//    final double ROBOT_DEG_TO_WHEEL_INCH = 19.694 * Math.PI / 360;//Robot rotation circumference [(wheel base (diagonal)] * pi/360 deg
//    double driveDistance = 24;//Target distance inches for robot motion in any omni direction for drive motors
//    int maxCounts = (int) Math.round(driveDistance*ROBOT_INCH_TO_MOTOR_DEG*DEGREES_TO_COUNTS);
    public float fakeAngle=0;
    public float fakeRate=0;
    public int flCnt=0;
    public int frCnt=0;
    public int brCnt=0;
    public int blCnt=0;
    public int lsCnt=0;
    public double serPos=0;

    public boolean haveBlueFoundation = false;
    public boolean haveRedFoundation = false;
    public boolean haveSkyStone = false;
    public boolean stoneLocated = false;


    public double robotFRBLCount = 0;
    public double robotFLBRCount = 0;

    public double robotX=0;
    public double robotY=0;
    public double robotDist=0;
    public double fieldX=0;
    public double fieldY=0;
    public double fieldDist=0;
    //Define RedFoundation initial position
    public double redFoundX = 20;
    public double redFoundY = 51;
    public double redFoundTheta = 0;
    //Define BlueFoundation initial position
    public double blueFoundX = -10;
    public double blueFoundY = 51;
    public double blueFoundTheta = 0;
    //Define SkyStone initial position
    public double stoneX = -20;
    public double stoneY = -43;
    public double stoneTheta = 0;

    public Orientation IMUangles = new Orientation(AxesReference.INTRINSIC, AxesOrder.ZYX, org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES, 100, 100, 100, 1);
    public AngularVelocity IMURate = new AngularVelocity(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES, 0, 0, 0, 1);
    public double priorAngle=0;
    public double timeValue=0;
    public int size = 300;
    public double timeStep = 100;

    public double[] timeArray =new double[size];
    public int[] flArray=new int[size];
    public int[] frArray=new int[size];
    public int[] brArray=new int[size];
    public int[] blArray=new int[size];
    public int[] lsArray=new int[size];


    public double[] FLBRArray=new double[size];
    public double[] FRBLArray=new double[size];

    public double[] robotXArray=new double[size];
    public double[] robotYArray=new double[size];
    public double[] robotDistArray=new double[size];
    public double[] robotAngleArray=new double[size];

    public double[] fieldXArray=new double[size];
    public double[] fieldYArray=new double[size];
    public double[] fieldDistArray=new double[size];

    public ArrayList<FieldLocation> RedFoundationPoints =new ArrayList();
    public ArrayList<FieldLocation> BlueFoundationPoints =new ArrayList();
    public ArrayList<FieldLocation> SkyStonePoints =new ArrayList();

    public int counter = 0;
    public double factor = 1;

    public static final I2cAddr I2CADDR_DEFAULT     = I2cAddr.create7bit(0x28);


    public Orientation getAngularOrientation(AxesReference reference, AxesOrder order, org.firstinspires.ftc.robotcore.external.navigation.AngleUnit angleUnit){
        int deltaFL = flCnt - flArray[counter-1];
        int deltaFR = frCnt - frArray[counter-1];
        int deltaBR = brCnt - brArray[counter-1];
        int deltaBL = blCnt - blArray[counter-1];
        int deltaLS = lsCnt - lsArray[counter-1];
        int deltaSum = (deltaFL  + deltaFR  + deltaBR  + deltaBL)/4;
        fakeAngle += (float) deltaSum / (params.DEGREES_TO_COUNTS * params.ROBOT_INCH_TO_MOTOR_DEG *
                params.ROBOT_DEG_TO_WHEEL_INCH * params.adjustedRotate);
        // no change to delta sum to put angle into field coordinates, + sum = CCW --> + IMU angle
        float fake1 = (float) (fakeAngle - priorAngle);
        float fake2 = (float) timeStep;
        fakeRate = fake1/fake2;

        robotFLBRCount = 0.707* Math.signum(deltaBR)*(Math.abs(deltaBR-deltaSum) + Math.abs(deltaFL-deltaSum))/2;//
        robotFRBLCount = 0.707* Math.signum(deltaFR)*(Math.abs(deltaFR-deltaSum) + Math.abs(deltaBL-deltaSum))/2;//
// removed +=

        if(Math.signum(deltaFL)== Math.signum(deltaFR) && Math.signum(deltaFL)!= Math.signum(deltaBL)){
            factor = 1/params.adjustedRight;
        }
        else if(deltaSum != 0)
            factor = 1/params.adjustedRotate;
        else {

            factor = 1;
        }
//adding +=
//Coordinate transformation to take motor drive coordinates to robot body reference frame - fixed 45 deg rotation
        double robotXInc = factor* ((robotFRBLCount*0.707) + (robotFLBRCount*0.707))/
                (params.DEGREES_TO_COUNTS*params.ROBOT_INCH_TO_MOTOR_DEG);
        double robotYInc = -factor* ((-robotFRBLCount*0.707) + (robotFLBRCount*0.707))/
                (params.DEGREES_TO_COUNTS*params.ROBOT_INCH_TO_MOTOR_DEG);//flipped sign based on sign correction in CoachDriveFunction for R/L
        robotX += robotXInc;
        robotY += robotYInc;
        robotDist = Math.sqrt(robotX*robotX + robotY*robotY);

//Coordinate transformation to take robot body coordinates to field reference frame - depends on IMU angle
//Angle reference from field to robot is negative angle in CW = + robot frame, uses + = CCW {IMU frame & field frame}
        double fieldXInc = (robotXInc*Math.cos(Math.toRadians(fakeAngle))) -
                (robotYInc*Math.sin(Math.toRadians(fakeAngle)));//flipped sign on Ry term
        double fieldYInc = (robotXInc*Math.sin(Math.toRadians(fakeAngle))) +
                (robotYInc*Math.cos(Math.toRadians(fakeAngle)));//flipped sign on Rx term
        fieldX += fieldXInc;
        fieldY += fieldYInc;
        fieldDist = Math.sqrt(fieldX*fieldX + fieldY*fieldY);

        if(haveRedFoundation){
            redFoundX += fieldXInc;
            redFoundY += fieldYInc;
            redFoundTheta += (fakeAngle - priorAngle);
        }
        if(haveBlueFoundation){
//            blueFoundX += fieldXInc;
//            blueFoundY += fieldYInc;
//            blueFoundTheta += (fakeAngle - priorAngle);
            double foundationOffset = 9 - (34.5 / 2);

            blueFoundX = fieldX + foundationOffset*Math.cos(Math.toRadians(fakeAngle));
            blueFoundY = fieldY + foundationOffset*Math.sin(Math.toRadians(fakeAngle));
            blueFoundTheta = fakeAngle + 90;
        }
        if(haveSkyStone){
            double stoneOffset = 10;//Distance from robot rotation center in inches, assume along x

            if(!stoneLocated){
                if(fieldX > 0){
                    stoneOffset = -9;//Distance from robot rotation center in inches, assume along x
                }
                stoneX = fieldX + stoneOffset;
                stoneY = fieldY;
                stoneTheta += (fakeAngle - priorAngle);
                stoneLocated = true;
            }
            else {

                stoneX = fieldX + stoneOffset*Math.cos(Math.toRadians(fakeAngle));
                stoneY = fieldY + stoneOffset*Math.sin(Math.toRadians(fakeAngle));
                stoneTheta = fakeAngle;


            }
        }


        IMUangles.firstAngle = fakeAngle;//Note IMU returns angle in opposite orientation than robot coordinate system
        //IMU + angle = CCW ; robot + angle = CW
        IMUangles.secondAngle = 0;
        IMUangles.thirdAngle = 0;

        timeValue+=timeStep;
        timeArray[counter] = timeValue/1000;
        flArray[counter] = flCnt;
        frArray[counter] = frCnt;
        brArray[counter] = brCnt;
        blArray[counter] = blCnt;
        lsArray[counter] = lsCnt;
        FLBRArray[counter] = robotFLBRCount;
        FRBLArray[counter] = robotFRBLCount;
        robotXArray[counter] = robotX;
        robotYArray[counter] = robotY;
        robotDistArray[counter] = robotDist;
        robotAngleArray[counter] =  IMUangles.firstAngle;//use IMU + = CCW convention for visualization
        fieldXArray[counter] = fieldX;
        fieldYArray[counter] = fieldY;
        fieldDistArray[counter] = fieldDist;
        RedFoundationPoints.add(new FieldLocation(redFoundX,redFoundY,redFoundTheta));
        BlueFoundationPoints.add(new FieldLocation(blueFoundX,blueFoundY,blueFoundTheta));
        SkyStonePoints.add(new FieldLocation(stoneX,stoneY,stoneTheta));

        counter+=1;



        return IMUangles;
    }

    public AngularVelocity getAngularVelocity(){
        IMURate.xRotationRate = fakeRate;
        IMURate.yRotationRate = 0;
        IMURate.zRotationRate = 0;
        return IMURate;
    }
    public void updateCounter(int counts){
        counter = counts;
    }


    /**
     * ITEMS BELOW ARE ALL FOR IMPLMENTING THE IMU INTERFACE AND ARE NOT UPDATED IN THSI CLASS
     * @param parameters
     * @return
     */
    public boolean initialize(BNO055IMU.Parameters parameters){
        params.defineParameters();
        return true;
    }
    /**
     * Instances of Parameters contain data indicating how a BNO055 absolute orientation
     * sensor is to be initialized.
     *
     */
//    public class Parameters implements Cloneable
    public static class Parameters{
        /** the address at which the sensor resides on the I2C bus.  */
        public I2cAddr          i2cAddr             = I2CADDR_DEFAULT;

        /** the mode we wish to use the sensor in */
        public SensorMode mode                = SensorMode.IMU;

        /** whether to use the external or internal 32.768khz crystal. External crystal
         * use is recommended by the BNO055 specification. */
        public boolean          useExternalCrystal  = true;

        /** units in which temperature are measured. See Section 3.6.1 (p31) of the BNO055 specification */
        public TempUnit temperatureUnit     = TempUnit.CELSIUS;
        /** units in which angles and angular rates are measured. See Section 3.6.1 (p31) of the BNO055 specification */
        public AngleUnit angleUnit           = AngleUnit.RADIANS;
        /** units in which accelerations are measured. See Section 3.6.1 (p31) of the BNO055 specification */
        public AccelUnit accelUnit           = AccelUnit.METERS_PERSEC_PERSEC;
        /** directional convention for measureing pitch angles. See Section 3.6.1 (p31) of the BNO055 specification */
        public PitchMode pitchMode           = PitchMode.ANDROID;    // Section 3.6.2

        /** accelerometer range. See Section 3.5.2 (p27) and Table 3-4 (p21) of the BNO055 specification */
        public AccelRange accelRange          = AccelRange.G4;
        /** accelerometer bandwidth. See Section 3.5.2 (p27) and Table 3-4 (p21) of the BNO055 specification */
        public AccelBandwidth accelBandwidth      = AccelBandwidth.HZ62_5;
        /** accelerometer power mode. See Section 3.5.2 (p27) and Section 4.2.2 (p77) of the BNO055 specification */
        public AccelPowerMode accelPowerMode      = AccelPowerMode.NORMAL;

        /** gyroscope range. See Section 3.5.2 (p27) and Table 3-4 (p21) of the BNO055 specification */
        public GyroRange gyroRange           = GyroRange.DPS2000;
        /** gyroscope bandwidth. See Section 3.5.2 (p27) and Table 3-4 (p21) of the BNO055 specification */
        public GyroBandwidth gyroBandwidth       = GyroBandwidth.HZ32;
        /** gyroscope power mode. See Section 3.5.2 (p27) and Section 4.4.4 (p78) of the BNO055 specification */
        public GyroPowerMode gyroPowerMode       = GyroPowerMode.NORMAL;

        /** magnetometer data rate. See Section 3.5.3 (p27) and Section 4.4.3 (p77) of the BNO055 specification */
        public MagRate magRate             = MagRate.HZ10;
        /** magnetometer op mode. See Section 3.5.3 (p27) and Section 4.4.3 (p77) of the BNO055 specification */
        public MagOpMode magOpMode           = MagOpMode.REGULAR;
        /** magnetometer power mode. See Section 3.5.3 (p27) and Section 4.4.3 (p77) of the BNO055 specification */
        public MagPowerMode magPowerMode        = MagPowerMode.NORMAL;

        /** Calibration data with which the BNO055 should be initialized. If calibrationData is non-null,
         * it is used. Otherwise, if calibrationDataFile is non-null, it is used. Otherwise, only the default
         * automatic calibration of the IMU is used*/
        public CalibrationData calibrationData     = null;
        public String           calibrationDataFile = null;


        public AccelerationIntegrator accelerationIntegrationAlgorithm = null;

        /** debugging aid: enable logging for this device? */
        public boolean          loggingEnabled      = false;
        /** debugging aid: the logging tag to use when logging */
        public String           loggingTag          = "AdaFruitIMU";

//        public BNO055IMU.Parameters clone()
//        {
//            try {
//                BNO055IMU.Parameters result = (BNO055IMU.Parameters)super.clone();
//                result.calibrationData = result.calibrationData==null ? null : result.calibrationData.clone();
//                return result;
//            }
//            catch (CloneNotSupportedException e)
//            {
//                throw new RuntimeException("internal error: Parameters can't be cloned");
//            }
//        }
    }

    public class CalibrationData implements Cloneable
    {
        public short dxAccel, dyAccel, dzAccel; // units are milli-g's
        public short dxMag,   dyMag,   dzMag;   // units are micro telsa
        public short dxGyro,  dyGyro,  dzGyro;  // units are degrees / second
        public short radiusAccel, radiusMag;    // units are unknown

//        public String serialize() {
//            return SimpleGson.getInstance().toJson(this);
//        }
//        public static CalibrationData deserialize(String data) {
//            return SimpleGson.getInstance().fromJson(data, CalibrationData.class);
//        }

        public CalibrationData clone()
        {
            try {
                CalibrationData result = (CalibrationData)super.clone();
                return result;
            }
            catch (CloneNotSupportedException e)
            {
                throw new RuntimeException("internal error: CalibrationData can't be cloned");
            }
        }
    }

    public AccelerationIntegrator accelerationIntegrationAlgorithm = null;

    public interface AccelerationIntegrator
    {
        /**
         * (Re)initializes the algorithm with a starting position and velocity. Any timestamps that
         * are present in these data are not to be considered as significant. The initial acceleration
         * should be taken as undefined; you should set it to null when this method is called.
         * @param parameters        configuration parameters for the IMU
         * @param initialPosition   If non-null, the current sensor position is set to this value. If
         *                          null, the current sensor position is unchanged.
         * @param initialVelocity   If non-null, the current sensor velocity is set to this value. If
         *                          null, the current sensor velocity is unchanged.
         *
         * @see #update(Acceleration)
         */
        void initialize(Parameters parameters, Position initialPosition, Velocity initialVelocity);

        /**
         * Returns the current position as calculated by the algorithm
         * @return  the current position
         */
        Position getPosition();

        /**
         * Returns the current velocity as calculated by the algorithm
         * @return  the current velocity
         */
        Velocity getVelocity();

        /**
         * Returns the current acceleration as understood by the algorithm. This is typically
         * just the value provided in the most recent call to {@link #update(Acceleration)}, if any.
         * @return  the current acceleration, or null if the current position is undefined
         */
        Acceleration getAcceleration();

        /**
         * Step the algorithm as a result of the stimulus of new acceleration data.
         * @param linearAcceleration  the acceleration as just reported by the IMU
         */
        void update(Acceleration linearAcceleration);
    }

    public enum AngleUnit { DEGREES(0), RADIANS(1); public final byte bVal; AngleUnit(int i) { bVal =(byte)i; }
        public org.firstinspires.ftc.robotcore.external.navigation.AngleUnit toAngleUnit()
        {
            if (this==DEGREES)
                return org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
            else
                return org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
        }
        public static AngleUnit fromAngleUnit(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit angleUnit)
        {
            if (angleUnit==org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES)
                return DEGREES;
            else
                return RADIANS;
        }
    }
    public enum AccelUnit { METERS_PERSEC_PERSEC(0), MILLI_EARTH_GRAVITY(1); public final byte bVal; AccelUnit(int i) { bVal =(byte)i; }}
    public enum PitchMode { WINDOWS(0), ANDROID(1);                          public final byte bVal; PitchMode(int i) { bVal =(byte)i; }}

    public enum GyroRange      { DPS2000(0), DPS1000(1), DPS500(2), DPS250(3), DPS125(4);                               public final byte bVal; GyroRange(int i)      { bVal =(byte)(i<<0);}}
    public enum GyroBandwidth  { HZ523(0), HZ230(1), HZ116(2), HZ47(3), HZ23(4), HZ12(5), HZ64(6), HZ32(7);             public final byte bVal; GyroBandwidth(int i)  { bVal =(byte)(i<<3);}}
    public enum GyroPowerMode  { NORMAL(0), FAST(1), DEEP(2), SUSPEND(3), ADVANCED(4) ;                                 public final byte bVal; GyroPowerMode(int i)  { bVal =(byte)(i<<0);}}
    public enum AccelRange     { G2(0), G4(1), G8(2), G16(3);                                                           public final byte bVal; AccelRange(int i)     { bVal =(byte)(i<<0);}}
    public enum AccelBandwidth { HZ7_81(0), HZ15_63(1), HZ31_25(2), HZ62_5(3), HZ125(4), HZ250(5), HZ500(6), HZ1000(7); public final byte bVal; AccelBandwidth(int i) { bVal =(byte)(i<<2);}}
    public enum AccelPowerMode { NORMAL(0), SUSPEND(1), LOW1(2), STANDBY(3), LOW2(4), DEEP(5);                          public final byte bVal; AccelPowerMode(int i) { bVal =(byte)(i<<5);}}

    public enum MagRate        { HZ2(0), HZ6(1), HZ8(2), HZ10(3), HZ15(4), HZ20(5), HZ25(6), HZ30(7);                   public final byte bVal; MagRate(int i)        { bVal =(byte)(i<<0);}}
    public enum MagOpMode      { LOW(0), REGULAR(1), ENHANCED(2), HIGH(3);                                              public final byte bVal; MagOpMode(int i)      { bVal =(byte)(i<<3);}}
    public  enum MagPowerMode   { NORMAL(0), SLEEP(1), SUSPEND(2), FORCE(3);                                             public final byte bVal; MagPowerMode(int i)   { bVal =(byte)(i<<5);}}
    public enum SensorMode
    {
        CONFIG(0X00),       ACCONLY(0X01),          MAGONLY(0X02),
        GYRONLY(0X03),      ACCMAG(0X04),           ACCGYRO(0X05),
        MAGGYRO(0X06),      AMG(0X07),              IMU(0X08),
        COMPASS(0X09),      M4G(0X0A),              NDOF_FMC_OFF(0X0B),
        NDOF(0X0C),
        DISABLED(-1);   // DISABLED isn't an actual IMU mode
        //------------------------------------------------------------------------------------------
        public final byte bVal;
        SensorMode(int i) { this.bVal = (byte) i; }}

    public enum TempUnit { CELSIUS(0), FARENHEIT(1); public final byte bVal; TempUnit(int i)  { bVal =(byte)i; }
        public org.firstinspires.ftc.robotcore.external.navigation.TempUnit toTempUnit()
        {
            if (this==CELSIUS)
                return org.firstinspires.ftc.robotcore.external.navigation.TempUnit.CELSIUS;
            else
                return org.firstinspires.ftc.robotcore.external.navigation.TempUnit.FARENHEIT;
        }
        public static TempUnit fromTempUnit(org.firstinspires.ftc.robotcore.external.navigation.TempUnit tempUnit)
        {
            if (tempUnit==org.firstinspires.ftc.robotcore.external.navigation.TempUnit.CELSIUS)
                return CELSIUS;
            else if (tempUnit==org.firstinspires.ftc.robotcore.external.navigation.TempUnit.FARENHEIT)
                return FARENHEIT;
            else
                throw new UnsupportedOperationException("TempUnit." + tempUnit + " is not supported by BNO055IMU");
        }
    }
}
