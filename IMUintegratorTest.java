package TestOpModesOffline;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.ArrayList;

public class IMUintegratorTest {
    double dt = 1e-3;
    double tend = 10.0;
    double dtMin = 0.5 * dt;
    double dtMAx = 2.0 * dt;
    double noiseAmplitude = 0.5;
    double motionTime = 2.0;
    double filtFreq1 = 20*2*Math.PI;
    double filtFreq2 = 5.0*2*Math.PI;

    Acceleration accel;
    Acceleration accelFiltered;
    Acceleration accelPrevious;

    Velocity vel;
    Velocity velPrevious;

    Position pos;
    Position posPrevious;

    Acceleration bias;

    Position posInit;

    ArrayList<Acceleration> accelOut = new ArrayList();
    ArrayList<Acceleration> accelFIltOut = new ArrayList();
    ArrayList<Velocity> velOut = new ArrayList();
    ArrayList<Position> posOut = new ArrayList();

    double[] time = new double[10000];

    //constructor
    public IMUintegratorTest() {
    //empty
    };
    public static void main(String []args){

        double randNum = (-1 + 2*Math.random());// random number -1 to 1 uniform distribution

    }
}
