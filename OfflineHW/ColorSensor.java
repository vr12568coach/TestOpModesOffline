package OfflineCode.OfflineHW;



/**
 * Created by Spiessbach on 8/26/2018.
 * Removed android reference in order to compile 10/4/2020
 */

public class ColorSensor {
    public int counter = 0;

    public int size = 100;
    public double timeStep = 0.0;//determined a fixed time step so that faster speeds will show shorter time to distance
    public float[] hueArray = new float[size];
    public static final int YELLOW      = 0xFFFFFF00;

    private void init(){
        for(int i=0; i<size; i++){
            hueArray[i] = (float)i / 2.0f;
        }
        hueArray[size-2] = 20.0f;
        hueArray[size-1] = 20.0f;

    }

    public static int red(int color) {
        return (color >> 16) & 0xFF;
    }

    /**
     * Return the green component of a color int. This is the same as saying
     * (color >> 8) & 0xFF
     */
    public static int green(int color) {
        return (color >> 8) & 0xFF;
    }

    /**
     * Return the blue component of a color int. This is the same as saying
     * color & 0xFF
     */

    public static int blue(int color) {
        return color & 0xFF;
    }


    public float[] generateHSV(){
       //HSV array[0] Hue needs to be between 15 and 30 to exit the sampling
        //HSV[1] Saturation needs to be > 0.55
        this.init();
        counter+=1;
        if(counter>=size){
            counter=0;
        }
        float colorList[] = {hueArray[counter], (float)  0.6, (float) (Math.random()*255)};
        return colorList;
    }
    public static void main(String []args){
        ColorSensor CS = new ColorSensor();
        float testHSV[] = {1F, 0F, 0F};
        double doubleHSV[] = {0,0,0};
        double ColorArray[] = {0,0,0};
        for(int i=0; i<30;i ++) {

            testHSV = CS.generateHSV();
            for (int j = 0; j < 3; j++) {
//                testHSV[j] = (float) (Math.random())*255;
                doubleHSV[j] = (double) Math.round(Math.random()*255);
            }
            ColorArray[0] = CS.red(YELLOW);
            ColorArray[1] = CS.blue(YELLOW);
            ColorArray[2] = CS.green(YELLOW);


            System.out.println(String.format("Fake HSV Output-- H: %.2f, S: %.2f, V: %.2f", doubleHSV[0], doubleHSV[1], doubleHSV[2]));
            System.out.println(String.format("Color Methods-- R: %.3f, B: %.3f, G: %.3f", ColorArray[0], ColorArray[1], ColorArray[2]));
            System.out.println(String.format("HSV Output-- H: %.2f, S: %.2f, V: %.2f",testHSV[0],testHSV[1],testHSV[2]));

        }
        double aNumber = 249.99;
        int bNumber = (int) aNumber;
        double cNumber = Math.round(aNumber);
        int dNumber = (int) Math.round(aNumber);
        System.out.println(String.format("aNumber = %.2f", aNumber));
        System.out.println(String.format("bNumber = %d", bNumber));
        System.out.println(String.format("cNUmber = %.2f", cNumber));
        System.out.println(String.format("dNUmber = %d", dNumber));


    }
}


