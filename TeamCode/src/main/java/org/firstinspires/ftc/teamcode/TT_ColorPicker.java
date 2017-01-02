package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by Niki Monsef on 12/10/2015.
 */


public class TT_ColorPicker {
    public enum Color {
        UNKNOWN,     // unknown color (other than blue or red)
        BLUE,        // blue color
        RED          // red color
    }

    ColorSensor _colorSensorL;
    ColorSensor _colorSensorR;

    //DeviceInterfaceModule cdim;
    //TouchSensor t;
    final static double COLOR_THRESHOLD = 4;
    double _currentRed  ;
    double _currentBlue ;

    double[] _runningRed  = new double[5] ;
    double[] _runningBlue = new double[5] ;

    TT_ColorPicker(ColorSensor colorSensorR, ColorSensor colorSensorL){
        _colorSensorR = colorSensorR ;
        _colorSensorL = colorSensorL ;
        reset();
    }

    public void reset() {
        for (int i = 0 ; i < 5 ; i++){
            _runningRed[i] = 0 ;
            _runningBlue[i]= 0 ;
        }
        _currentRed = 0;
        _currentBlue = 0;
    }

    public Color getColor(boolean is_left){ // get Color of Left beacon
        if (is_left) {
            insertNewSamples(_colorSensorL.red(), _colorSensorL.blue());
        } else {
            insertNewSamples(_colorSensorL.red(), _colorSensorL.blue());
        }
        calcFinal();
        if ( _currentBlue > ( _currentRed + COLOR_THRESHOLD) ){
            return  Color.BLUE; // 1 = Blue
        }
        else if ( _currentRed > ( _currentBlue + COLOR_THRESHOLD )){
            return Color.RED ; // 2 = Red
        }
        else { // use _colorSensorR to implies the left beacon
            return Color.UNKNOWN ; // not sure ( not enough difference )
        }
    }

    private void insertNewSamples(double red, double blue){
        for ( int i = 4 ; i >= 1 ; i-- ) {
            _runningRed[i]=_runningRed[i-1] ; // 0->1 ; 1->2 ; ... ; 5-> drop
            _runningBlue[i]=_runningBlue[i-1] ; // 0->1 ; 1->2 ; ... ; 5-> drop
        }
        _runningRed[0]  = red ;
        _runningBlue[0] = blue ;
    }

    private void calcFinal(){
        double sumOfRed  = 0 ;
        double sumOfBlue = 0 ;
        int count = 0;
        for ( count = 0 ; count < 5 ; count++ ){
            if ((_runningBlue[count]+_runningRed[count]==0))
                break;
            sumOfBlue += _runningBlue[count] ;
            sumOfRed  += _runningRed[count]  ;
        }
        if (count>0) {
            _currentRed = sumOfRed / count;
            _currentBlue = sumOfBlue / count;
        } else {
            _currentRed = 0;
            _currentBlue = 0;
        }
    }
}
