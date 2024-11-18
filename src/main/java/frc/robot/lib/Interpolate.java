package frc.robot.lib;

public class Interpolate {
    public static double getY(double[] _x, double[] _y, double _xInput){
        double rtn = 0;
        int index = 0;

        if(_xInput >= _x[_x.length-1]){
            return _y[_x.length - 1];
        }else if(_xInput <= _x[0]){
            return _y[_x.length - 1];
        }

        for(int i = 0; i< _x.length; i++){
            if(_xInput <= _x[i]){
                index = i-1;
                break;
            }

        }
        double xPercent = (_xInput - _x[index]) / (_x[index + 1] - _x[index]);
        rtn = (_y[index + 1] - _y[index]) * xPercent + _y[index];
        return rtn;
    }
}
