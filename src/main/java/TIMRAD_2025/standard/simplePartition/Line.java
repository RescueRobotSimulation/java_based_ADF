package TIMRAD_2025.standard.simplePartition;

import java.util.ArrayList;
import java.util.List;


public class Line {
    public static final int MAX_M = 1000;
    public static final double TOLERANCE_ANGLE = Math.PI/8;
    public static final double TOLERANCE_C = 0.5E7;
    double m;
    double c;

    public Line(double m, double c) {
        this.m = m;
        this.c = c;
    }

    public Line(double x1, double y1, double x2, double y2) {
        if ( x1 == x2){
            m = Line.MAX_M;
        }else{
            m = ((double)y1-y2)/(x1-x2);
            if(m > Line.MAX_M){
                m = Line.MAX_M;
            }
        }
        c = y1 - m * x1;
    }

    public boolean isClose(Line other){
        if(hasSimilarM(other)){
            return Math.abs(c-other.c)<TOLERANCE_C;
        }
        return false;
    }

    public boolean hasSimilarM(Line other){
        double a = Math.atan(m);
        double b = Math.atan(other.m);
        return Math.abs(a-b)<TOLERANCE_ANGLE;
    }


    public boolean isOnLeft(int x, int y){
        return y - m * x - c > 0;
    }

    public static List<Line> removeSimilarLines(List<Line> lines){
        List<Line> newLines = new ArrayList<Line>();
        for(Line line : lines){
            boolean found = false;
            for(Line acceptedLine : newLines){
                if(acceptedLine.isClose(line)){
                    found = true;
                    break;
                }
            }
            if(!found){
                newLines.add(line);
            }
        }
        return newLines;
    }

    @Override
    public String toString(){
        return "Line: "+ m + ", " + c;
    }
}
