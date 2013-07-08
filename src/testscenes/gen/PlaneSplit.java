import java.util.*;
import java.math.*;
import java.util.regex.*;

public class PlaneSplit {

    static class Point {
        double x, y, z;
    }
    
    static class Vector {
        double x, y, z;
    }
    
    static void minus(Point a, Point b, Vector v) {
        v.x = a.x - b.x;
        v.y = a.y - b.y;
        v.z = a.z - b.z;
    }
    
    static void divide(Vector v, double d) {
        v.x /= d;
        v.y /= d;
        v.z /= d;
    }
    
    final static Pattern pointPattern = Pattern.compile("(\\d+),(\\d+),(\\d+)");
    
    static boolean readPoint(String str, Point p) {
        Matcher m = pointPattern.matcher(str);
        
        if (!m.matches())
            return false;
        
        p.x = Double.valueOf(m.group(1));
        p.y = Double.valueOf(m.group(2));
        p.z = Double.valueOf(m.group(3));
        return true;
    }

    public static void main(String[] args) {
        if (args.length < 3 || args.length > 4) {
            System.err.println("please enter the correct number of arguments");
            System.exit(2);
        }
        
        Point p1 = new Point(), p2 = new Point(), p3 = new Point();
        
        if (!readPoint(args[0], p1)) {
            System.err.format("Invalid point: %s\n", args[0]);
            System.exit(1);
        }
        if (!readPoint(args[2], p2)) {
            System.err.format("Invalid point: %s\n", args[2]);
            System.exit(1);
        }
        if (!readPoint(args[2], p3)) {
            System.err.format("Invalid point: %s\n", args[2]);
            System.exit(1);
        }
        
        int size = 100;
        if (args.length == 4)
            size = Integer.valueOf(args[3]);
        if (size <= 0) {
            System.err.format("Please enter a positive size, not %i\n", size);
            System.exit(3);
        }
        
        Vector dir1 = new Vector(), dir2 = new Vector();
        
        minus(p2, p1, dir1);
        minus(p3, p1, dir2);
        
        int count;
        {
            double count_x = Math.max(dir1.x / size, dir2.x / size);
            double count_y = Math.max(dir1.y / size, dir2.y / size);
            double count_z = Math.max(dir1.z / size, dir2.z / size);
            
            count = (int) Math.max(count_x, Math.max(count_y, count_z));
        }
        
        divide(dir1, count);
        divide(dir2, count);
        
        int x, y;
        int idx0, idx1, idx2, idx3;
        
        System.out.println("Shape \"trianglemesh\"");
        System.out.println("\t\"point P\" [");
        for (x = 0; x <= count; x++) {
            for (y = 0; y <= count; y++) {
                System.out.printf("\t\t%f %f %f\n",
                    p1.x + (x * dir1.x) + (y * dir2.x),
                    p1.y + (x * dir1.y) + (y * dir2.y),
                    p1.z + (x * dir1.z) + (y * dir2.z)
                );
            }
        }
        System.out.println("\t]");
        
        System.out.println("\t\"integer indices\" [");
        for (x = 0; x < count; x++) {
            for (y = 0; y < count; y++) {
                idx0 = x       * (count + 1) + y;
                idx1 = (x + 1) * (count + 1) + y;
                idx2 = (x + 1) * (count + 1) + (y + 1);
                idx3 = x       * (count + 1) + (y + 1);
                
                System.out.printf("\t\t%d %d %d %d %d %d\n",
                    idx0, idx1, idx2,
                    idx2, idx3, idx0
                );
            }
        }
        System.out.println("\t]");
    }

}
