package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.stream.IntStream;

import com.studica.frc.Lidar;

import org.opencv.core.Mat;

import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.lidar;
// import frc.robot.commands.driveCommands.StandardDriveBase.Pose2d;
public class Lidar360 extends SubsystemBase {

    // Lidar Library
    private static Lidar lidar;
    // Lidar Scan Data Storage Class
    private static Lidar.ScanData scanData;

    private static double prevHeading = 0;
    private long stateSwitchTime = 0;               // Timestamp in ms
    private static final long GRACE_PERIOD_MS = 1500; // Grace period duration in milliseconds

    private double maxXDist, maxYDist;// = 0;//0.6;//0.5;
    private double CGtoLidar = 0.0;
    private double robotWidth, reactivityBigAngle, reactivitySmallAngle;


    private int startAngle = Constants.lidar.startAngle;
    private int endAngle = Constants.lidar.endAngle;


    MedianFilter filterLeft = new MedianFilter(15);
    MedianFilter filterRight = new MedianFilter(15);


    private enum State {
        AVOIDING,
        GOAL_SEEKING
    }
    
    private State currentState = State.GOAL_SEEKING;

    public Lidar360() {

        /**
         * Top USB 2.0 port of VMX = kUSB1 Bottom USB 2.0 port of VMX = kUSB2
         */
        lidar = new Lidar(Constants.lidar.lidarPort);


        // Configure filters
        lidar.clusterConfig(50.0f, 5);
        // lidar.kalmanConfig(1e-5f, 1e-1f, 1.0f);
        // lidar.movingAverageConfig(5);
        // lidar.medianConfig(5);
        // lidar.jitterConfig(50.0f);

        // Enable Filter
        lidar.enableFilter(Lidar.Filter.kCLUSTER, true);
        // lidar.enableFilter(Lidar.Filter.kMEDIAN, true);
        // lidar.enableFilter(Lidar.Filter.kMOVINGAVERAGE, true);
        // lidar.enableFilter(Lidar.Filter.kJITTER, true);

        //! Drive Settings
        if (Constants.CURRENTROBOT == Constants.ROBOTTYPE.FourWheel) {
            this.maxXDist = 0.55;
            this.maxYDist = 1.5;
            this.CGtoLidar = 0.13;
            this.robotWidth = 0.4;
            this.reactivityBigAngle = 0.6;
            this.reactivitySmallAngle = 1;
        }
        else if (Constants.CURRENTROBOT == Constants.ROBOTTYPE.TwoWheel) {
            this.maxXDist = 0.6;
            this.maxYDist = 1.5;
            this.CGtoLidar = 0.0;
            this.robotWidth = 0.4;
            this.reactivityBigAngle = 0.5;
            this.reactivitySmallAngle = 1;
        }
        else if (Constants.CURRENTROBOT == Constants.ROBOTTYPE.Mecanum) {
            this.maxXDist = 0.6;
            this.maxYDist = 1.5;
            this.CGtoLidar = 0.0;
            this.robotWidth = 0.4;
            this.reactivityBigAngle = 0.5;
            this.reactivitySmallAngle = 1;
        }
        else if (Constants.CURRENTROBOT == Constants.ROBOTTYPE.SixWheel) {
            this.maxXDist = 0.6;
            this.maxYDist = 1.5;
            this.CGtoLidar = 0.0;
            this.robotWidth = 0.4;
            this.reactivityBigAngle = 0.6;
            this.reactivitySmallAngle = 1;
        }
        else if (Constants.CURRENTROBOT == Constants.ROBOTTYPE.XDrive) {
            this.maxXDist = 0.5;
            this.maxYDist = 1.5;
            this.CGtoLidar = 0.11;
            this.robotWidth = 0.4;
            this.reactivityBigAngle = 0.6;
            this.reactivitySmallAngle = 1;
        }
        else if (Constants.CURRENTROBOT == Constants.ROBOTTYPE.KiwiDrive) {
            this.maxXDist = 0.4;
            this.maxYDist = 1.5;
            this.CGtoLidar = 0.19;
            this.robotWidth = 0.4;
            this.reactivityBigAngle = 0.5;
            this.reactivitySmallAngle = 1;
        }

    }

    public void startLidar() {
        lidar.start();
    }

    public void stopLidar() {
        lidar.stop();
    }

    public double getLidarDistance(int angle) {
        scanData = lidar.getData();
        return scanData.distance[angle]/1000.0;
    }

    public double getLidarAngle(int angle) {
        scanData = lidar.getData();
        return scanData.angle[angle];
    }

    //* Used only for debugging / configuring objectpickup checker
    private void lidarReadings(int startAngle, int endAngle) {
        int index = 0;
        double[] dist = new double[360 - startAngle + endAngle + 1];
        for (int i = startAngle; i != (endAngle + 1) % 360; i = (i + 1) % 360) {
            double distance = scanData.distance[i] / 1000.0;
            dist[index] = distance;
            index++;
        }

        System.out.print("[");
        for (int i = 0; i < dist.length; i++) {
            System.out.print(dist[i]);
            if (i < dist.length - 1) {
                System.out.print(",");
            }
        }
        System.out.println("]");
    }


    //! =============================== Commands for Obstacle Avoid ===============================

    public void resetNavigationState(){
        currentState = State.GOAL_SEEKING;
    }

    private int countZeros(double[] array) {
        int count = 0;
        for (double num : array) {
            if (num == 0) {
                count += 1;
            }
        }
        return count;
    }

    public double[] lidarScan() { 
        boolean debug = false;

        scanData = lidar.getData();

        // double[] dist = new double[360 - startAngle + endAngle + 1]; // Array size should match the number of angles processed 181
        // double[] dist = new double[endAngle-startAngle+1];

        double[] dist;
        if (startAngle < endAngle) {
            dist = new double[endAngle-startAngle+1];
        }
        else {
            dist = new double[360 - startAngle + endAngle + 1];
        }
        
        int index = 0;
        // double maxXDist = 0.5;
        double minXDist = 0.00;
        
        for (int i = startAngle; i != (endAngle + 1) % 360; i = (i + 1) % 360) {
        // for (int i = startAngle; i <= endAngle; i++) {
            double distance = scanData.distance[i] / 1000.0;
            // double distance = filtereddist[i] / 1000.0;

            double angle_rad = Math.toRadians(i-270);
            // double yDist = Math.abs(Math.sin(angle_rad)) * distance;
            double xDist = Math.abs(Math.cos(angle_rad)) * distance;

            if (distance <= maxXDist && distance >= minXDist && xDist < maxXDist-0.1) {
                dist[index] = distance;
            } else {
                dist[index] = 0;
            }
            index++;
        }

        if (debug) {
            System.out.print("[");
            for (int i = 0; i < dist.length; i++) {
                System.out.print(dist[i]);
                if (i < dist.length - 1) {
                    System.out.print(",");
                }
            }
            System.out.println("]");
        }
        return dist;
    }

    private List<Integer> findbestgap(List<List<Integer>> listOfLists, int target) {
        if (listOfLists == null || listOfLists.isEmpty()) {
            return new ArrayList<>();
        }

        List<Integer> closestList = null;
        int minDifference = Integer.MAX_VALUE;

        for (List<Integer> currentList : listOfLists) {
            if (currentList == null || currentList.isEmpty()) {
                continue;
            }

            for (int num : currentList) {
                int currentDifference = Math.abs(num - target);

                if (currentDifference < minDifference) {
                    minDifference = currentDifference;
                    closestList = currentList;
                }
            }
        }

        return closestList;
    }


    private List<List<Integer>> convertLidarAngletoRobotAngle(List<List<Integer>> freeSegments) {
        // double CGtoLidar = 0.13;//0.2;//0.22;
        // double CGtoLidar = 0.0;

        // double maxXDist = 0.5;
        if (freeSegments == null || freeSegments.isEmpty()) {
            return null;
        }

        List<List<Integer>> result = new ArrayList<>();

        for (List<Integer> currentList : freeSegments) {
            if (currentList == null || currentList.isEmpty()) {
                continue;
            }

            List<Integer> convertedList = new ArrayList<>();

            for (int num : currentList) {

                // double thetaLidar = Math.toRadians(num - endAngle);
                double thetaLidar = Math.toRadians(num - Math.abs(140)/2);
                double thetaCoG = Math.atan((Math.tan(thetaLidar) * maxXDist) / (maxXDist + CGtoLidar));
                // convertedList.add((int) Math.toDegrees(thetaCoG)); //!negativeforupright

                if (Constants.lidar.lidarUpRight) {
                    convertedList.add((int) Math.toDegrees(-thetaCoG)); //!negativeforupright
                }
                else {
                    convertedList.add((int) Math.toDegrees(thetaCoG)); //!negativeforupright
                }

            }


            result.add(convertedList);

        }
        return result;
    }


    private double findMedian(List<Integer> list) {
        if (list == null || list.isEmpty()) {
            throw new IllegalArgumentException("Input list cannot be null or empty.");
        }

        List<Integer> sortedList = new ArrayList<>(list); // Create a copy to avoid modifying the original
        Collections.sort(sortedList); // Sort the copied list

        int size = sortedList.size();

        if (size % 2 == 0) {
            // Even number of elements
            int mid1 = sortedList.get(size / 2 - 1);
            int mid2 = sortedList.get(size / 2);
            return (double) (mid1 + mid2) / 2.0;
        } else {
            // Odd number of elements
            return sortedList.get(size / 2);
        }
    }

    /**
     * <h2> Command used by ObsAvoid to find the angle the robot should rotate to avoid obstacles and move to target location </h2>
     * Gets lidar reading > filter out angles that are not free > convert free segements to robot angle >
     * get best segement to move > returns either target angle or median angle 
     * 
     * @param target
     * @return double[] - [angle to turn, status code]
     */
    public double[] findFreeSpace1(double target) { //!Target refers to how much to turn from robot perspective
        boolean debug = false;

        double[] sendback = new double[2];
       
        return sendback;
    }


//! ===========================================================================================================

    public boolean frontClear(double dist){
        scanData = lidar.getData();

        // double[] dist = new double[360 - startAngle + endAngle + 1]; // Array size should match the number of angles processed 181
        int count = 0;
        double minXDist = 0.00;
        double maxXDist = dist;
        double distLidartoWheel = 0.4;
        // double distLidartoWheel = 0.26;
    
        for (int i = startAngle; i != (endAngle + 1) % 360; i = (i + 1) % 360) {
            double distance = scanData.distance[i] / 1000.0;
            double angle_rad = Math.toRadians(i);
            double yDist = Math.abs(Math.sin(angle_rad)) * distance;
            double xDist = Math.abs(Math.cos(angle_rad)) * distance;

            if (xDist >= minXDist && xDist <= maxXDist) {
                if (yDist < distLidartoWheel) {
                    count += 1;
                }
            }
        }

        // System.out.println("Count: " + count);
        if (count <= 2) {
            return true;
        }
        else {
            return false;
        }
    }

    public double[] offsetLidarValues(int lidarFrontAngle, double yOffset) {

        float[] lidarVals = lidar.getData().distance;
        double[] offsetValues = new double[lidarVals.length];
        
        int posTh, negTh;
        double posY, negY, sAoI, eAoI;
        if (startAngle > endAngle && lidarFrontAngle < endAngle) {
            posTh = lidarFrontAngle + (360-startAngle);
        } else {
            posTh = lidarFrontAngle - startAngle;
        }
        if ((startAngle > endAngle) && (lidarFrontAngle > endAngle)) {
            negTh = endAngle + (360-lidarFrontAngle);
        } else {
            negTh = endAngle - lidarFrontAngle;
        }

        posY = maxYDist + yOffset;
        negY = maxYDist - yOffset;

        sAoI = Math.floor(Math.toDegrees(Math.atan(posY/(maxYDist/Math.tan(Math.toRadians(posTh))))));
        eAoI = Math.ceil(Math.toDegrees(Math.atan(negY/(maxYDist/Math.tan(Math.toRadians(negTh))))));

        if (lidarFrontAngle - sAoI < 0) {
            sAoI = 360 + (lidarFrontAngle - sAoI);
        } else {
            sAoI = lidarFrontAngle - sAoI;
        }

        eAoI = (lidarFrontAngle + eAoI) % 360;

        //System.out.println("START END ANGLES: " + sAoI + " " + eAoI);

        int posThetaLimit = Math.round((float) Math.toDegrees(Math.atan(posY/maxXDist)));
        int negThetaLimit = Math.round((float) Math.toDegrees(Math.atan(negY/maxXDist)));

        //System.out.print("ANGLE LIMITS: " + posThetaLimit + " " + negThetaLimit);

        for (int i = (int) sAoI; i != ((int) eAoI + 1) % 360; i = (i + 1) % 360) {

            double distance = lidarVals[i] / 1000.0;
            double angle_rad;
            boolean posDir;
            
            if (sAoI > eAoI) {
                if (lidarFrontAngle >= 0 && lidarFrontAngle < eAoI) {
                    if (i >= sAoI && i <= 359) {
                        angle_rad = Math.toRadians(lidarFrontAngle + 360 - i);
                        posDir = true;
                    } else if (i >= 0 && i <= lidarFrontAngle) {
                        angle_rad = Math.toRadians(lidarFrontAngle - i);
                        posDir = true;
                    } else {
                        angle_rad = Math.toRadians(i - lidarFrontAngle);
                        posDir = false;
                    }
                } else {
                    if (i >= sAoI && i <= lidarFrontAngle) {
                        angle_rad = Math.toRadians(lidarFrontAngle - i);
                        posDir = true;
                    } else if (i > lidarFrontAngle && i <= 359) {
                        angle_rad = Math.toRadians(i - lidarFrontAngle);
                        posDir = false;
                    } else {
                        angle_rad = Math.toRadians(360-lidarFrontAngle + i);
                        posDir = false;
                    }
                }
            } else {
                if (i < lidarFrontAngle) {
                    angle_rad = Math.toRadians(lidarFrontAngle - i);
                    posDir = true;
                } else {
                    angle_rad = Math.toRadians(i - lidarFrontAngle);
                    posDir = false;
                }
            }

            double yDist = Math.abs(Math.sin(angle_rad)) * distance;
            double xDist = Math.abs(Math.cos(angle_rad)) * distance;
            if (!posDir) yDist *= -1;
            yDist -= yOffset;

            //System.out.println(i + " Dist: " + distance + " Deg: " + Math.toDegrees(angle_rad) + " Dir: " + posDir + "X: " + xDist + " Y: " + yDist);

            boolean validDist = true;
            if (distance == -0.001) {
                validDist = false;
                if (posDir) {
                    if (Math.toDegrees(angle_rad) >= posThetaLimit) {
                        yDist = maxYDist;
                        xDist = (maxYDist + yOffset)/Math.tan(angle_rad);
                    } else {
                        xDist = maxXDist;
                        yDist = Math.abs(xDist * Math.tan(angle_rad) - yOffset);
                    }
                } else {
                    if (Math.toDegrees(angle_rad) >= negThetaLimit) {
                        yDist = -maxYDist;
                        xDist = (maxYDist - yOffset)/Math.tan(angle_rad);
                    } else {
                        xDist = maxXDist;
                        yDist = -Math.abs(xDist * Math.tan(angle_rad) + yOffset);
                    }
                }
            } 

            int newTheta = lidarFrontAngle - Math.round((float) Math.toDegrees(Math.atan(yDist/xDist)));
            if (newTheta < 0) newTheta += 360;
            else if (newTheta > 359) newTheta %= 360;
            if (validDist) offsetValues[newTheta] = Math.sqrt(xDist*xDist + yDist*yDist); 
            else offsetValues[newTheta] = 0;     

            // System.out.println(i + " Deg: " + Math.toDegrees(angle_rad) + " newTheta: " + newTheta + " Distance: " + offsetValues[newTheta]);
        }

        // for (int i = startAngle; i != (endAngle + 1) % 360; i = (i + 1) % 360) {
        //     if (offsetValues[i] == 0.0 && offsetValues[correctAngle(i+1)] == 0.0 && offsetValues[correctAngle(i+2)] != 0.0 && offsetValues[correctAngle(i-1)] != 0.0) {
        //         offsetValues[i] = (2*offsetValues[correctAngle(i-1)]+offsetValues[correctAngle(i+2)])/3;
        //         offsetValues[correctAngle(i+1)] = (2*offsetValues[correctAngle(i-1)]+offsetValues[correctAngle(i+2)])/6;
        //     } else if (offsetValues[i] == 0.0 && offsetValues[correctAngle(i+1)] != 0.0) {
        //         offsetValues[i] = (offsetValues[correctAngle(i-1)]+offsetValues[correctAngle(i+1)])/2;
        //     }
        // }

        return offsetValues;
    }

    public double[] simpleOffset(int lidarFrontAngle, double  yOffset) {

        float[] lidarVals = lidar.getData().distance;
        double[] offsetValues = new double[lidarVals.length];

        if (yOffset == 0.0 || yOffset == 0) {
            for (int i = 0; i < offsetValues.length; i++) {
                offsetValues[i] = lidarVals[i]/1000;
            }
            return offsetValues;

        } else {
            
            int sTh, eTh;
    
            sTh = lidarFrontAngle - 90;
            eTh = lidarFrontAngle + 90;
    
            for (int i = (int) sTh; i != ((int) eTh + 1) % 360; i = (i + 1) % 360) {
                double dist = lidarVals[i];
                if (dist == -1) continue;
                dist /= 1000;
    
                double angle_rad = Math.toRadians(findBasicAngle(lidarFrontAngle, i));
                double yDist = Math.sin(angle_rad) * dist;
                double xDist = Math.cos(angle_rad) * dist;
                yDist -= yOffset;
              
                int newTheta = lidarFrontAngle - Math.round((float) Math.toDegrees(Math.atan(yDist/xDist)));
                if (newTheta < 0) newTheta += 360;
                else if (newTheta > 359) newTheta %= 360;
                offsetValues[newTheta] = Math.sqrt(xDist*xDist + yDist*yDist); 
            }
    
            return offsetValues;
        }

    }

    public void debugger(double[] idk) {

        float[] lidarDists = lidar.getData().distance;

        System.out.println("NORMAL LIDAR VALUES:");
        System.out.print("[ ");
        for (int i = 180; i != (0 + 1) % 360; i = (i + 1) % 360)  {
            System.out.print("(" + i + ") " + lidarDists[i]/1000 + " ");
        }
        System.out.println(" ]");

        System.out.println("OFFSET VALUES:");
        System.out.print("[ ");
        for (int i = 180; i != (0 + 1) % 360; i = (i + 1) % 360)  {
            System.out.print("(" + i + ") " + idk[i] + " ");
        }
        System.out.println(" ]");
        System.out.println("PRINT ENDED SUCCESSFULLY");
    }

    public double[] lidarScanWOffset(int lidarCentreAngle, double yOffset) { 
        boolean debug = false;

        double[] scanData = simpleOffset(lidarCentreAngle, yOffset);

        // double[] dist = new double[360 - startAngle + endAngle + 1]; // Array size should match the number of angles processed 181
        // double[] dist = new double[endAngle-startAngle+1];

        double[] dist;
        if (startAngle < endAngle) {
            dist = new double[endAngle-startAngle+1];
        }
        else {
            dist = new double[360 - startAngle + endAngle + 1];
        }
        
        int index = 0;
        // double maxXDist = 0.5;
        double minXDist = 0.00;
        
        for (int i = startAngle; i != (endAngle + 1) % 360; i = (i + 1) % 360) {
        // for (int i = startAngle; i <= endAngle; i++) {
            double distance = scanData[i];
            // double distance = filtereddist[i] / 1000.0;

            double angle_rad = Math.toRadians(i-270);
            // double yDist = Math.abs(Math.sin(angle_rad)) * distance;
            double xDist = Math.abs(Math.cos(angle_rad)) * distance;
            double yDist = Math.abs(Math.sin(angle_rad)) * distance;

            if (distance <= maxXDist && distance >= minXDist && xDist < maxXDist-0.1) {
                dist[index] = distance;
            } else {
                dist[index] = 0;
            }
            index++;
        }

        if (debug) {
            System.out.print("[");
            for (int i = 0; i < dist.length; i++) {
                System.out.print(dist[i]);
                if (i < dist.length - 1) {
                    System.out.print(",");
                }
            }
            System.out.println("]");
        }
        return dist;
    }

    public int correctAngle(int angle) {
        if (angle >= 360) {
            return angle % 360;
        } else if (angle < 0) {
            return 360 + angle;
        } else {
            return angle;
        }
    }


    public int findBasicAngle(int lidarCentreAngle, int angle) {
        return (lidarCentreAngle - angle + 540) % 360 - 180;
    }

    @Override
    public void periodic() {
        // ! Sensors Debug Section
        if (Constants.DEBUG_LIDAR) {
            // scanData = lidar.getData();
            // debugger(offsetLidarValues(270, -0.105));

            // SmartDashboard.putNumber("Angle:", Math.toDegrees(findFreeSpace1(0)[0]));

            // offsetLidarValuesDouble(270, 0, 0.105);

            // SmartDashboard.putNumber("Smallest Value F:", frontDistancewithFilter()[0]);
            // SmartDashboard.putNumber("Largest Value F:", frontDistancewithFilter()[1]);
            // SmartDashboard.putNumber("Distance L 0 deg:", scanData.distance[0]/1000);
            // SmartDashboard.putNumber("Distance R 270 deg:", scanData.distance[270]/1000);
            // SmartDashboard.putNumber("Distance L 180 deg:", scanData.distance[180]/1000);
            // SmartDashboard.putNumber("Distance R 271 deg:", scanData.distance[271]/1000);
            // SmartDashboard.putNumber("Distance L 88 deg:", scanData.distance[88]/1000);
            // SmartDashboard.putNumber("Distance R 272 deg:", scanData.distance[272]/1000);



        }
    }
}
