package org.archwood.frc2607;

import java.awt.Color;
import java.io.File;
import java.io.FileWriter;

import com.team254.lib.trajectory.Path;
import com.team254.lib.trajectory.PathGenerator;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryGenerator;
import com.team254.lib.trajectory.WaypointSequence;
import com.team254.lib.trajectory.io.TextFileSerializer;
import com.team254.lib.trajectory.Trajectory.Segment;

import usfirst.frc.team2168.robot.FalconLinePlot;
import usfirst.frc.team2168.robot.FalconPathPlanner;

public class Main {
	
	public static void main(String args[]) {
		
		/* right peg:
			Waypoint(0.0, 0.0, 0.0)
			Waypoint(3.5, -.5, 6.0)
			Waypoint(7.83, 1.75, Math.PI / 3.0)
		*/
		/* left peg:
		 * actually can just run the same trajectory, just swap sides
		 * 		i.e. send the left trajectory to the right wheels, and right
		 * 		trajectory to the left wheels
		 * 
		 * or:
		 * 		Waypoint(0.0, 0.0, 0.0)
		 * 		Waypoint(3.5, .5, .3)
		 * 		Waypoint(7.83, -1.75, (5.0/3.0) * Math.PI)
		 * 
		 * 		or 0,0,0 and 7.5, -1.75, 5.6
		 * 
		 * 
		 */

		TrajectoryGenerator.Config cheesyConfig = new TrajectoryGenerator.Config();
		cheesyConfig.dt = .05;
		cheesyConfig.max_acc = 7.0;
		cheesyConfig.max_jerk = 30.0;
		cheesyConfig.max_vel = 7.0;

		WaypointSequence p = new WaypointSequence(10);
        p.addWaypoint(new WaypointSequence.Waypoint(0.0, 0.0, 0.0));
        p.addWaypoint(new WaypointSequence.Waypoint(4.5, 0.0, 0.0));
        p.addWaypoint(new WaypointSequence.Waypoint(7.8, -1.8, 5.2));

        // FalconPathPlanner assumes absolute x,y positions on graph, whereas cheesyPoofs assume positions
        // are relative to initial robot position (i.e. cheesy position (0,0) is robot start)
        // convert cheesy Waypoints into absolute coordinates for FalconPathPlanner
		double robotOriginX = 3.0, robotOriginY = 23.0 - 0.0875 - (52.0 / 12.0);

		double trackWidth = 29.872 / 12.0; //(25.75/12.0);
		
		double startTime = System.currentTimeMillis();

        startTime = System.currentTimeMillis();
        Path cheesyPath = PathGenerator.makePath(p, cheesyConfig,
                trackWidth, "Left Peg");
        System.out.println("cheesyPath calculated in " + (System.currentTimeMillis() - startTime) + "ms");
        
        TextFileSerializer tfs = new TextFileSerializer();
        String traj = tfs.serialize(cheesyPath);

        try {
        	FileWriter f = new FileWriter(new File("leftPeg.txt"));
        	f.write(traj);
        	f.flush();
        	f.close();
        } catch (Exception e) {
        	e.printStackTrace();
        }     
        
        Trajectory cheesyLeftTrajectory = cheesyPath.getLeftWheelTrajectory(),
        		   cheesyRightTrajectory = cheesyPath.getRightWheelTrajectory();
        
		double[][] cheesyLeftPath = new double[cheesyLeftTrajectory.getNumSegments()][2],
				   cheesyRightPath = new double[cheesyRightTrajectory.getNumSegments()][2],
				   cheesyLeftVelocity = new double[cheesyLeftTrajectory.getNumSegments()][2],
				   cheesyRightVelocity = new double[cheesyRightTrajectory.getNumSegments()][2],
				   cheesyLeftPos = new double[cheesyLeftTrajectory.getNumSegments()][2],
				   cheesyRightPos = new double[cheesyRightTrajectory.getNumSegments()][2];

		for (int i=0; i<cheesyLeftPath.length;i++) {
			Segment s = cheesyLeftTrajectory.getSegment(i);
			cheesyLeftPath[i][0] = s.x + robotOriginX;
			cheesyLeftPath[i][1] = s.y + robotOriginY;
			if (i==0) {
				cheesyLeftVelocity[i][0] = s.dt;
				cheesyLeftPos[i][0] = s.dt;
			} else {
				cheesyLeftVelocity[i][0] = cheesyLeftVelocity[i-1][0] + s.dt;
				cheesyLeftPos[i][0] = cheesyLeftPos[i-1][0] + s.dt;
			}
			cheesyLeftVelocity[i][1] = s.vel;
			cheesyLeftPos[i][1] = s.pos;
			System.out.print(i + ": LtPos: " + s.pos + " LtVel: " + s.vel + " LtRPM: " + 
					(s.vel * 2173.0) * 60.0 / 1024.0 / 4.0);
			s = cheesyRightTrajectory.getSegment(i);
			cheesyRightPath[i][0] = s.x + robotOriginX;
			cheesyRightPath[i][1] = s.y + robotOriginY;
			if (i==0) {
				cheesyRightVelocity[i][0] = s.dt;
				cheesyRightPos[i][0] = s.dt;
			} else {
				cheesyRightVelocity[i][0] = cheesyRightVelocity[i-1][0] + s.dt;
				cheesyRightPos[i][0] = cheesyRightPos[i-1][0] + s.dt;
			}
			cheesyRightVelocity[i][1] = s.vel;
			cheesyRightPos[i][1] = s.pos;
			System.out.println(" RtPos: " + s.pos + " RtVel: " + s.vel + " RtRPM: " +
					(s.vel * 2173.0) * 60.0 / 1024.0 / 4.0);
		}
        
		double fieldWidth = 27.0;
		double fieldLength = (((54.0 * 12.0) + 4.0) / 12.0) / 2.0;
        
		// plot trajectory
		FalconLinePlot cheesyFieldPlot = new FalconLinePlot(cheesyLeftPath, Color.MAGENTA, Color.MAGENTA);
        cheesyFieldPlot.addData(cheesyRightPath, Color.MAGENTA, Color.MAGENTA);
        
        // add horizontal bisecting line
        double fieldCenterY = fieldWidth / 2.0;
        cheesyFieldPlot.addData(new double[][] {{0.0,fieldCenterY},{fieldLength,fieldCenterY}}, Color.BLACK);
		
        // add key corner
        double[][] keyCorner = new double[][] {
        	{0, fieldWidth - (36.875 / 12.0)},
        	{(36.5 / 12.0), fieldWidth}
        };
               
        cheesyFieldPlot.addData(keyCorner, Color.BLACK);
        
        // add retrieval station corner
        double[][] retrCorner = new double[][] {
        	{0,38.0 / 12.0},
        	{72.0625 / 12.0, 0}
        };
        cheesyFieldPlot.addData(retrCorner, Color.BLACK);
        
        // each heaxgon side should be 40.755in, based on 70.59in side-to-side measure (70.59 / sqrt(3))
		double hxSideLength = 40.55 / 12.0;
        double[][] dsSide = new double[][] {{(114.3 / 12.0),fieldCenterY + (hxSideLength / 2.0)},
			{(114.3 / 12.0),fieldCenterY - (hxSideLength / 2.0)}};
        // heaxagon is centered on the fieldWidth, and is 114.3in from driver station wall
        cheesyFieldPlot.addData(dsSide, Color.BLACK);
        
        // endpoints of subsequent sides are [hxSideLength * cos(60)] +/- x, [hxSideLength * sin(60)] +/- y
        double cos60 = Math.cos(Math.PI / 3.0), 
        	   sin60 = Math.sin(Math.PI / 3.0);
        double[][] upperSides = new double[2][2];
        upperSides[0][0] = dsSide[0][0];
        upperSides[0][1] = dsSide[0][1];
        upperSides[1][0] = upperSides[0][0] + (hxSideLength * cos60);
        upperSides[1][1] = upperSides[0][1] + (hxSideLength * sin60);
        
        double[][] lowerSides = new double[2][2];
        lowerSides[0][0] = dsSide[1][0];
        lowerSides[0][1] = dsSide[1][1];
        lowerSides[1][0] = lowerSides[0][0] + (hxSideLength * cos60);
        lowerSides[1][1] = lowerSides[0][1] - (hxSideLength * sin60);

        cheesyFieldPlot.addData(upperSides, Color.BLACK);
        cheesyFieldPlot.addData(lowerSides, Color.BLACK);
        
        
        
        
        cheesyFieldPlot.xGridOn();
		cheesyFieldPlot.yGridOn();
		cheesyFieldPlot.setXTic(0, fieldLength, 1);
		cheesyFieldPlot.setYTic(0, fieldWidth, 1);

		FalconLinePlot cheesyMotorPlot = new FalconLinePlot(cheesyLeftVelocity, Color.RED, Color.RED);
		cheesyMotorPlot.addData(cheesyRightVelocity, Color.GREEN, Color.GREEN);
		cheesyMotorPlot.addData(cheesyLeftPos, Color.PINK, Color.PINK);
		cheesyMotorPlot.addData(cheesyRightPos, Color.CYAN, Color.CYAN);
		cheesyMotorPlot.xGridOn();
		cheesyMotorPlot.yGridOn();
		

		System.out.println(" cheesyPath points: " + cheesyPath.getLeftWheelTrajectory().getNumSegments());
	}

}
