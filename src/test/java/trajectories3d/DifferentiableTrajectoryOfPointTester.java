package trajectories3d;


import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

import numericalLibrary.types.Vector3;
import sensorSimulationLibrary.trajectories3d.DifferentiableTrajectory3dOfPoint;
import sensorSimulationLibrary.trajectories3d.Trajectory3dOfPointNumericalDifferentiator;



abstract class DifferentiableTrajectoryOfPointTester<T extends DifferentiableTrajectory3dOfPoint>
{
    ////////////////////////////////////////////////////////////////
    // PROTECTED VARIABLES
    ////////////////////////////////////////////////////////////////
    protected List<T> lTrajectories;
    protected double Ti;  // initial time
    protected double Tf;  // final time
    protected double dt;  // time increment
    protected double h;  // increment used to compute numerical derivatives
    
    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC CONSTRUCTORS
    ////////////////////////////////////////////////////////////////

    public DifferentiableTrajectoryOfPointTester()
    {
        this.lTrajectories = new ArrayList<T>();
        this.Ti = 0.0;
        this.Tf = 10.0;
        this.dt = 0.1;
        this.h = 1.0e-6;
    }
    
    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC METHODS
    ////////////////////////////////////////////////////////////////
    
    public void addTrajectoryForTesting( T theTrajectory )
    {
        this.lTrajectories.add( theTrajectory );
    }
    
    
    @Test
    public void velocityIsPositionDerivative()
    {
        for( T trajectory : this.lTrajectories ) {
            DifferentiableTrajectory3dOfPoint td = new Trajectory3dOfPointNumericalDifferentiator( trajectory );
            for(double t=this.Ti; t<this.Tf; t+=this.dt) {
                // we obtain the velocity from the trajectory
                trajectory.setTime( t );
                Vector3 vOOF = trajectory.velocity();
                // we obtain the velocity from the TrajectoryOfPointNumericalDifferentiator
                td.setTime( t );
                Vector3 vOOFnumerical = td.velocity();
                // we check if they are the same
                double tolerance = 1.0e-6;
                assertEquals( vOOFnumerical.x() , vOOF.x() , tolerance );
                assertEquals( vOOFnumerical.y() , vOOF.y() , tolerance );
                assertEquals( vOOFnumerical.z() , vOOF.z() , tolerance );
            }
        }
    }
    
    
    @Test
    public void accelerationIsVelocityDerivative()
    {
        for( T trajectory : this.lTrajectories ) {
            DifferentiableTrajectory3dOfPoint td = new Trajectory3dOfPointNumericalDifferentiator( trajectory );
            for(double t=this.Ti; t<this.Tf; t+=this.dt) {
                // we obtain the velocity from the trajectory
                trajectory.setTime( t );
                Vector3 aOOF = trajectory.acceleration();
                // we obtain the velocity from the TrajectoryOfPointNumericalDifferentiator
                td.setTime( t );
                Vector3 aOOFnumerical = td.acceleration();
                // we check if they are the same
                double tolerance = 1.0e-2;
                assertEquals( aOOFnumerical.x() , aOOF.x() , tolerance );
                assertEquals( aOOFnumerical.y() , aOOF.y() , tolerance );
                assertEquals( aOOFnumerical.z() , aOOF.z() , tolerance );
            }
        }
    }
    
    
    @Test
    public void jerkIsAccelerationDerivative()
    {
        for( T trajectory : this.lTrajectories ) {
            DifferentiableTrajectory3dOfPoint td = new Trajectory3dOfPointNumericalDifferentiator( trajectory );
            for(double t=this.Ti; t<this.Tf; t+=this.dt) {
                // we obtain the velocity from the trajectory
                trajectory.setTime( t );
                Vector3 jOOF = trajectory.jerk();
                // we obtain the velocity from the TrajectoryOfPointNumericalDifferentiator
                td.setTime( t );
                Vector3 jOOFnumerical = td.jerk();
                // we check if they are the same
                double tolerance = 3.0e2;
                assertEquals( jOOFnumerical.x() , jOOF.x() , tolerance );
                assertEquals( jOOFnumerical.y() , jOOF.y() , tolerance );
                assertEquals( jOOFnumerical.z() , jOOF.z() , tolerance );
            }
        }
    }
    
}
