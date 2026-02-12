package trajectories3d;


import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import numericalLibrary.types.Vector3;
import sensorSimulationLibrary.trajectories3d.DifferentiableTrajectory3dOfReferenceFrame;
import sensorSimulationLibrary.trajectories3d.Trajectory3dOfReferenceFrameNumericalDifferentiator;



abstract class DifferentiableTrajectoryOfReferenceFrameTester
    extends DifferentiableTrajectoryOfPointTester<DifferentiableTrajectory3dOfReferenceFrame>
{
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC METHODS
    ////////////////////////////////////////////////////////////////
    
    @Test
    public void angularVelocityFromDifferentialEquation()
    {
        for( DifferentiableTrajectory3dOfReferenceFrame trajectory : this.lTrajectories ) {
            DifferentiableTrajectory3dOfReferenceFrame td = new Trajectory3dOfReferenceFrameNumericalDifferentiator( trajectory );
            for( double t=this.Ti; t<this.Tf; t+=this.dt ) {
                // we obtain the velocity from the trajectory
                trajectory.setTime( t );
                Vector3 omgFOF = trajectory.angularVelocity();
                // we obtain the velocity from the TrajectoryOfPointNumericalDifferentiator
                td.setTime( t );
                Vector3 omgFOFnumerical = td.angularVelocity();
                // we check if they are the same
                double tolerance = 1.0e-6;
                assertEquals( omgFOFnumerical.x() , omgFOF.x() , tolerance );
                assertEquals( omgFOFnumerical.y() , omgFOF.y() , tolerance );
                assertEquals( omgFOFnumerical.z() , omgFOF.z() , tolerance );
            }
        }
    }
    
}
