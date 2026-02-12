package trajectories3d;

import sensorSimulationLibrary.trajectories3d.SphereTrajectory;

class SphereTrajectoryTest
    extends DifferentiableTrajectoryOfReferenceFrameTester
{
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC CONSTRUCTORS
    ////////////////////////////////////////////////////////////////
    
    public SphereTrajectoryTest()
    {
        this.addTrajectoryForTesting( new SphereTrajectory( 50.0 , 1.0 , 3.0 ) );
    }
    
}
