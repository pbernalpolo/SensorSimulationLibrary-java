package trajectories3d;

import sensorSimulationLibrary.trajectories3d.CircleTrajectory;

class CircleTrajectoryTest
    extends DifferentiableTrajectoryOfReferenceFrameTester
{
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC CONSTRUCTORS
    ////////////////////////////////////////////////////////////////
    
    public CircleTrajectoryTest()
    {
        this.addTrajectoryForTesting( new CircleTrajectory( 50.0 , 1.0 ) );
    }

}
