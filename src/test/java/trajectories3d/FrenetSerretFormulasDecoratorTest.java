package trajectories3d;

import sensorSimulationLibrary.trajectories3d.CircleTrajectory;
import sensorSimulationLibrary.trajectories3d.CylinderTrajectory;
import sensorSimulationLibrary.trajectories3d.FrenetSerretFormulasDecorator;
import sensorSimulationLibrary.trajectories3d.SphereTrajectory;
import sensorSimulationLibrary.trajectories3d.TorusTrajectory;

class FrenetSerretFormulasDecoratorTest
    extends DifferentiableTrajectoryOfReferenceFrameTester
{
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC CONSTRUCTORS
    ////////////////////////////////////////////////////////////////

    public FrenetSerretFormulasDecoratorTest()
    {
        this.addTrajectoryForTesting( new FrenetSerretFormulasDecorator( new CircleTrajectory( 50.0 , 1.0 ) ) );
        this.addTrajectoryForTesting( new FrenetSerretFormulasDecorator( new CylinderTrajectory( 50.0 , 50.0 , 4.0 , 1.0 ) ) );
        this.addTrajectoryForTesting( new FrenetSerretFormulasDecorator( new SphereTrajectory( 50.0 , 4.0 , 1.0 ) ) );
        this.addTrajectoryForTesting( new FrenetSerretFormulasDecorator( new TorusTrajectory( 50.0 , 20.0 , 10.0 , 1.0 ) ) );
    }
    
}
