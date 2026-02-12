package sensorSimulationLibrary.trajectories3d;


import numericalLibrary.types.Quaternion;
import numericalLibrary.types.UnitQuaternion;
import numericalLibrary.types.Vector3;



public class Trajectory3dOfReferenceFrameNumericalDifferentiator
    extends Trajectory3dOfPointNumericalDifferentiator
    implements DifferentiableTrajectory3dOfReferenceFrame
{
    ////////////////////////////////////////////////////////////////
    // PRIVATE VARIABLES
    ////////////////////////////////////////////////////////////////
    private ContinuousTrajectory3dOfReferenceFrame ctf;  // the ContinuousTrajectoryOfReferenceFrame to be differentiated
    
    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC CONSTRUCTORS
    ////////////////////////////////////////////////////////////////
    
    public Trajectory3dOfReferenceFrameNumericalDifferentiator( ContinuousTrajectory3dOfReferenceFrame theContinuousTrajectoryOfReferenceFrame )
    {
        super( theContinuousTrajectoryOfReferenceFrame );
        this.ctf = theContinuousTrajectoryOfReferenceFrame;
    }
    
    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC METHODS
    ////////////////////////////////////////////////////////////////
    
    public UnitQuaternion orientation()
    {
        this.ctf.setTime( this.t );
        return this.ctf.orientation();
    }
    
    
    public Vector3 angularVelocity()
    {
        // first we obtain the quaternion
        this.ctf.setTime( this.t );
        Quaternion qOF = this.ctf.orientation().quaternion();
        // then we compute the derivative of the quaternion
        this.ctf.setTime( this.t + this.h );
        Quaternion qOFplus = this.ctf.orientation().quaternion();
        this.ctf.setTime( this.t - this.h );
        Quaternion qOFminus = this.ctf.orientation().quaternion();
        Quaternion qOFdot = qOFplus.subtract( qOFminus ).scale( 1.0/( 2.0 * this.h ) );
        // now we can use the quaternion and its derivative to compute the angular velocity (omg = 2 q^* qdot)
        return qOF.conjugate().multiply( qOFdot ).vectorPart().scaleInplace( 2.0 );
    }
    
    
    public Vector3 angularAcceleration()
    {
        double t0 = this.t;
        this.setTime( t0 + this.h );
        Vector3 omgFOFplus = this.angularVelocity();
        this.setTime( t0 - this.h );
        Vector3 omgFOFminus = this.angularVelocity();
        Vector3 alpFOF = this.numericalDerivative( omgFOFplus , omgFOFminus );
        this.setTime( t0 );
        return alpFOF;
    }
    
}
