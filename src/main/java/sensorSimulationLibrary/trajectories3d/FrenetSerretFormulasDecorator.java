package sensorSimulationLibrary.trajectories3d;


import numericalLibrary.types.MatrixReal;
import numericalLibrary.types.UnitQuaternion;
import numericalLibrary.types.Vector3;



public class FrenetSerretFormulasDecorator
    implements DifferentiableTrajectory3dOfReferenceFrame
{
    ////////////////////////////////////////////////////////////////
    // PRIVATE VARIABLES
    ////////////////////////////////////////////////////////////////
    private DifferentiableTrajectory3dOfPoint dtp;  // the DifferentiableTrajectoryOfPoint used to generate the orientation for the reference frame
    
    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC CONSTRUCTORS
    ////////////////////////////////////////////////////////////////

    public FrenetSerretFormulasDecorator( DifferentiableTrajectory3dOfPoint theDifferentiableTrajectoryOfPoint )
    {
        this.dtp = theDifferentiableTrajectoryOfPoint;
    }
    
    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC METHODS
    ////////////////////////////////////////////////////////////////
    
    public void setTime( double theTime )
    {
        this.dtp.setTime( theTime );
    }
    
    
    public double time()
    {
        return this.dtp.time();
    }
    
    
    public Vector3 position()
    {
        return this.dtp.position();
    }
    
    
    public Vector3 velocity()
    {
        return this.dtp.velocity();
    }
    
    
    public Vector3 acceleration()
    {
        return this.dtp.acceleration();
    }
    
    
    public Vector3 jerk()
    {
        return this.dtp.jerk();
    }
    
    
    public UnitQuaternion orientation()
    {
        Vector3 v = this.velocity();
        Vector3 aOOF = this.acceleration();
        Vector3 T = v.normalize();
        Vector3 B = v.crossProduct( aOOF ).normalizeInplace();
        Vector3 N = B.crossProduct( T );
        MatrixReal R = MatrixReal.fromEntries3x3(
                T.x() , N.x() , B.x() ,
                T.y() , N.y() , B.y() ,
                T.z() , N.z() , B.z() );
        return UnitQuaternion.fromRotationMatrix( R );
    }
    
    
    public Vector3 angularVelocity()
    {
        Vector3 vOOF = this.velocity();
        Vector3 aOOF = this.acceleration();
        Vector3 jOOF = this.jerk();
        // we start by computing kappa and tau
        double vOOFnorm = vOOF.norm();
        double uanorm = vOOF.normalize().crossProduct( aOOF ).norm();
        double kappa = uanorm / vOOFnorm;
        double tau = ( ( vOOF.dot( aOOF.crossProduct( jOOF ) ) / uanorm )/ uanorm )/ vOOFnorm;
        return Vector3.fromComponents( tau , 0.0 , kappa );
    }
    
}
