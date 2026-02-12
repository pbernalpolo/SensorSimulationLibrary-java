package sensorSimulationLibrary.trajectories3d;


import numericalLibrary.types.Quaternion;
import numericalLibrary.types.UnitQuaternion;
import numericalLibrary.types.Vector3;



abstract class TrajectoryOfReferenceFrameFromOrientation
{
    ////////////////////////////////////////////////////////////////
    // PROTECTED VARIABLES
    ////////////////////////////////////////////////////////////////
    protected Quaternion q;  // quat_F_from_I
    protected Quaternion qd;  // quat_F_from_I_dot
    protected Quaternion qdd;  // quat_F_from_I_dotdot
    
    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC METHODS
    ////////////////////////////////////////////////////////////////

    public UnitQuaternion orientation()
    {
        return UnitQuaternion.fromNormalizedQuaternion( this.q );
    }

    
    public Vector3 angularVelocity()
    {
        // qdot = 1/2 q * omg  =>  omg = 2 q^* qdot
        return this.q.conjugate().multiply( this.qd ).vectorPart().scaleInplace( 2.0 );
    }


    public Vector3 angularAcceleration()
    {
        // alp = d omg/dt = 2 * ( qdot^* qdot + q^* qdotdot )
        Vector3 alpFirst = this.qd.conjugate().multiply( this.qd ).vectorPart();
        Vector3 alpSecond = this.q.conjugate().multiply( this.qdd ).vectorPart();
        return alpFirst.addInplace( alpSecond ).scaleInplace( 2.0 );
    }
    
}
