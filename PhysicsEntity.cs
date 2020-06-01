using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using GXPEngine;

[Flags]
public enum Layer
{
    First = 1,
    Second = 2,
    Third = 3,

}

public abstract class PhysicsEntity:GameObject
{
    public Vector position;
    public Vector velocity;
    public Vector acceleration;

    public float angularVelocity;
    public float angularAcc;

    public float restitution;

    protected List<PhysicsEntity> CollisionList = new List<PhysicsEntity>();
    protected List<PhysicsEntity> prevCollisionList = new List<PhysicsEntity>();

    protected float invMass;
    protected float invInertia;
    protected float inertia;

    protected float staticFriction = 0.6f;
    protected float dynamicFriction = 0.2f;

    private Vector COM;

    protected Vector COMOffset = new Vector(0, 0);

    private bool IsDestroyed = false;

    public float Restitution
    {
        get
        {
            return restitution;
        }
        set
        { restitution = Mathf.Clamp(value,0,1); }
    }
    
    protected Layer Layer;

    public float ExternalRot = 0;
    public Vector ExternalVel = Vector.Zero();

    public static int ID = 0;
    public int IDP;

    public PhysicsEntity(float restitutionToSet,float Mass,Vector x,Vector v,Vector a,string filename,Layer layer):base()
    {
        Layer = layer;
        //game.AddChildAt(this,childLayer);
        //SetOrigin(width / 2, height / 2);
        restitution = restitutionToSet;

        if(Utils.IsNearEqual(Mass,0f,0.01f))
        {
            invMass = 0;
            invInertia = 0;
        }
        else
        {
            invMass = 1 / Mass;

        }
        position = x;
        velocity = v;
        acceleration = a;
        UpdatePosition(position);

        MyGame.AddEntity(this);
        COM = position;

        IDP = ID;
        ID++;
    }

    //Update every physics time step
    abstract public void PhysicsUpdate(float Coefficient);

    public float GetInvMass()
    {
        return invMass;
    }

    public void AddToCollisionList(PhysicsEntity Entity)
    {
        CollisionList.Add(Entity);
    }

    public bool IsMarkedDestroy()
    {
        return IsDestroyed;
    }

    public void MarkDestroyed()
    {
        IsDestroyed = true;
    }

    public float GetInvInertia()
    {
        return invInertia;
    }

    virtual protected void UpdatePosition(Vector Position)
    {
        x = Position.x;
        y = Position.y;

    }

    public Layer GetLayer()
    {
        return Layer;

    }

    public float GetDynamicFriction()
    {
        return dynamicFriction;
    }

    public float GetStaticFriction()
    {
        return staticFriction;
    }
    public void ApplyImpulse(Vector impulse, Vector r)
    {
        velocity.Add(impulse * invMass);
        angularVelocity += Mathf.Rad2Deg(invInertia * Vector.Cross(r, impulse));
    }
    
    public Vector GetCenterOfMass()
    {
        return COM;
    }
    public void UpdateCOM(Vector COMOffset)
    {
        COM = position + COMOffset;
    }
    public void SetOffset(Vector Offset)
    {
        COMOffset = Offset;
    }

    public abstract bool CheckCollision(CircleEntity other);

    public abstract bool CheckCollision(PhysicsEntity other);

    public abstract bool CheckCollision(PolygonEntity other);
    

    public abstract Manifold GenerateManifold(CircleEntity other);

    public abstract Manifold GenerateManifold(PhysicsEntity other);

    public abstract Manifold GenerateManifold(PolygonEntity other);


}

