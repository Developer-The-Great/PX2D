using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using GXPEngine;

public class CircleEntity:PhysicsEntity
{
    public float Radius { private set; get; }
    
    public CircleEntity(float restitution,float Mass, Vector x, Vector v, Vector a,float radius,Layer layer) :base(restitution,Mass, x, v, a, "bomb.png", layer)
    {
        dynamicFriction = 0.5f;
        Radius = radius-2.5f;
        //width = (int)radius * 2;
       // height = (int)radius * 2;

        
        if (Utils.IsNearEqual(Mass, 0f, 0.01f))
        {
            invMass = 0;
            invInertia = 0;
        }

        else
        {
            Console.WriteLine((2f / 5f) * Mass * radius * radius);
            inertia = (2f / 5f) * Mass * radius * radius;
            invInertia = 1f / inertia;

        }

    }

    void Update()
    {
        CollisionList.Clear();
    }

    public override void PhysicsUpdate(float Coefficient)
    {
        velocityIntegration();

        angularVelocityIntegration( );

        UpdateCOM(COMOffset);

        Console.WriteLine(GetInvInertia());
    }

    protected void velocityIntegration()
    {
        UpdatePosition(position);
        position.Add(velocity);
        position.Add(ExternalVel);
        velocity.Add(acceleration);
        ExternalVel = Vector.Zero();

    }

    protected void angularVelocityIntegration()
    {
        rotation += angularVelocity;
        rotation += ExternalRot;
        angularVelocity += angularAcc;
        ExternalRot = 0;
    }

    public override bool CheckCollision(PhysicsEntity other)
    {
        return other.CheckCollision(this);
    }

    public override bool CheckCollision(CircleEntity other)
    {
        return true;
    }

    public override Manifold GenerateManifold(CircleEntity other)
    {
        return CollisionResolution.GenerateManifold(this, other);
    }

    public override Manifold GenerateManifold(PhysicsEntity other)
    {
        return other.GenerateManifold(this);
    }

    public override Manifold GenerateManifold(PolygonEntity other)
    {
        return CollisionResolution.GenerateManifold(other,this);
    }

    public override bool CheckCollision(PolygonEntity other)
    {
        return CollisionDetection.DetectBoundingVolumeCollision(this,other); ;
    }
}

