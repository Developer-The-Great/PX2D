using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using GXPEngine;


public class PolygonEntity:PhysicsEntity
{

    protected float OldRot;
    protected Vector[] vertices;
    protected Vector[] localSpaceVertices;
    
    protected float BoundingCircleRadius = 0;

    public int PolyID = 0;
    
    public PolygonEntity(float restitution, float Mass, Vector xpos, Vector v, Vector a,  Layer layer,Vector[] _vertices,string filename) :base(restitution, Mass, xpos, v, a, filename, layer)
    {

        localSpaceVertices = new Vector[_vertices.Length];
        Array.Copy(_vertices, localSpaceVertices, _vertices.Length);

        vertices = new Vector[localSpaceVertices.Length];

        float EstWidth = Mathf.Abs(GetSupportPoint(Vector.Right()).x - GetSupportPoint(Vector.Left()).x) ;
        float EstHeight = Mathf.Abs(GetSupportPoint(Vector.Up()).y - GetSupportPoint(Vector.Down()).y);

        inertia = Mass * (((EstWidth * EstWidth) + (EstHeight * EstHeight))*1/6);
        invInertia = 1 / inertia;

        if (Utils.IsNearEqual(Mass, 0f, 0.01f))
        {
            invMass = 0;
            invInertia = 0;
        }
        else
        {
            invMass = 1 / Mass;

        }
        float[] furthestdistance = new float[4];

        Vector supportPoint = GetSupportPoint(Vector.Left());
        furthestdistance[0] = Mathf.Abs(Mathf.Distance(0,0,supportPoint.x,supportPoint.y));

        supportPoint = GetSupportPoint(Vector.Right());
        furthestdistance[1] = Mathf.Abs(Mathf.Distance(0, 0, supportPoint.x, supportPoint.y));

        supportPoint = GetSupportPoint(Vector.Up());
        furthestdistance[2] = Mathf.Abs(Mathf.Distance(0, 0, supportPoint.x, supportPoint.y));

        supportPoint = GetSupportPoint(Vector.Down());
        furthestdistance[3] = Mathf.Abs(Mathf.Distance(0, 0, supportPoint.x, supportPoint.y));

        for (int i = 0; i < furthestdistance.Length; i++)
        {
            if(furthestdistance[i] > BoundingCircleRadius)
            {
                BoundingCircleRadius = furthestdistance[i];
            }

        }

    }

    void Update()
    {
        CollisionList.Clear();
    }

    public override void PhysicsUpdate(float Coefficient)
    {
        OldRot = rotation;

        velocityIntegration(Coefficient);

        angularVelocityIntegration();

        UpdateCOM(COMOffset);

        

        for (int i = 0; i < vertices.Length; i++)
        {
            localSpaceVertices[i] = Vector.rotate_point((GetCenterOfMass() - position).x, (GetCenterOfMass() - position).y, rotation - OldRot, localSpaceVertices[i]);

            vertices[i] = position + localSpaceVertices[i];
        }
    }

    protected void velocityIntegration(float Coefficient)
    {
        UpdatePosition(position * Coefficient);
        position.Add(velocity * Coefficient);
        position.Add(ExternalVel);
        velocity.Add(acceleration * Coefficient);
        ExternalVel = Vector.Zero();

    }

    protected void angularVelocityIntegration()
    {
        rotation += angularVelocity;
        rotation += ExternalRot;
        angularVelocity += angularAcc;
        ExternalRot = 0;

    }

    public Vector GetSupportPoint(Vector dir)
    {
        Vector BestVertex = new Vector(-1000,-1000);
        float bestProjection = -1000;

        for (int i = 0; i < localSpaceVertices.Length; i++)
        {
            float projection = Vector.Dot(localSpaceVertices[i], dir);

            if(projection > bestProjection)
            {
                bestProjection = projection;
                BestVertex = localSpaceVertices[i];
            }

        }
        return BestVertex;
    }

    public float GetVolumeRadius()
    {
        return BoundingCircleRadius;
    }

    public override bool CheckCollision(CircleEntity other)
    {
        return CollisionDetection.DetectBoundingVolumeCollision(other, this);
    }

    public override bool CheckCollision(PolygonEntity other)
    {
        return CollisionDetection.DetectBoundingVolumeCollision(this, other);
    }

    public override bool CheckCollision(PhysicsEntity other)
    {
        return other.CheckCollision(this);
    }

    public override Manifold GenerateManifold(CircleEntity other)
    {
        return CollisionResolution.GenerateManifold(this,other);
    }

    public override Manifold GenerateManifold(PolygonEntity other)
    {
        return CollisionResolution.GenerateManifold(this, other);
    }

    public override Manifold GenerateManifold(PhysicsEntity other)
    {
        return other.GenerateManifold(this);
    }

    public Vector[] GetVertices()
    {
        return vertices;
    }
    public Vector[] GetLocalSpaceVertices()
    {
        return localSpaceVertices;
    }
    public Vector GetFaceNormal(int index)
    {
        Vector v1 = localSpaceVertices[index];

        int secondIndex = index + 1 >= localSpaceVertices.Length ? 0 : index + 1;
        Vector v2 = localSpaceVertices[secondIndex];


        return (v2-v1).GetNormal();

    }
    public Vector GetFaceDirection(int index)
    {
        Vector v1 = localSpaceVertices[index];

        int secondIndex = index + 1 >= localSpaceVertices.Length ? 0 : index + 1;
        Vector v2 = localSpaceVertices[secondIndex];


        return (v2 - v1);

    }
    public List<Vector> GetVerticeList()
    {
        List<Vector> verticeList = new List<Vector>();
        for(int i =0;i< vertices.Length;i++)
        {
            verticeList.Add(vertices[i]);
        }

        return verticeList;
    }
    public void SetPolyID(int ID)
    {
        PolyID = ID;
    }
}

