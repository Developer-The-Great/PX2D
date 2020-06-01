using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using GXPEngine;


public struct Manifold
{
    public PhysicsEntity A;
    public PhysicsEntity B;
    public Vector Normal;
    public float PenetrationDepth;
    public List<Vector> ContactPoint;
    public bool IsEmpty;

}

public static class CollisionResolution
{
    static float flipperMultiplier = 0.7f;
    static float EPSILON = 0.0001f;

    /// <summary>
    /// Given a manifold containing all of the required information to resolve a known collision between 2 Colliders,
    /// resolves the collision
    /// </summary>
    /// <param name="m"> A Manifold containing the required collision information </param>
    public static void ResolveCollisions(Manifold m)
    {
        PhysicsEntity A = m.A;
        PhysicsEntity B = m.B;

        if (m.IsEmpty || IsEntitiesInfinitelyMassed(A, B)) { return; }

        
        float e = Mathf.Min(m.A.Restitution, m.B.Restitution);


        float AAngVelo = Mathf.Deg2Rad(angularFlipperException(A));
        float BAngVelo = Mathf.Deg2Rad(angularFlipperException(B));

        Vector BVel = B.velocity;
        Vector AVel = A.velocity;

        for (int i = 0; i < m.ContactPoint.Count; i++)
        {
            Vector rAP = m.ContactPoint[i] - A.GetCenterOfMass();
            Vector rBP = m.ContactPoint[i] - B.GetCenterOfMass();

            //get relative velocity 
            Vector AB = BVel + (Vector.Cross(BAngVelo, rBP)) - AVel - (Vector.Cross(AAngVelo, rAP));

            Vector normal = m.Normal.Normalized();

            CorrectPosition(m.PenetrationDepth, A, B, -1 * normal);

            //get relative velocity along the normal
            float NormalVelocity = Vector.Dot(AB, normal);

            //if they are seperating, do not resolve
            if (NormalVelocity > 0)
            {
                return;
            }

            float jNumerator = -1 * (1f + e) * NormalVelocity;

            float rAPCross = Vector.Cross(rAP, normal);
            float rBPCross = Vector.Cross(rBP, normal);

            float jDenominator = A.GetInvMass() + B.GetInvMass() + (Mathf.Pow(rAPCross, 2) * A.GetInvInertia()) + (Mathf.Pow(rBPCross, 2) * B.GetInvInertia());

            float j = jNumerator / jDenominator;

            j /= m.ContactPoint.Count;

            A.ApplyImpulse(-j * normal, rAP);
            B.ApplyImpulse(j * normal, rBP);

            //FRICTION

            Vector tangent = AB - (normal * Vector.Dot(AB, normal));
            tangent.Normalize();

            //find jt
            float jTangent = -Vector.Dot(AB, tangent);
            jTangent = jTangent / jDenominator;

            //spread out friction based on contact point
            jTangent = jTangent / m.ContactPoint.Count;

            float mu = SetFriction(A.GetStaticFriction(), B.GetStaticFriction());

            Vector frictionImpulse;

            //Ffriciton < normal * mu
            if (Mathf.Abs(jTangent) < j * mu)
            {
                //force applied on object did not pass activation force
                frictionImpulse = jTangent * tangent;
            }
            else
            {
                //force applied on object passes activation force
                float dynamicFriction = SetFriction(A.GetDynamicFriction(), B.GetDynamicFriction());
                frictionImpulse = -j * tangent * dynamicFriction;
            }

            A.ApplyImpulse(-1*frictionImpulse, rAP);
            B.ApplyImpulse( frictionImpulse, rBP);
        }
        
    }

    /// <summary>
    /// Checks if the given PhysicsEntities are both infintely massed
    /// </summary>
    /// <returns></returns>
    private static bool IsEntitiesInfinitelyMassed(PhysicsEntity A,PhysicsEntity B)
    {
        return Utils.IsNearEqual(A.GetInvMass(), 0, 0.001f) && Utils.IsNearEqual(B.GetInvMass(), 0, 0.001f);
    }

    /// <summary>
    /// Given 2 Physics Entities that are known to have overlapping colliders, seperates them by shifting their position
    /// based on the collision normal and penetration depth
    /// </summary>
    /// <param name="PenetrationDepth"> The depth of penetration of the PhysicsEntitites in question </param>
    /// <param name="A"> The first physics Entity </param>
    /// <param name="B"> The second physics Entity </param>
    /// <param name="normal"> The collision normal </param>
    public static void CorrectPosition(float PenetrationDepth, PhysicsEntity A, PhysicsEntity B, Vector normal)
    {
        float percent = 0.6f;
        float tolerance = 0.2f;

        Vector Correction = normal * (Math.Max(PenetrationDepth - tolerance, 0.0f) / ((A.GetInvMass() + B.GetInvMass())) * percent);

        Vector bef = A.position;

        if (Mathf.Abs(Correction.x) > 100 || Mathf.Abs(Correction.y) > 100)
        {
            return;
        }
        A.position.Add(Correction * A.GetInvMass());
        B.position.Add(Correction * -B.GetInvMass());
        bef = bef - A.position;
        
    }
   
    /// <summary>
    /// Given 2 CircleEntities, generates a Manifold containing the information required to resolve a collision (if there was one to begin with).
    /// </summary>
    public static Manifold GenerateManifold(CircleEntity A,CircleEntity B)
    {
        Manifold Result;
        Result.A = A;
        Result.B = B;

        float totalRadius = A.Radius + B.Radius;

        float dx = A.x - B.x;
        float dy = A.y - B.y;

        float distancePowered = (dx * dx) + (dy * dy);

        if (totalRadius * totalRadius <= distancePowered)
        {
            return GenerateEmptyManifold();
        }

        float distance = Mathf.Sqrt(distancePowered);

        Result.PenetrationDepth = totalRadius - distance;

        Vector BtoA = (B.position - A.position).Normalized();

        Vector contactPoint = A.position + BtoA * A.Radius;

        Result.ContactPoint = new List<Vector>();
        Result.ContactPoint.Add(contactPoint);

        Result.Normal = BtoA;

        Result.IsEmpty = false;
        return Result;
    }

    /// <summary>
    /// Given a PolygonEntitiy and a Circle Entity, generates a manifold containing the information required to 
    /// resolve the collision (if there was one to begin with).
    /// </summary>
    public static Manifold GenerateManifold(PolygonEntity poly,CircleEntity circle)
    {
        Manifold Result;
        Result.A = poly;
        Result.B = circle;
        Result.IsEmpty = false;
        Result.ContactPoint = new List<Vector>();
        
        Vector relativeCirclePos = circle.position - poly.position;
        
        float seperation = -10000;
        int face = -1234;

        for (int i = 0; i < poly.GetVertices().Length; i++)
        {
            //get the normal
            Vector normal = poly.GetFaceNormal(i).Normalized();
            Vector circleToPoint = relativeCirclePos - poly.GetLocalSpaceVertices()[i];
            //dot the normal with the circle position
            float currentSeperation = Vector.Dot(normal, circleToPoint);

            //if result of dot is bigger than radius
            if(currentSeperation > circle.Radius)
            {
                return GenerateEmptyManifold();
            }
                //return

            //if its bigger than stored penetration
            if(currentSeperation > seperation)
            {
                face = i;
                seperation = currentSeperation;
            }
               
        }


        Vector v1 = poly.GetVertices()[face];
        int secondIndex = face + 1 >= poly.GetVertices().Length ? 0 : face + 1;
        Vector v2 = poly.GetVertices()[secondIndex];

        //if point is inside polygon
        if(seperation < EPSILON)
        {
            //normal is negative face normal
            Result.Normal = poly.GetFaceNormal(face).Normalized() *-1f;
            Result.PenetrationDepth = circle.Radius;
            Result.ContactPoint.Add(Result.Normal * circle.Radius + circle.position);
            //penetration depth is radius
            //contact point is -normal*radius + position

        }


        //determine which voronoi region the circle lies in
        //dot with first point 
        float v1Dot = Vector.Dot(circle.position - v1, v2 - v1);
        float v2Dot = Vector.Dot(circle.position - v2, v1 - v2);

        Result.PenetrationDepth = circle.Radius - seperation;

        //dot with second point 
        //penetration depth is radius-seperation

        //if dot with first point is negative
        if(v1Dot <= 0)
        {
            if(Mathf.DistanceSquared(v1.x,v1.y,circle.position.x,circle.position.y) > circle.Radius * circle.Radius)
            {
                return GenerateEmptyManifold();
            }
            Result.Normal = (v1 - poly.position).Normalized();
            Result.ContactPoint.Add(v1);
            //normal is point-center
            //contact point is point
        }
        else if(v2Dot <= 0)
        {
            if (Mathf.DistanceSquared(v2.x, v2.y, circle.position.x, circle.position.y) > circle.Radius * circle.Radius)
            {
                return GenerateEmptyManifold();
            }
            Result.Normal = (v2 - poly.position).Normalized();
            Result.ContactPoint.Add(v2);
        }
        else
        {
            Result.Normal = poly.GetFaceNormal(face).Normalized();
            Result.ContactPoint.Add(Result.Normal * circle.Radius + circle.position);
        }

        return Result;
    }

    /// <summary>
    /// Given 2 PolygonEntities generates a Manifold , generates a manifold containing the information required to 
    /// resolve the collision (if there was one to begin with).
    /// </summary>
    public static Manifold GenerateManifold(PolygonEntity A, PolygonEntity B)
    {

        Manifold Result;

        int FaceA = 0;
        int FaceB = 0;
        //find reference face
        float AtBPenetration = CollisionDetection.FindAxisLeastPenetration(A, B, ref FaceA);
        
        if (AtBPenetration > 0)
        {

            return GenerateEmptyManifold();
        }

        float BtAPenetration = CollisionDetection.FindAxisLeastPenetration(B, A, ref FaceB);

        if (BtAPenetration > 0)
        {

            return GenerateEmptyManifold();
        }
       
        
        



        PolygonEntity refPol;
        PolygonEntity IncPol;
        bool flip = false;
        int refFace = -1234;

        if (BiasGreaterThan(AtBPenetration,BtAPenetration))
        {
            refPol = A;
            IncPol = B;
            flip = false;
            refFace = FaceA;
        }
        else
        {
            refPol = B;
            IncPol = A;
            refFace = FaceB;
            flip = true;
        }
     
        if(refFace == -1234)
        {
            Console.WriteLine("ERROR");
        }

        Vector Normal = refPol.GetFaceNormal(refFace).Normalized();

        //SUTHERLAND HODGEMAN
        List<Vector> ClippedPolygon = SutherlandHodgmanClipping(A, B);


        Vector Max = Utils.GetSupportPoint(Normal, ClippedPolygon);
        Vector Min = Utils.GetSupportPoint(Normal * -1, ClippedPolygon);

        Vector PenetrationVector = Max - Min;
        float Penertration = Mathf.Abs(Vector.Dot(PenetrationVector, Normal));


        //Find points in polygon that are in the reference face
        Vector v1 = A.GetVertices()[FaceA];

        int nextIndex = FaceA + 1 >= A.GetVertices().Length ? 0 : FaceA + 1;

        Vector v2 = A.GetVertices()[nextIndex];
        //Console.WriteLine("V1 " + v1);
       // Console.WriteLine("V2 :  " + v2);
        Vector faceDirection = v2 - v1;

        //Console.WriteLine("START ");
        //PointInFace.RemoveRange(2, PointInFace.Count - 2);        

        List<Vector> ContactPoint = new List<Vector>();

        foreach (Vector Point in ClippedPolygon)
        {
            //Console.WriteLine("Point " + Point);
            if (Point.IsEqualTo(v1, 0.01f) || Point.IsEqualTo(v2, 0.01f))
            {
                //Console.WriteLine("Inline v1/v2");
                continue;
            }
            float Distance = DistancePointToLine(v2, v1, Point);
            //Console.WriteLine("DIst " + Distance);
            if(Distance > 0 ) { ContactPoint.Add(Point); }
        }

        
        if(ContactPoint.Count >=2)
        {
            if(ContactPoint[0].IsEqualTo(ContactPoint[1],0.01f))
            {
                ContactPoint.RemoveAt(1);
            }
        }



        Penertration /= ContactPoint.Count;

        if (flip) { Normal = -1 * Normal; }
        
        Result.A = A;
        Result.B = B;
        Result.PenetrationDepth = Penertration;
        Result.Normal =  Normal;
        Result.ContactPoint = ContactPoint;


        //Console.WriteLine("POF C" + ContactPoint.Count);
        Result.IsEmpty = false;
        return Result;

    }

    /// <summary>
    /// Given the penetration depth of 2 colliding PhysicsEntities, 
    /// decides which PhysicsEntity will be used as a reference point for collision detection manifold generation
    /// </summary>
    /// <param name="a"> The resulting depth penetration of the first PhysicsEntity to the second PhysicsEntity </param>
    /// <param name="b"> The resulting depth penetration of the second PhysicsEntity to the first PhysicsEntity</param>
    /// <returns></returns>
    static bool BiasGreaterThan(float aToBPenetration, float bToAPenetration)
    {
        const float k_biasRelative = 0.95f;
        const float k_biasAbsolute = 0.01f;
        return aToBPenetration >= bToAPenetration * k_biasRelative + aToBPenetration * k_biasAbsolute;
    }

    /// <summary>
    /// Finds the polygon generated by the overlap between 2 PolygonEntitites
    /// </summary>
    /// <param name="referencePolygon"> The polygon that will be used as a base for trimming </param>
    /// <param name="polygonToBeTrimmed"> The polygon that will be trimmed s</param>
    /// <returns></returns>
    public static List<Vector> SutherlandHodgmanClipping(PolygonEntity referencePolygon,PolygonEntity polygonToBeTrimmed)
    {
        List<Vector> outputList = polygonToBeTrimmed.GetVerticeList();

        //for each face in reference poly
        for (int i =0;i< referencePolygon.GetVertices().Length;i++)
        {
            //use output from previous iteration as new input
            List<Vector> inputList = new List<Vector>(outputList);

            //start with an empty output list
            outputList.Clear();
            
            int secondIndex = i + 1 >= referencePolygon.GetVertices().Length ? 0 : i + 1;
            //get reflected face normal 
            Vector StartPoint = referencePolygon.GetVertices()[i];

            Vector Endpoint = referencePolygon.GetVertices()[secondIndex];
            
            //for every point in incident poly
            for(int j =0; j < inputList.Count;j++)
            {
                //get v1
                Vector v1 = inputList[j];
                int secondIncIndex = j+ 1 >= inputList.Count ? 0 : j + 1;
                Vector v2 = inputList[secondIncIndex];
                //get v2
                float v1DisToLine = DistancePointToLine(Endpoint, StartPoint, v1);
                float v2DisToLine = DistancePointToLine(Endpoint, StartPoint, v2);

                //entered end point first to get the normal facing towards the polygon
                bool v1IsFront = v1DisToLine > 0;
                bool v2IsFront = v2DisToLine > 0;

                
                //if v1 + v2 in front
                if(v1IsFront && v2IsFront)
                {
                    //save v2
                    outputList.Add(v2);
                    
                }
                //if v1 in front and v2 in the back
                else if (v1IsFront && !v2IsFront)
                {
                    //save intersection
                    outputList.Add(CollisionDetection.FindLineIntersection(StartPoint, Endpoint, v1, v2));
                   
                }
                //if v1 in back and v2 in the front
                else if (!v1IsFront && v2IsFront)
                {
                    //save intersection and v2
                    outputList.Add(CollisionDetection.FindLineIntersection(StartPoint, Endpoint, v1, v2));
                    outputList.Add(v2);
                }
                
                //if both are on the back
                //save none
            }
        }
        return outputList;
    }

    /// <summary>
    /// Given 2 2D Vectors representing a line, finds the distance of that line to the point 
    /// </summary>
    /// <param name="v1"> A 2D vector that lies on the line </param>
    /// <param name="v2"> A 2D vector that lies on the line that is not equal to to 'v1' </param>
    /// <param name="point"> A 2D Vector representing the point we would like to check </param>
    /// <returns></returns>
    public static float DistancePointToLine(Vector v1,Vector v2,Vector point)
    {
        Vector planeDirection = v2 - v1;
        //normal not normalized because a normalized normal is not necessary for the calculation
        Vector planeNormal = planeDirection.GetNormal();

        Vector pointToPlaneStart = point - v1;

        return Vector.Dot(pointToPlaneStart,planeNormal);
    }

    static Manifold GenerateEmptyManifold()
    {
        Manifold Result;
        Result.A = null;
        Result.B = null;
        Result.ContactPoint = new List<Vector>();
        Result.Normal = Vector.Zero();
        Result.PenetrationDepth = 0;
        Result.IsEmpty = true;
        return Result;

    }

    public static float SetFriction(float frictionA, float frictionB)
    {
        return (frictionA + frictionB) / 2f;
    }

    private static float angularFlipperException(PhysicsEntity A)
    {
        if (A is Flipper flipper)
        {
            if (flipper.IsClamped())
            {
                return 0f;
            }

            return A.angularVelocity * flipperMultiplier;
        }
        else
        {
            return A.angularVelocity;
        }
    }


}

