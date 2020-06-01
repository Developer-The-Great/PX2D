using GXPEngine;
using System;

//Handles collision detection
public static class CollisionDetection
{

    /// <summary>
    /// Given 2 circle entities, checks if the circles are overlapping
    /// </summary>
    /// <returns> returns true if CircleA and CircleB are overlapping each other </returns>
    public static bool DetectCollision(CircleEntity CircleA, CircleEntity CircleB)
    {
        float CollisionDistance = CircleA.Radius + CircleB.Radius;

        float dx = CircleA.x - CircleB.x;
        float dy = CircleA.y - CircleB.y;

        return CollisionDistance * CollisionDistance >= (dx * dx) + (dy * dy);
    }

    /// <summary>
    /// Given an AABB Box Entity and a CircleEntity, checks if these objects are overlapping
    /// </summary>
    /// <returns> returns true if 'Circle' and 'Box' are overlapping each other </returns>
    public static bool DetectCollision(CircleEntity Circle,BoxEntity Box)
    {
        Vector Edge = new Vector(Circle.x, Circle.y);
        //if the circle is on the left
        if(Circle.x < Box.Min.x)
        {
            //edge x to check is set to left side
            Edge.x = Box.Min.x;

        }
        //if the circle is on the right
        else if (Circle.x > Box.Max.x)
        {
            //edge x to check is set to the right side
            Edge.x = Box.Max.x;

        }

        //if the circle is above box
        if(Circle.y < Box.Min.y)
        {
            //edge y to check is set on top
            Edge.y = Box.Min.y;
        }
        //if the circle is below
        else if (Circle.y > Box.Max.y)
        {
            //edge y to check is set on the bottom
            Edge.y = Box.Max.y;

        }
        Vector Distance = new Vector(Edge.x - Circle.x, Edge.y - Circle.y);
        float RadiusSquared = Circle.Radius * Circle.Radius;

        Console.WriteLine();
        return Mathf.Pow(Distance.x, 2) + Mathf.Pow(Distance.y, 2) < RadiusSquared;
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="Circle"></param>
    /// <param name="poly"></param>
    /// <returns></returns>
    public static bool DetectCollision(CircleEntity Circle, Vector[] poly)
    {
        Vector Edge = new Vector(Circle.position.x, Circle.position.y);
        float minX = poly[1].x;
        float maxX = poly[0].x;
        float minY = poly[2].y;
        float maxY = poly[1].y;
        //if the circle is on the left
        if (Circle.x < minX)
        {
            //edge x to check is set to left side
            Edge.x = minX;

        }
        //if the circle is on the right
        else if (Circle.x > maxX)
        {
            //edge x to check is set to the right side
            Edge.x = maxX;

        }
        //if the circle is above box
        if (Circle.y < minY)
        {
            //edge y to check is set on top
            Edge.y = minY;
        }
        //if the circle is below
        else if (Circle.y > maxY)
        {
            //edge y to check is set on the bottom
            Edge.y = maxY;

        }


        Vector Distance = new Vector(Edge.x - Circle.x, Edge.y - Circle.y);
        float RadiusSquared = Circle.Radius * Circle.Radius;
        MyGame.Points.Add(new Vector(Edge.x, Edge.y));

        return Mathf.Pow(Distance.x, 2) + Mathf.Pow(Distance.y, 2) < RadiusSquared;
    }

    /// <summary>
    /// Given 2 AABB colliders of type BoxEntity, checks if these objects are overlapping
    /// </summary>
    /// <returns></returns>
    public static bool DetectCollision(BoxEntity A,BoxEntity B)
    {
        if (A.Max.x < B.Min.x || A.Min.x > B.Max.x) { return false; }
        if(A.Max.y < B.Min.y || A.Min.y > B.Max.y) { return false; }

        return true;
    }

    /// <summary>
    /// Given 2 PolygonEntities, checks if their bounding volumes are colliding
    /// </summary>
    /// <returns></returns>
    public static bool DetectBoundingVolumeCollision(PolygonEntity A,PolygonEntity B)
    {
        if(Utils.IsNearEqual(A.GetInvMass(),0,0.0001f) && Utils.IsNearEqual(B.GetInvMass(), 0, 0.0001f)) { return false; }

        float CollisionDistance = A.GetVolumeRadius() + B.GetVolumeRadius();

        float dx = A.x - B.x;
        float dy = A.y - B.y;

        return CollisionDistance * CollisionDistance >= (dx * dx) + (dy * dy);
       
    }

    /// <summary>
    /// Given a CircleEntity and a PolygonEntity, checks if their bounding volumes overlap each other
    /// </summary>
    /// <returns></returns>
    public static bool DetectBoundingVolumeCollision(CircleEntity A, PolygonEntity B)
    {
        float CollisionDistance = A.Radius + B.GetVolumeRadius();

        float dx = A.x - B.x;
        float dy = A.y - B.y;

        return CollisionDistance * CollisionDistance >= (dx * dx) + (dy * dy);

    }

    /// <summary>
    /// Given a CircleEntity and the vertices and position of a 2D Polygon, checks if the circle and the polygon are overlapping
    /// </summary>
    /// <param name="circle"> The circle entity in question </param>
    /// <param name="polyPosition"> The position of the polygon in question </param>
    /// <param name="poly"> The world space position of the vertices of the polygon </param>
    /// <param name="localPoly"> The local space position of the vertices of the polygon </param>
    /// <returns></returns>
    public static bool DetectCollision( CircleEntity circle, Vector polyPosition, Vector[] poly,Vector[] localPoly)
    {
        Vector relativeCirclePos = circle.position - polyPosition;
        float seperation = -10000;
        int face = -1234;
        //Console.WriteLine("MOVEMNT TICK");
        for (int i = 0; i < poly.Length; i++)
        {
            //get the normal
            Vector normal = poly[i].GetNormal().Normalized();
            

            Vector circleToPoint = relativeCirclePos - localPoly[i];
            //dot the normal with the circle position
            float currentSeperation = Vector.Dot(normal, circleToPoint);

            //if result of dot is bigger than radius
            if (currentSeperation >= circle.Radius)
            {
                //Console.WriteLine("NO COL");
                return false;
            }
            //return

            //if its bigger than stored penetration
            if (currentSeperation > seperation)
            {
                //store current face
                //store current penetration
                face = i;
                seperation = currentSeperation;
            }

        }

        Vector v1 = poly[face];
        int secondIndex = face + 1 >= poly.Length ? 0 : face + 1;
        Vector v2 = poly[secondIndex];

        float v1Dot = Vector.Dot(circle.position - v1, v2 - v1);
        float v2Dot = Vector.Dot(circle.position - v2, v1 - v2);

        if (v1Dot <= 0)
        {
            if (Mathf.DistanceSquared(v1.x, v1.y, circle.position.x, circle.position.y) > circle.Radius * circle.Radius)
            {
                //Console.WriteLine("NO COL");
                return false;
            }

        }
        else if (v2Dot <= 0)
        {
            if (Mathf.DistanceSquared(v2.x, v2.y, circle.position.x, circle.position.y) > circle.Radius * circle.Radius)
            {
                //Console.WriteLine("NO COL");
                return false;
            }

        }
        return true;
    }

    /// <summary>
    /// Given 2 PolygonEntitites, finds the edge with the least penetration 
    /// </summary>
    /// <param name="A"></param>
    /// <param name="B"></param>
    /// <param name="PolygonIndex"> An out parameter giving the index where the Axis of least penetration was found </param>
    /// <returns>returns the value of the axis of least penetration </returns>
    public static float FindAxisLeastPenetration(PolygonEntity A, PolygonEntity B, ref int PolygonIndex)
    {
        float greatestProjection = -100000;
        int faceIndex = 0; 
        for(int i = 0;i < A.GetVertices().Length;i++)
        {
            Vector Normal = A.GetFaceNormal(i).Normalized();
            Vector verticePosition = A.GetVertices()[i];

            Vector verticeIndex = B.GetSupportPoint(Normal * -1);
            verticeIndex += B.position;

            Vector DistanceToVertice = verticeIndex - verticePosition;

            float projection = Vector.Dot(DistanceToVertice, Normal );

            if(projection > greatestProjection)
            {
                greatestProjection = projection;
                faceIndex = i;
            }
        }
        PolygonIndex = faceIndex;
        return greatestProjection;
    }


    /// <summary>
    /// Given the vertices of 2 Polygons, finds the axis of least penetration 
    /// </summary>
    /// <param name="A"></param>
    /// <param name="B"></param>
    /// <param name="PolygonIndex"> An out parameter giving the index where the Axis of least penetration was found </param>
    /// <returns></returns>
    public static float FindAxisLeastPenetration(Vector[] A, Vector[] B, ref int PolygonIndex)
    {
        float greatestProjection = -100000;
        int faceIndex = 0;
        for (int i = 0; i < A.Length; i++)
        {
            Vector Normal = GetArrayFaceNormal(i,A).Normalized();
            Vector verticePosition = A[i];

            Vector verticeIndex = Utils.GetSupportPoint(Normal * -1,B);
            //verticeIndex += B.position;

            Vector DistanceToVertice = verticeIndex - verticePosition;

            float projection = Vector.Dot(DistanceToVertice, Normal);

            if (projection > greatestProjection)
            {
                greatestProjection = projection;
                faceIndex = i;
            }
        }
        PolygonIndex = faceIndex;
        return greatestProjection;
    }

    /// <summary>
    /// Given the index of an array of vertices representing a closed polygon, finds the normal of the edge
    /// </summary>
    /// <param name="index"></param>
    /// <param name="array"></param>
    /// <returns></returns>
    private static Vector GetArrayFaceNormal(int index,Vector[] array )
    {
        Vector v1 = array[index];

        int secondIndex = index + 1 >= array.Length ? 0 : index + 1;
        Vector v2 = array[secondIndex];


        return (v2 - v1).GetNormal();

    }

    /// <summary>
    /// Given a 2D Point in space, Checks if the point is inside the AABB Collider of type  BoxEntitiy
    /// </summary>
    /// <param name="Point"></param>
    /// <param name="Box"></param>
    /// <returns></returns>
    public static bool PointVBox(Vector Point,BoxEntity Box)
    {
        if (Point.x < Box.Min.x || Point.x > Box.Max.x) { return false; }
        if (Point.y < Box.Min.y || Point.y > Box.Max.y) { return false; }

        return true;
    }

    /// <summary>
    /// Given a 2D point in space checks if it is inside the given box represented by its minumum and maximum vertices
    /// </summary>
    /// <param name="Point"></param>
    /// <param name="min"></param>
    /// <param name="max"></param>
    /// <returns></returns>
    public static bool PointVBox(Vector Point, Vector min,Vector max)
    {
        if (Point.x < min.x || Point.x > max.x)
        {
            Console.WriteLine("NOT IN X");
            return false;
        }
        if (Point.y < min.y || Point.y > max.y)
        {
            Console.WriteLine("NOT IN Y");
            return false;
        }

        return true;
    }

    /// <summary>
    /// Given the start and end of 2 2D lines, finds the intersection point of these 2 lines
    /// </summary>
    /// <param name="start1"> The start of the first line </param>
    /// <param name="end1"> The end of the first line </param>
    /// <param name="start2"> The start of the second line </param>
    /// <param name="end2"> The end of the second line </param>
    /// <returns></returns>
    public static Vector FindLineIntersection(Vector start1, Vector end1, Vector start2, Vector end2)
    {
        Vector a = end1 - start1;

        float tNumerator = (start1.x - start2.x) * (start2.y - end2.y) - (start1.y - start2.y) * (start2.x - end2.x);
        float tDenominator = (start1.x - end1.x) * (start2.y - end2.y) - (start1.y - end1.y) * (start2.x - end2.x);

        float t = tNumerator / tDenominator;

        return a * t + start1;

    }

}

    
        

