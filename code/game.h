/* date = July 26th 2021 7:49 pm */

#ifndef GAME_H
#define GAME_H


#include "game_platform.h"
#include <math.h>

#define internal static
#define local_persist static
#define global_variable static

#define Pi32 3.14159265359f
#if GAME_SLOW
// TODO(casey): complete assertion macro
#define Assert(Expression) if(!(Expression)) {*(int*)0 = 0;}
#else
#define Assert(Expression)
#endif

#define Kilobytes(Value) ((Value)*1024LL)
#define Megabytes(Value) (Kilobytes(Value)*1024LL)
#define Gigabytes(Value) (Megabytes(Value)*1024LL)
#define Terabytes(Value) (Gigabytes(Value)*1024LL)

#define ArrayCount(Array) (sizeof(Array) / sizeof((Array)[0]))

// TODO(casey): swap, min, max ... macros?

inline game_controller_input *GetController(game_input *Input, int unsigned ControllerIndex)
{
    Assert(ControllerIndex < ArrayCount(Input->Controllers));
    
    game_controller_input *Result = &Input->Controllers[ControllerIndex];
    return(Result);
}


struct v3
{
    union
    {
        struct
        {
            f32 x;
            f32 y;
            f32 z;
        };
        
        f32 V[3];
    };
    
    f32 operator*(v3 Operand)// const v3& Operand)  // dot product
    {
        return (this->x * Operand[0] + 
                this->y * Operand[1] + 
                this->z * Operand[2]);
    }
    
    v3 operator-(v3 Operand)
    {
        v3 Result = {};
        Result[0] = this->x - Operand[0];
        Result[1] = this->y - Operand[1];
        Result[2] = this->z - Operand[2];
        return Result;
    }
    
    v3 operator+(v3 Operand)
    {
        v3 Result = {};
        Result[0] = this->x + Operand[0];
        Result[1] = this->y + Operand[1];
        Result[2] = this->z + Operand[2];
        return Result;
    }
    
    v3 operator/(f32 Divisor)
    {
        v3 Result = {};
        Result[0] = this->x / Divisor;
        Result[1] = this->y / Divisor;
        Result[2] = this->z / Divisor;
        return Result;
    }
    
    f32& operator[](int index)
    {
        return this->V[index];
    }
};

v3
operator*(f32 Scalar, v3 Vector)
{
    return {
        Vector[0]*Scalar,
        Vector[1]*Scalar,
        Vector[2]*Scalar};
}

v3
Cross(v3 A, v3 B)
{
    v3 Result = {};
    Result.x = A.y*B.z - A.z*B.y;
    Result.y = A.z*B.x - A.x*B.z;
    Result.z = A.x*B.y - A.y*B.x;
    return Result;
}

inline f32
LSquaredV3(v3 V)
{
    f32 l_squared = V[0]*V[0] +
        V[1]*V[1] +
        V[2]*V[2];
    return l_squared;
}

inline v3
NormalizeV3(v3 V)
{
    f32 l_squared = LSquaredV3(V);
    if (l_squared > 0)
    {
        f32 l = sqrt(l_squared);
        
        return {V[0]/l,
            V[1]/l,
            V[2]/l};
    }
    else return {0, 0, 0};
}

struct matrix3
{
    union
    {
        struct
        {
            v3 V1;
            v3 V2;
            v3 V3;
        };
        
        v3 V[3];
    };
    v3& operator[](int index)
    {
        return this->V[index];
    }
    
    matrix3 operator*(matrix3 B)
    {
        matrix3 Result = {};
        matrix3* A = this;
        
        for (int X = 0;
             X < 3;
             ++X)
        {
            for (int Y = 0;
                 Y < 3;
                 ++Y)
            {
                f32 Sum = 0;
                for (int VecIndex = 0;
                     VecIndex < 3;
                     ++VecIndex)
                {
                    Sum = Sum + (*A)[X][VecIndex] * B[VecIndex][Y];
                }
                
                Result[X][Y] = Sum;
            }
        }
        return Result;
    }
    v3 operator*(v3 B)
    {
        v3 Result = {};
        matrix3* A = this;
        
        Result.x = (*A)[0][0]*B.x + (*A)[0][1]*B.y + (*A)[0][2]*B.z;
        Result.y = (*A)[1][0]*B.x + (*A)[1][1]*B.y + (*A)[1][2]*B.z;
        Result.z = (*A)[2][0]*B.x + (*A)[2][1]*B.y + (*A)[2][2]*B.z;
        
        return Result;
    }
};

struct v2
{
    union
    {
        struct
        {
            f32 x;
            f32 y;
        };
        f32 V[2];
    };
    
    f32& operator[](int index)
    {
        return this->V[index];
    }
    
    v2 operator-(v2 Operand)
    {
        v2 Result = {};
        Result[0] = this->x - Operand[0];
        Result[1] = this->y - Operand[1];
        return Result;
    }
};

inline v3
WedgeVectors(v3 u, v3 v)
{
    v3 Result = {
        u[0]*v[1] - u[1]*v[0],
        u[0]*v[2] - u[2]*v[0],
        u[1]*v[2] - u[2]*v[1]};
    return Result;
}

struct rotor3
{
    union
    {
        struct
        {
            f32 Scalar;
            v3 Bivector;
        };
        
        f32 R[4];
        
        struct
        {
            f32 w;
            f32 x;
            f32 y;
            f32 z;
        };
    };
    
    f32& 
        operator[](int index)
    {
        return this->R[index];
    }
    
    // Equivalent to quaternion conjugate
    rotor3
        operator-()
    {
        return {this->R[0], -this->R[1], -this->R[2], -this->R[3]};
    }
    
    rotor3
        operator*(f32 s)
    {
        return {s*(*this)[0], s*(*this)[1], s*(*this)[2], s*(*this)[3]};
    }
    
    // Rotor3-Rotor3 product
    rotor3
        operator*(rotor3 q2)
    {
        f32 A, B, C, D, E, F, G, H;
        rotor3* q1 = this;
        
        A = (q1->w + q1->x)*(q2.w + q2.x);
        B = (q1->z - q1->y)*(q2.y - q2.z);
        C = (q1->w - q1->x)*(q2.y + q2.z); 
        D = (q1->y + q1->z)*(q2.w - q2.x);
        E = (q1->x + q1->z)*(q2.x + q2.y);
        F = (q1->x - q1->z)*(q2.x - q2.y);
        G = (q1->w + q1->y)*(q2.w - q2.z);
        H = (q1->w - q1->y)*(q2.w + q2.z);
        
        rotor3 res;
        res.w = B + (-E - F + G + H) /2;
        res.x = A - (E + F + G + H)/2; 
        res.y = C + (E - F + G - H)/2; 
        res.z = D + (E - F - G + H)/2;
        return res;
    }
};

inline rotor3
NormalizeRotor(rotor3 Rotor)
{
    f32 l_squared = Rotor[0]*Rotor[0] +
        Rotor[1]*Rotor[1] +
        Rotor[2]*Rotor[2] +
        Rotor[3]*Rotor[3];
    f32 l = sqrt(l_squared);
    
    return {Rotor[0]/l,
        Rotor[1]/l,
        Rotor[2]/l,
        Rotor[3]/l};
}

inline rotor3
RotorFromVectors(v3 From, v3 To)
{
    rotor3 Result;
    Result.Scalar = 1+(To*From);
    Result.Bivector = WedgeVectors(To, From);
    return NormalizeRotor(Result);
}

inline rotor3
AxisAngleToRotor(v3 axis, f32 theta)
{
    f32 sina = sin(theta*0.5f);
    rotor3 R = {
        cos(theta*0.5f),
        axis.x * sina,
        axis.y * sina,
        axis.z * sina,
    };
    return R;
}

#if 0
void Camera::SetViewByMouse(void) { 
    // the coordinates of our mouse coordinates 
    int MouseX, MouseY; 
    // the middle of the screen in the x direction
    int MiddleX = SCREENWIDTH/2; 
    // the middle of the screen in the y direction
    int MiddleY = SCREENHEIGHT/2; 
    // vector that describes mouseposition - center
    Vector MouseDirection(0, 0, 0); 
    // static variable to store the rotation about the x-axis, since 
    // we want to limit how far up or down we can look. 
    // We don't need to cap the rotation about the y-axis as we 
    // want to be able to turn around 360 degrees 
    static double CurrentRotationAboutX = 0.0; 
    // The maximum angle we can look up or down, in radians 
    double maxAngle = 1; 
    // This function gets the position of the mouse 
    SDL_GetMouseState(&MouseX, &MouseY); 
    // if the mouse hasn't moved, return without doing 
    // anything to our view 
    if((MouseX == MiddleX) && (MouseY == MiddleY)) return; 
    // otherwise move the mouse back to the middle of the screen 
    SDL_WarpMouse(MiddleX, MiddleY); 
    // get the distance and direction the mouse moved in x (in 
    // pixels). We can't use the actual number of pixels in radians, 
    // as only six pixels would cause a full 360 degree rotation. 
    // So we use a mousesensitivity variable that can be changed to 
    // vary how many radians we want to turn in the x-direction for 
    // a given mouse movement distance 
    // We have to remember that positive rotation is counter-clockwise. 
    // Moving the mouse down is a negative rotation about the x axis
    // Moving the mouse right is a negative rotation about the y axis 
    MouseDirection.x = (MiddleX - MouseX)/MouseSensitivity; 
    MouseDirection.y = (MiddleY - MouseY)/MouseSensitivity; 
    CurrentRotationX += MouseDirection.y; 
    // We don't want to rotate up more than one radian, so we cap it. 
    if(CurrentRotationX > 1) { CurrentRotationX = 1; return; } 
    // We don't want to rotate down more than one radian, so we cap it. 
    if(CurrentRotationX < -1) { CurrentRotationX = -1; return; } 
    else { 
        // get the axis to rotate around the x-axis. 
        Vector Axis = CrossProduct(View - Position, Up); 
        // To be able to use the quaternion conjugate, the axis to 
        // rotate around must be normalized. 
        Axis = Normalize(Axis); 
        // Rotate around the y axis 
        RotateCamera(MouseDirection.y, Axis.x, Axis.y, Axis.z);
        // Rotate around the x axis 
        RotateCamera(MouseDirection.x, 0, 1, 0); 
    } 
}
#endif

// non-optimized
v3 
RotateWithRotor( v3 x, rotor3 p )
{
	// q = P (x,0)
	rotor3 q;
	q[1] = p[0] * x[0] + p[2] * x[2] - p[3] * x[1];
	q[2] = p[0] * x[1] + p[3] * x[0] - p[1] * x[2];
	q[3] = p[0] * x[2] + p[1] * x[1] - p[2] * x[0];
	
	q[0] = - p[1] * x[0] - p[2] * x[1] - p[3] * x[2];
    
    // r = q P*
    v3 r;
    r[0] = q[0] * -p[1] + p[0] * q[1]  -  q[2] * p[3] + q[3] * p[2];
    r[1] = q[0] * -p[2] + p[0] * q[2]  -  q[3] * p[1] + q[1] * p[3];
    r[2] = q[0] * -p[3] + p[0] * q[3]  -  q[1] * p[2] + q[2] * p[1];
    
    
    /*// q = P V
    v3 q;
    q[0] = R[0] * V[0] + V[1] * R[1] + V[2] * R[2];
    q[1] = R[0] * V[1] - V[0] * R[1] + V[2] * R[3];
    q[2] = R[0] * V[2] - V[0] * R[2] - V[1] * R[3];
        
    f32 Trivector = V[0] * R[3] - V[1] * R[2] + V[2] * R[1]; // trivector
        
    // r = q P*
    v3 Result;
    Result[0] = R[0] * q[0] + q[1] * R[1] + q[2] * R[2]    + Trivector * R[3];
    Result[1] = R[0] * q[1] - q[0] * R[1] - Trivector * R[2]    + q[2] * R[3];
            Result[2] = R[0] * q[2] + Trivector * R[1] - q[0] * R[2]    - q[1] * R[3];
        
    */// trivector part of the result is always zero!
    
    return r;
}

matrix3
RotorToMatrix(rotor3 q)
{
    matrix3 m = {};
    
    f32 sqw = q[0]*q[0];
    f32 sqx = q[1]*q[1];
    f32 sqy = q[2]*q[2];
    f32 sqz = q[3]*q[3];
    
    // invs (inverse square length) is only required if quaternion is not already normalised
    f32 invs = 1 / (sqx + sqy + sqz + sqw);
    m[0][0] = ( sqx - sqy - sqz + sqw)*invs ; // since sqw + sqx + sqy + sqz =1/invs*invs
    m[1][1] = (-sqx + sqy - sqz + sqw)*invs ;
    m[2][2] = (-sqx - sqy + sqz + sqw)*invs ;
    
    f32 tmp1 = q[1]*q[2];
    f32 tmp2 = q[3]*q[0];
    m[1][0] = 2.0 * (tmp1 + tmp2)*invs ;
    m[0][1] = 2.0 * (tmp1 - tmp2)*invs ;
    
    tmp1 = q[1]*q[3];
    tmp2 = q[2]*q[0];
    m[2][0] = 2.0 * (tmp1 - tmp2)*invs ;
    m[0][2] = 2.0 * (tmp1 + tmp2)*invs ;
    tmp1 = q[2]*q[3];
    tmp2 = q[1]*q[0];
    m[2][1] = 2.0 * (tmp1 + tmp2)*invs ;
    m[1][2] = 2.0 * (tmp1 - tmp2)*invs ;      
    return m;
}

inline rotor3
MatrixToRotor( matrix3 a) 
{
    rotor3 q = {};
    f32 trace = a[0][0] + a[1][1] + a[2][2]; // I removed + 1.0f; see discussion with Ethan
    if( trace > 0 ) {// I changed M_EPSILON to 0
        f32 s = 0.5f / sqrtf(trace+ 1.0f);
        q[0] = 0.25f / s;
        q[1] = ( a[2][1] - a[1][2] ) * s;
        q[2] = ( a[0][2] - a[2][0] ) * s;
        q[3] = ( a[1][0] - a[0][1] ) * s;
    } else {
        if ( a[0][0] > a[1][1] && a[0][0] > a[2][2] ) {
            float s = 2.0f * sqrtf( 1.0f + a[0][0] - a[1][1] - a[2][2]);
            q[0] = (a[2][1] - a[1][2] ) / s;
            q[1] = 0.25f * s;
            q[2] = (a[0][1] + a[1][0] ) / s;
            q[3] = (a[0][2] + a[2][0] ) / s;
        } else if (a[1][1] > a[2][2]) {
            float s = 2.0f * sqrtf( 1.0f + a[1][1] - a[0][0] - a[2][2]);
            q[0] = (a[0][2] - a[2][0] ) / s;
            q[1] = (a[0][1] + a[1][0] ) / s;
            q[2] = 0.25f * s;
            q[3] = (a[1][2] + a[2][1] ) / s;
        } else {
            float s = 2.0f * sqrtf( 1.0f + a[2][2] - a[0][0] - a[1][1] );
            q[0] = (a[1][0] - a[0][1] ) / s;
            q[1] = (a[0][2] + a[2][0] ) / s;
            q[2] = (a[1][2] + a[2][1] ) / s;
            q[3] = 0.25f * s;
        }
    }
    return q;
}


// rotate a rotor by another
inline rotor3 
RotateRotor( rotor3 R, rotor3 Rotation )
{
    // should unwrap this for efficiency
    return NormalizeRotor((Rotation) * R * (-Rotation));
}

inline rotor3
EulerToRotor(f32 y, f32 z, f32 x)
{
    f32 c1, c2, c3, s1, s2, s3, c1c2, s1s2;
    // calculate trig identities
    c1 = cos(y*0.5);
    s1 = sin(y*0.5);
    c2 = cos(z*0.5);
    s2 = sin(z*0.5);
    c3 = cos(x*0.5);
    s3 = sin(x*0.5);
    c1c2 = c1*c2;
    s1s2 = s1*s2;
    
    rotor3 R;
    R.w =c1c2*c3 - s1s2*s3;
    R.x =c1c2*s3 + s1s2*c3;
    R.y =s1*c2*c3 + c1*s2*s3;
    R.z =c1*s2*c3 - s1*c2*s3;
    return R;
}

// try again later
//inline rotor3
//TaitBryanToRotor(f32 x, f32 z, rotor3 R)
//{
//
//rotor3 X;
//X.w = cos(x/2);
//X.x = sin(x/2);
//
//rotor3 Z;
//Z.w = cos(z/2);
//Z.x = sin(z/2)*cos();
//Z.y = sin(z/2)*cos(x);
//Z.z = sin(z/2)*cos(x);
//}
//

#if 1
struct euler_angles
{
    f32 yaw, pitch, roll;
};

internal f32
clamp(f32 value, f32 lower, f32 upper)
{
    return value < lower ? lower : (value > upper ? upper : value);
}

internal euler_angles
MatrixToEuler(matrix3 M) //YZX
{
    euler_angles r = {};
    r.yaw = asin(clamp(M[1][0], -1, 1));
    if (abs(M[1][0] < 0.99999999f))
    {
        r.roll = atan2(-M[1][2], M[1][1]);
        r.pitch = atan2(-M[2][0], M[0][0]);
    }
    else
    {
        r.roll = 0;
        r.pitch = atan2(M[0][2], M[2][2]);
    }
    return r;
}

inline euler_angles
RotorToEuler(rotor3 R) //YZX
{ 
    return MatrixToEuler(RotorToMatrix(R));
}
#endif

struct sphere
{
    v3 Position;
    f32 Radius;
};

struct game_camera
{
    v3 Position;
    f32 FocalDistance;
    union
    {
        struct{
            f32 Width;
            f32 Height;
        };
        v2 Dimension;
    };
    rotor3 Rotation;
};

struct game_state
{
    v3 PlayerPosition;
    v3 PlayerVelocity;
    v3 PlayerCameraOffset;
    game_camera PlayerCamera;
    game_camera DebugCamera;
    matrix3* Triangles;
    int TriangleCount;
    matrix3* Walkables;
    int WalkableCount;
    void* ZBuffer;
};

#endif //GAME_H
