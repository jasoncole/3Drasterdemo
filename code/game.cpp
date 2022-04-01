#include <cmath>
#include "game.h"
#include <stdio.h>

/* TODO(jason):
PLATFORM CODE
load textures from disc
- memory arena
sound
threading
hardware accellerated graphics (OpenGL)
bring framerate down to <33ms/frame

 "ENGINE" CODE
shading
textures
normal map

DEBUG
add debug infrastructure

GAME LOGIC
entity system

BUG LIST
rotation still rolling, pitch working fine (still needs to be clamped)
movement does not rotate with camera
clipping floating point error
- added .01f to clipping plane, seems fine for now
 double edge fix for transparent surfaces
- https://www.scratchapixel.com/lessons/3d-basic-rendering/rasterization-practical-implementation/rasterization-stage

*/

internal void
DEBUGDrawRectangle(game_offscreen_buffer* Buffer, v2 ScreenCoords)
{
    u32* EndOfBuffer = (u32*)Buffer->Memory + Buffer->Pitch*Buffer->Height;
    int PixelX = (int)(ScreenCoords[0]*(f32)Buffer->Width + 0.5f);
    int PixelY = (int)(ScreenCoords[1]*(f32)Buffer->Height + 0.5f);
    
    u32 Color = 0xFFFFFFFF;
    int RectSize = 10;
    
    int MinX = PixelX - RectSize/2;
    int MaxX = PixelX + RectSize/2;
    int MinY = PixelY - RectSize/2;
    int MaxY = PixelY + RectSize/2;
    
    if (MinX < 0) MinX = 0;
    if (MinY < 0) MinY = 0;
    if (MaxX > Buffer->Width) MaxX = Buffer->Width;
    if (MaxY > Buffer->Height) MaxY = Buffer->Height;
    
    u8* Row = ((u8*)Buffer->Memory +
               MinX*Buffer->BytesPerPixel +
               MinY*Buffer->Pitch);
    
    for (int Y = MinY;
         Y < MaxY;
         ++Y)
    {
        u32* Pixel = (u32*)Row;
        for(int X = MinX;
            X < MaxX;
            ++X)
        {
            *Pixel++ = Color;
        }
        Row += Buffer->Pitch;
    }
}

inline internal v2
CameraSpaceToScreenCoords(v3 CameraVert, f32 FocalDistance, v2 CameraDimension)
{
    v2 Result; 
    Result[0] = CameraVert[0]*(FocalDistance/CameraVert[2]);
    Result[0] = (Result[0] + CameraDimension[0]/2) / CameraDimension[0];
    Result[1] = CameraVert[1]*(FocalDistance/CameraVert[2]);
    Result[1] = (Result[1] + CameraDimension[1]/2) / CameraDimension[1];
    return Result;
}

internal b32 
InsideEdge(v2 V1, v2 V2, v2 P) 
{ 
    return ((P.x - V1.x) * (V2.y - V1.y) - (P.y - V1.y) * (V2.x - V1.x) >= 0); 
} 

internal b32 
InsideTriangle(v2 V1, v2 V2, v2 V3, v2 P)
{
    b32 Result = true;
    Result &= InsideEdge(V1, V2, P);
    Result &= InsideEdge(V2, V3, P);
    Result &= InsideEdge(V3, V1, P);
    return Result;
}

internal f32
Determinant( v2 A, v2 B )
{
    return (A.x*B.y - A.y*B.x);
}

internal u32
VecColorToHex(v3 Color)
{
    u32 Result = 0xFF000000;
    Result = Result | (int)(Color[0]*(f32)0xFF) << 16;
    Result = Result | (int)(Color[1]*(f32)0xFF) << 8;
    Result = Result | (int)(Color[2]*(f32)0xFF);
    return Result;
}

internal f32
f32Min(f32* arr, int size)
{
    f32 Result = arr[0];
    for (int FloatIndex = 1;
         FloatIndex < size;
         ++FloatIndex)
    {
        Result = (arr[FloatIndex] < Result) ? arr[FloatIndex] : Result;
    }
    return Result;
}

internal f32
f32Max(f32* arr, int size)
{
    f32 Result = arr[0];
    for (int FloatIndex = 1;
         FloatIndex < size;
         ++FloatIndex)
    {
        Result = (arr[FloatIndex] > Result) ? arr[FloatIndex] : Result;
    }
    return Result;
}

internal void 
PoleTriangleIntersection(v3* Intersections, int* IntersectionSize, matrix3* Walkables, int WalkableCount,
                         v3 PolePos, v3 PoleProjection)
{
    
#define EPSILON 0.0000001
    for (int WalkableIndex = 0;
         WalkableIndex < WalkableCount;
         ++WalkableIndex)
    {
        // Muller Trumbore triangle intersection
        // TODO(jason): smarter implementation with lines
        
        matrix3 WalkableTriangle = Walkables[WalkableIndex];
        
        v3 edge1 = WalkableTriangle[1] - WalkableTriangle[0];
        v3 edge2 = WalkableTriangle[2] - WalkableTriangle[0];
        
        v3 pvec = Cross(PoleProjection, edge2);
        f32 det = edge1 * pvec;
        
        if ((det > -EPSILON) && (det < EPSILON)) continue;
        
        f32 invDet = 1.0f / det; 
        
        v3 tvec = PolePos - WalkableTriangle[0]; 
        f32 u = invDet * (tvec * pvec); 
        if (u < 0.0f|| u > 1.0f) continue; 
        
        v3 qvec = Cross(tvec, edge1); 
        f32 v = invDet * (PoleProjection * qvec) ; 
        if (v < 0.0f || u + v > 1.0f) continue; 
        
        f32 t = invDet * (edge2 * qvec); 
        
        if (t > EPSILON)
        {
            Intersections[(*IntersectionSize)++] = PolePos + t * PoleProjection;
        }
    }
}

void
RasterizeTriangle(matrix3 TrianglePos, matrix3 TriangleColor, game_camera* Camera, game_offscreen_buffer* Buffer, f32 NearClippingPlane)
{
    // NOTE(jason): RasterizeTrianglePos takes Camera Coordinates, NOT world coordinates
    // check clip plane
    u16 InsidePlane = ((TrianglePos[0][2] >= NearClippingPlane) |
                       ((TrianglePos[1][2] >= NearClippingPlane) << 1) |
                       ((TrianglePos[2][2] >= NearClippingPlane) << 2));
    
    switch(InsidePlane)
    {
        case 7:
        {
            v2 ScreenV0 = CameraSpaceToScreenCoords(TrianglePos[0], Camera->FocalDistance, Camera->Dimension);
            v2 ScreenV1 = CameraSpaceToScreenCoords(TrianglePos[1], Camera->FocalDistance, Camera->Dimension);
            v2 ScreenV2 = CameraSpaceToScreenCoords(TrianglePos[2], Camera->FocalDistance, Camera->Dimension);
            
            DEBUGDrawRectangle(Buffer, ScreenV0);
            DEBUGDrawRectangle(Buffer, ScreenV1);
            DEBUGDrawRectangle(Buffer, ScreenV2);
            
            // BoundingRect
            f32 ScreenXCoords[3] = {ScreenV0.x, ScreenV1.x, ScreenV2.x};
            f32 ScreenYCoords[3] = {ScreenV0.y, ScreenV1.y, ScreenV2.y};
            f32 MinX = f32Min(ScreenXCoords, 3);
            f32 MaxX = f32Max(ScreenXCoords, 3);
            f32 MinY = f32Min(ScreenYCoords, 3);
            f32 MaxY = f32Max(ScreenYCoords, 3);
            
            if (MinX < 0.0f) MinX = 0.0f;
            if (MinY < 0.0f) MinY = 0.0f;
            if (MaxX > 1.0f) MaxX = 1.0f;
            if (MaxY > 1.0f) MaxY = 1.0f;
            
            f32 ZV1 = (TrianglePos[0][2]);
            f32 ZV2 = (TrianglePos[1][2]);
            f32 ZV3 = (TrianglePos[2][2]);
            
            v3 SurfaceNormal = NormalizeV3(Cross(TrianglePos[0]-TrianglePos[2], TrianglePos[1]-TrianglePos[2]));
            f32 AreaPGram = Determinant(ScreenV0-ScreenV2, ScreenV1-ScreenV2);
            
            // get source color from texture and filter (bilinear vs other options...)
            // get destination color from world state
            // blend source color and destination color 
            
            //Min/Max is in NDC, convert to pixel coordinates
            int PixelMinX = MinX*Buffer->Width + 0.5f;
            int PixelMaxX = MaxX*Buffer->Width + 0.5f;
            int PixelMinY = MinY*Buffer->Height + 0.5f;
            int PixelMaxY = MaxY*Buffer->Height + 0.5f;
            
            u8* Row = ((u8*)Buffer->Memory +
                       PixelMinX*Buffer->BytesPerPixel +
                       PixelMinY*Buffer->Pitch);
            u8* ZRow = ((u8*)Buffer->ZBuffer +
                        PixelMinX*Buffer->BytesPerPixel +
                        PixelMinY*Buffer->Pitch);
            for (int Y = PixelMinY;
                 Y< PixelMaxY;
                 ++Y)
            {
                u32*Pixel = (u32*)Row;
                u32*ZPixel = (u32*)ZRow;
                for (int X = PixelMinX;
                     X< PixelMaxX;
                     ++X)
                {
                    f32 NormX = (f32)X/(f32)Buffer->Width;
                    f32 NormY = (f32)Y/(f32)Buffer->Height;
                    v2 P = {NormX, NormY};
                    
                    f32 Lambda3 = Determinant(ScreenV0-P,ScreenV1-P); // these are parallelograms
                    f32 Lambda1 = Determinant(ScreenV1-P,ScreenV2-P); // div 2 for triangle
                    f32 Lambda2 = Determinant(ScreenV2-P,ScreenV0-P);
                    
                    if ((Lambda1 >= 0) && (Lambda2 >= 0) && (Lambda3 >= 0)) // if inside triangle
                    {
                        Lambda1 /= AreaPGram;
                        Lambda2 /= AreaPGram;
                        Lambda3 /= AreaPGram;
                        
                        f32 ZValue = 1.0f/((1.0f/ZV1)*Lambda1 +
                                           (1.0f/ZV2)*Lambda2 + 
                                           (1.0f/ZV3)*Lambda3);
                        v3 Color = ZValue*(Lambda1*(TriangleColor[0]/ZV1)+ 
                                           Lambda2*(TriangleColor[1]/ZV2)+ 
                                           Lambda3*(TriangleColor[2]/ZV3));
                        
                        if ((ZValue <= *(f32*)ZPixel) && ZValue >= Camera->FocalDistance )
                        {
                            *Pixel = VecColorToHex(Color);
                            *ZPixel = *(u32*)&ZValue;
                        }
                    }
                    Pixel++;
                    ZPixel++;
                }
                Row += Buffer->Pitch;
                ZRow += Buffer->Pitch;
            }
        } break;
        case 0:
        {
            // skip
        } break;
        case 3: // 2 verts in frustrum
        case 6:
        case 5:
        {
            v3 InsideVerts[2] = {};
            int InsideIndex = 0;
            
            v3 VertOut = (~(InsidePlane) & 1) * TrianglePos[0] + 
            (~(InsidePlane >> 1) & 1) * TrianglePos[1] +
            (~(InsidePlane >> 2) & 1) * TrianglePos[2];
            
            for (int VertIndex = 0;
                 VertIndex < 3;
                 ++VertIndex)
            {
                if ((InsidePlane >> VertIndex) & 1)
                {
                    InsideVerts[InsideIndex++] = TrianglePos[VertIndex];
                }
            }
            
            f32 V0t = (NearClippingPlane+0.01f - VertOut[2])/(InsideVerts[0][2] - VertOut[2]);
            f32 V1t = (NearClippingPlane+0.01f - VertOut[2])/(InsideVerts[1][2] - VertOut[2]);
            
            v3 Intersection0 = VertOut + V0t*(InsideVerts[0] - VertOut);
            v3 Intersection1 = VertOut + V1t*(InsideVerts[1] - VertOut);
            matrix3 NewTrianglePos0 = {};
            matrix3 NewTrianglePos1 = {};
            if (~(InsidePlane >> 1) & 1)
            {
                NewTrianglePos0 = {Intersection0, InsideVerts[1], InsideVerts[0]};
                NewTrianglePos1 = {Intersection0, Intersection1, InsideVerts[1]};
            }
            else
            {
                NewTrianglePos0 = {Intersection0, InsideVerts[0],  InsideVerts[1] };
                NewTrianglePos1 = {Intersection1, Intersection0, InsideVerts[1]};
            }
            RasterizeTriangle(NewTrianglePos0, TriangleColor, Camera, Buffer, NearClippingPlane); // TODO(jason): lerp color
            RasterizeTriangle(NewTrianglePos1, TriangleColor, Camera, Buffer, NearClippingPlane);
        } break;
        case 4: // 1 vert in frustrum
        case 2:
        case 1:
        {
            v3 VertIn = (InsidePlane & 1) * TrianglePos[0] +
            ((InsidePlane >> 1) & 1) * TrianglePos[1] +
            ((InsidePlane >> 2) & 1) * TrianglePos[2];
            
            v3 Sides[2] = {};
            int SideIndex = 0;
            
            for (int VertIndex = 0;
                 VertIndex < 3;
                 ++VertIndex)
            {
                if (~(InsidePlane >> VertIndex) & 1)
                {
                    Sides[SideIndex++] = TrianglePos[VertIndex] - VertIn;
                }
            }
            
            f32 V0t = (NearClippingPlane+.01f - VertIn[2])/Sides[0][2];
            f32 V1t = (NearClippingPlane+.01f - VertIn[2])/Sides[1][2];
            
            v3 Intersection0 = VertIn + V0t*Sides[0];
            v3 Intersection1 = VertIn + V1t*Sides[1];
            
            matrix3 NewTrianglePos = {};
            if((InsidePlane >> 1) & 1)
            {
                NewTrianglePos = {VertIn, Intersection1, Intersection0};
            }
            else
            {
                NewTrianglePos = {VertIn, Intersection0, Intersection1};
            }
            RasterizeTriangle(NewTrianglePos, TriangleColor, Camera, Buffer, NearClippingPlane);
            
        }break;
        default:
        {
            Assert("failure");
        } break;
    }
}

void
GameUpdateAndRender(thread_context* Thread, game_memory* Memory, 
                    game_input* Input, game_offscreen_buffer* Buffer)
{
    Assert((&Input->Controllers[0].Terminator - &Input->Controllers[0].Buttons[0]) ==
           (ArrayCount(Input->Controllers[0].Buttons)));
    Assert(sizeof(game_state) <= Memory->PermanentStorageSize);
    
    // TODO(jason): switch to memory arena
    
    game_state *GameState = (game_state *)Memory->PermanentStorage;
    GameState->Triangles = (matrix3*)(Buffer->ZBuffer + Buffer->BufferSize);
    GameState->TriangleCount = 2;
    GameState->Walkables = (matrix3*)(GameState->Triangles + GameState->TriangleCount);
    GameState->WalkableCount = 2;
    // TODO(jason): allocate space for walkables
    
    f32 NearClippingPlane = 1.0f;
    f32 FarClippingPlane = 1000000000.0f;
    game_camera* Camera = &GameState->PlayerCamera;
    
    // clear Buffer to black
    for (int BufferIndex = 0;
         BufferIndex < (Buffer->BufferSize/Buffer->BytesPerPixel);
         ++BufferIndex)
    {
        u32* Pixel = (u32*)Buffer->Memory + BufferIndex;
        *Pixel = 0x00;
    }
    
    // init Zbuffer to far clip plane
    for (int ZBufferIndex = 0;
         ZBufferIndex < (Buffer->BufferSize/Buffer->BytesPerPixel);
         ++ZBufferIndex)
    {
        u32* ZPixel = (u32*)Buffer->ZBuffer + ZBufferIndex;
        *ZPixel = *(u32*)&FarClippingPlane;
    }
    
    if(!Memory->IsInitialized)
    {
        
        GameState->PlayerPosition = {1.0f, 0.0f, 0.0f}; // TODO(jason): snap to closest walkable geometry
        GameState->PlayerVelocity = {};
        GameState->PlayerCameraOffset = {0.0f, 0.0f, 5.0f};
        Camera->Position = {0.0f, 0.0f, 5.0f};
        Camera->FocalDistance = 1.0f;
        Camera->Dimension = {1.0f, 1.0f};
        
        rotor3 ZRotation = AxisAngleToRotor({0, 1, 0}, Pi32*0.5f);
        
        Camera->Rotation = EulerToRotor(0, 0, -Pi32*0.5f);
        
        Camera->Rotation = Camera->Rotation * ZRotation;
        
        // TODO(jason): WHY IS THE X AXIS UP??????????
        
        matrix3 triangle0 = {};
        triangle0[0] = {-10.0f, -10.0f, 0.0f};
        triangle0[2] = {10.0f, -10.0f, 0.0f};
        triangle0[1] = {-10.0f, 10.0f, 0.0f};
        
        matrix3 triangle1 = {};
        triangle1[0] = {10.0f, 10.0f, 0.0f};
        triangle1[2] = {-10.0f, 10.0f, 0.0f};
        triangle1[1] = {10.0f, -10.0f, 0.0f};
        
        GameState->Triangles[0] = triangle0;
        GameState->Triangles[1] = triangle1;
        GameState->Walkables[0] = triangle0;
        GameState->Walkables[1] = triangle1;
        
        Memory->IsInitialized = true;
    }
    
    // Movement
    // for walkable geometry: (walkable geometry is defined as collidable geometry with a normal close to vertical)
    // cast poles from current player position
    // legal moves are within LegHeight of current player position and have no other collidable geometry within HeadHeight above the pole intersection
    // floodfill pathfinding algorithm
    
    // update PlayerVelocity
    game_controller_input Player1 = Input->Controllers[0];
    if (Player1.MoveLeft.EndedDown)
    {
        GameState->PlayerVelocity.x -= 1.0f;
    }
    if (Player1.MoveRight.EndedDown)
    {
        GameState->PlayerVelocity.x += 1.0f;
    }
    if (Player1.MoveUp.EndedDown)
    {
        GameState->PlayerVelocity.y += 1.0f;
    }
    if (Player1.MoveDown.EndedDown)
    {
        GameState->PlayerVelocity.y -= 1.0f;
    }
    
    f32 PlayerSpeed = 1.0f;
    v3 NormVelocity = NormalizeV3(GameState->PlayerVelocity);
    GameState->PlayerVelocity = PlayerSpeed * NormVelocity;
    
    if (LSquaredV3(GameState->PlayerVelocity) > 0.0f) // if player moved
    {
        f32 MinStepSize = 0.5f;
        v3 UpVector = {0,0,-1.0f}; // global? also not an upvector
        f32 LegHeight = 5.0f;
        f32 PlayerHeight = 10.0f;
        f32 tdest = NormVelocity * GameState->PlayerVelocity; // magnitude of PlayerVelocity
        
#define MAX_POLYGONS 2
        
        // each pole is an array of intersections
        // process each pole to get 1 intersection, each movement is an array of poles
        
        v3 PoleCollisions[3] = {};
        PoleCollisions[0] = GameState->PlayerPosition;
        int PoleCount = 1;
        for (f32 t = MinStepSize;
             t < tdest;
             t += MinStepSize)
        {
            v3 Intersections[MAX_POLYGONS] = {}; // TODO(jason): arenas
            int IntersectionSize = 0;
            v3 PolePos = GameState->PlayerPosition + t * NormVelocity;
            PolePos.z += PlayerHeight;
            PoleTriangleIntersection(Intersections, &IntersectionSize, GameState->Walkables, GameState->WalkableCount,
                                     PolePos, UpVector);
            f32 BestZ = GameState->PlayerPosition[2] - LegHeight;
            b32 NewPole = false; // TODO(jason): do this better
            for (int IntersectionIndex = 0;
                 IntersectionIndex < IntersectionSize;
                 ++IntersectionIndex)
            {
                // DEBUG display intersection
                f32 IntersectionZ = Intersections[IntersectionIndex].z;
                if (IntersectionZ >= BestZ)
                {
                    if (IntersectionZ <= GameState->PlayerPosition.z + LegHeight) // && (IntersectionZ >= GameState->PlayerPosition.z - LegHeight)
                        // within leg distance
                    {
                        PoleCollisions[PoleCount] = Intersections[IntersectionIndex];
                        BestZ = IntersectionZ;
                        NewPole = true;
                    }
                    else if (IntersectionZ < (BestZ + PlayerHeight))
                    {
                        break;
                    }
                }
            }
            if(NewPole) PoleCount++;
        }
        v3 Intersections[MAX_POLYGONS] = {}; // TODO(jason): arenas
        int IntersectionSize = 0;
        v3 PolePos = GameState->PlayerPosition + GameState->PlayerVelocity;
        PolePos.z += PlayerHeight;
        PoleTriangleIntersection(Intersections, &IntersectionSize, GameState->Walkables, GameState->WalkableCount,
                                 PolePos, UpVector);
        f32 BestZ = GameState->PlayerPosition[2] - LegHeight;
        b32 NewPole = false;
        for (int IntersectionIndex = 0;
             IntersectionIndex < IntersectionSize;
             ++IntersectionIndex)
        {
            // DEBUG display intersection
            f32 IntersectionZ = Intersections[IntersectionIndex].z;
            if (IntersectionZ >= BestZ)
            {
                if (IntersectionZ <= GameState->PlayerPosition.z + LegHeight) // && (IntersectionZ >= GameState->PlayerPosition.z - LegHeight)
                    // within leg distance
                {
                    PoleCollisions[PoleCount] = Intersections[IntersectionIndex];
                    BestZ = IntersectionZ;
                    NewPole = true;
                }
                else if (IntersectionZ < (BestZ + PlayerHeight))
                {
                    break;
                }
            }
        }
        if(NewPole) PoleCount++;
        
        Assert(LSquaredV3(PoleCollisions[PoleCount-1] - GameState->PlayerPosition) <=
               LSquaredV3(GameState->PlayerVelocity) + 0.01f);
        
        GameState->PlayerPosition = PoleCollisions[PoleCount-1]; 
        // TODO(jason): forward-only pathfinding algorithm
    }
    GameState->PlayerVelocity = {};
    Camera->Position = GameState->PlayerPosition + GameState->PlayerCameraOffset;
    
    // Camera Rotation
    v3 CameraPlanePos = Camera->Position; // defined as an offset from camera position
    CameraPlanePos.z += Camera->FocalDistance;
    v3 OriginViewVector = CameraPlanePos - Camera->Position;
    
    f32 MouseSens = Pi32*2.0f;
    f32 Yaw = Input->MouseX*MouseSens;  // TODO(jason): use angles instead of raw numbers
    f32 Pitch = Input->MouseY*MouseSens; 
    
    if ((Yaw != 0) && (Pitch != 0)) // if mouse moved then move camera
    {
        v3 UpVector = {0, 0, 1.0f};
        v3 XRotationAxis = Cross(RotateWithRotor(OriginViewVector, Camera->Rotation), UpVector);
        XRotationAxis = NormalizeV3(XRotationAxis);
        
        rotor3 XRotation = AxisAngleToRotor(RotateWithRotor(XRotationAxis, -Camera->Rotation), -Pitch);
        // TODO(jason): clamp pitch
        rotor3 foo = Camera-> Rotation * XRotation;
        if (foo.z < foo.w && foo.z > -foo.w)
        {
            XRotation = {1, 0, 0, 0};
        }
        
        rotor3 ZRotation = AxisAngleToRotor({0, 1, 0}, Yaw );
        
        
        Camera->Rotation = Camera->Rotation * ZRotation * XRotation;
        
        // Pitch = z, Yaw = x
        // get yaw to y axis
        // get pitch to x axis
        //rotor3 Rotation = EulerToRotor(-Pitch, Yaw, 0);
        rotor3 PitchRotor = EulerToRotor(Pitch, 0, 0);
        rotor3 YawRotor = EulerToRotor(0, Yaw, 0);
    }
    
#if 1
    char TextBuffer[256];
    euler_angles CameraOrientation = RotorToEuler(Camera->Rotation);
    _snprintf_s(TextBuffer, sizeof(TextBuffer), "y:%f p:%f r:%f\n", CameraOrientation.yaw, CameraOrientation.pitch, CameraOrientation.roll);
    OutputDebugStringA(TextBuffer);
#endif
    
    matrix3 Color = {};
    Color[0] = {1.0f, 0.0f, 0.0f};
    Color[1] = {0.0f, 1.0f, 0.0f};
    Color[2] = {0.0f, 0.0f, 1.0f};
    
    for (int TriangleIndex = 0;
         TriangleIndex < GameState->TriangleCount;
         ++TriangleIndex)
    {
        matrix3 Triangle = GameState->Triangles[TriangleIndex];
        
        matrix3 CameraTriangle = {};
        CameraTriangle[0] = RotateWithRotor(Triangle[0] - Camera->Position, -Camera->Rotation);
        CameraTriangle[1] = RotateWithRotor(Triangle[1] - Camera->Position, -Camera->Rotation);
        CameraTriangle[2] = RotateWithRotor(Triangle[2] - Camera->Position, -Camera->Rotation);
        RasterizeTriangle(CameraTriangle, Color, Camera, Buffer, NearClippingPlane);
    }
    
}
