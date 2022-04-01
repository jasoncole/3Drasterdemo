/* date = August 1st 2021 3:29 pm */

#ifndef GAME_PLATFORM_H
#define GAME_PLATFORM_H

#ifdef __cplusplus
extern "C" {
#endif
    
#include <stdint.h>
    
    typedef int8_t i8;
    typedef int16_t i16;
    typedef int32_t i32;
    typedef int64_t i64;
    typedef i32 b32;
    
    typedef uint8_t u8;
    typedef uint16_t u16;
    typedef uint32_t u32;
    typedef uint64_t u64;
    
    typedef float f32;
    typedef double f64;
    
    typedef struct thread_context
    {
        int Placeholder;
    } thread_context;
    
    /*
      NOTE(casey): Services that the platform layer provides to the game
    */
#if GAME_INTERNAL
#endif
    
    
    struct game_offscreen_buffer
    {
        void* Memory;
        u32* ZBuffer;
        int BufferSize;
        int Width;
        int Height;
        int Pitch;
        int BytesPerPixel;
    };
    
    typedef struct game_button_state
    {
        int HalfTransitionCount;
        b32 EndedDown;
    } game_button_state;
    
    struct game_controller_input
    {
        b32 IsConnected;
        b32 IsAnalog;    
        f32 StickAverageX;
        f32 StickAverageY;
        
        union
        {
            game_button_state Buttons[11];
            struct
            {
                game_button_state MoveUp;
                game_button_state MoveDown;
                game_button_state MoveLeft;
                game_button_state MoveRight;
                
                game_button_state WeaponSwitch;
                game_button_state Jump;
                game_button_state Reload;
                
                game_button_state Attack;
                game_button_state Block;
                
                game_button_state Back;
                game_button_state Start;
                
                // NOTE(casey): All buttons must be added above this line
                
                game_button_state Terminator;
            };
        };
    };
    
    struct game_input
    {
        game_button_state MouseButtons[5];
        f32 MouseX, MouseY, MouseZ;
        
        f32 dtForFrame;
        
        game_controller_input Controllers[5];
    };
    
    struct game_memory
    {
        b32 IsInitialized;
        u64 PermanentStorageSize;
        void* PermanentStorage; // required to be cleared to 0 at startup
        
        u64 TransientStorageSize;
        void* TransientStorage; // required to be cleared to 0 at startup
        
        /*debug_platform_read_entire_file* DEBUGPlatformReadEntireFile;
        debug_platform_free_file_memory* DEBUGPlatformFreeFileMemory;
        debug_platform_write_entire_file* DEBUGPlatformWriteEntireFile;*/
    };
    
#ifdef __cplusplus
}
#endif


#endif //GAME_PLATFORM_H
