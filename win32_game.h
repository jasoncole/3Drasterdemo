/* date = July 28th 2021 10:56 am */

#ifndef WIN32_GAME_H
#define WIN32_GAME_H

struct win32_window_dimension
{
    int Width;
    int Height;
};

struct win32_offscreen_buffer
{
    BITMAPINFO Info;
    void *Memory;
    int Width;
    int Height;
    int Pitch;
    int BytesPerPixel;
};


struct win32_state
{
    u64 TotalSize;
    void *GameMemoryBlock;
    
#if 0
    win32_replay_buffer ReplayBuffers[4];
    
    HANDLE RecordingHandle;
    int InputRecordingIndex;
    
    HANDLE PlaybackHandle;
    int InputPlayingIndex;
    
    char EXEFileName[WIN32_STATE_FILE_NAME_COUNT];
    char *OnePastLastEXEFileNameSlash;
#endif
};


#endif //WIN32_GAME_H
