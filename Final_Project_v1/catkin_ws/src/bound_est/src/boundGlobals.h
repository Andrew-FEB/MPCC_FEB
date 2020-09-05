#pragma once

//Resets logs on first starting program
#if defined(DEBUG) || defined(TIME_LOG)
extern bool reset_logs;
#endif

//Allows option 
extern bool counter_clockwise_track;
extern bool clockwise_track;