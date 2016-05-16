#ifndef __UMF_UMFDEBUG_H
#define __UMF_UMFDEBUG_H

#include "singleton.h"
#include "image.h"
#include "renderer.h"
#include "../defines.h"

#include <vector>
#include <map>
#include <string>

#if defined(__WIN32__) || defined(_WIN32) || defined(__CYGWIN__)
#include <Windows.h>
#else
#include <sys/time.h>
#endif

#include <assert.h>

namespace umf
{

/**
 * Timer for measuiring performance. Should provide nanosecond precision.
 * Don't use directly. Better to use the logged times as implemented in
 * the debug class below. 
 */
class UMF Timer {
private:

#if defined(__WIN32__) || defined(_WIN32) || defined(__CYGWIN__)
    LARGE_INTEGER startTime;
    LARGE_INTEGER frequency;
    int initialized;
#else
    timeval startTime;
#endif

public:

    Timer(){
#if defined(__WIN32__) || defined(_WIN32) || defined(__CYGWIN__)
        this->initialized = 0;
#endif

    }
    ~Timer(){}

    void start(){
#if defined(__WIN32__) || defined(_WIN32) || defined(__CYGWIN__)
        BOOL r;

        if (!initialized) {                           /* first call */
            initialized = 1;
            r = QueryPerformanceFrequency(&frequency);  /* take the sampling frequency */
            if (r == 0) {                               /* in case high-res frequency timer is not available */
                assert(0 && "HiRes timer is not available.");
                exit(-1);
            }
        }

        r = QueryPerformanceCounter(&(this->startTime));          /* take the value of the counter */
        assert(r != 0 && "This should never happen.");
#else
        gettimeofday(&startTime, NULL);
#endif
    }

    double stop(){
        double duration;
#if defined(__WIN32__) || defined(_WIN32) || defined(__CYGWIN__)
        BOOL r;
        LARGE_INTEGER value;
        r = QueryPerformanceCounter(&value); 
        assert(r != 0 && "This should never happen.");
        duration = (double)(value.QuadPart - startTime.QuadPart)*1000.0 / (double)frequency.QuadPart; /* vrat hodnotu v milisekundach */
#else
        timeval endTime;
        long seconds, useconds;

        gettimeofday(&endTime, NULL);

        seconds  = endTime.tv_sec  - startTime.tv_sec;
        useconds = endTime.tv_usec - startTime.tv_usec;

        duration = seconds*1000 + useconds/1000.0;

#endif
        return duration;
    }
};



/**
 * Debug and Performance evaluation helper. Also provides interface for accessing the renderer
 * for debug purposes (drawing lines, points).
 */
class UMF UMFDebug
{
public:
    UMFDebug();

    void setRenderer(Renderer *r){ this->renderer = r; }
    Renderer *getRenderer(){ return this->renderer; }

    //PIXEL counter
#ifdef UMF_DEBUG_COUNT_PIXELS
    void addPixels(int count) { this->pixelCount += count; }
    long long getPixels() { return this->pixelCount; }
#endif

#ifdef UMF_DEBUG_TIMING
    //TIMING loger
    
    /**
     * Start a timer - an ID is returned for further use
     */
    int logEventStart();
    
    /**
     * Tag the end of the timer given by ID. the messages are groupped
     */
    void logEventEnd(int id, std::string message);
    
    /**
     * More verbose and direct way of logging events
     * @param time in milliseconds
     */
    void logEvent(double time, std::string name);
    
    /**
     * Get all logged events from the beginning of the program
     */
    std::vector< std::pair<double, std::string> > &getEvents(){return this->timeEvents;}
    
    /**
     * Fill the vector with all the events
     */
    void getTimeLog(std::vector< std::pair<double, std::string> > &events);

    /**
     * Group by message and compute the average time for all events
     */
    void getUniqLog(std::vector< std::pair<double, std::string> > &events);

    void clearEvents();
#endif

	void setReasonFailed(std::string reason) { this->failedReason = reason; }
	std::string getReasonFailed() { return this->failedReason; }

    //platform independent log (android + stdout)
    static void logMsg(std::string message);

private:
    Renderer *renderer;
    long long pixelCount;

    //timing stuff
    std::vector< std::pair<double, std::string> > timeEvents;
    std::map<int, Timer> startTick;
    std::map< std::string, std::pair<int, double> > eventMap;
    int prevID;
	std::string failedReason;
    //timing stuff ent
};


typedef Singleton<UMFDebug> UMFDSingleton;

}

#endif // UMFDEBUG_H
