#include "umfdebug.h"
#include "../defines.h"

#ifdef UMF_ANDROID
#include <jni.h>
#include <android/log.h>

#define  LOG_TAG    "umf_gl"
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)
#else
#include <cstdio>
#endif

namespace umf {

//singleton definition
template<>
UMF UMFDebug* Singleton<UMFDebug>::m_pInstance = NULL;

UMFDebug::UMFDebug()
{
    this->renderer = NULL;
#ifdef UMF_DEBUG_TIMING
    this->clearEvents();
#endif
}

#ifdef UMF_DEBUG_TIMING
void UMFDebug::logEvent ( double tickms, std::string name )
{
    this->timeEvents.push_back(std::pair<double, std::string>(tickms, name));
    std::map< std::string, std::pair<int, double> >::iterator it;
    if((it = this->eventMap.find(name)) != this->eventMap.end())
    {
        it->second.first += 1;
        it->second.second += tickms;
    } else {
        this->eventMap.insert(std::pair< std::string, std::pair<int, int> >(name, std::pair<int, int>(1, tickms)));
    }
}

void UMFDebug::logEventEnd ( int id, std::string message )
{

    std::map<int, Timer>::iterator it;
    if((it = this->startTick.find(id)) != this->startTick.end())
    {
        this->logEvent(it->second.stop(), message);
        this->startTick.erase(it);
    }
}

int UMFDebug::logEventStart()
{
    this->prevID++;
    Timer timer;
    timer.start();
    this->startTick.insert(std::pair<int,Timer>(this->prevID, timer));
    return this->prevID;
}

void UMFDebug::clearEvents()
{
    this->startTick.clear();
    this->prevID = 0;
    this->timeEvents.clear();
    this->eventMap.clear();
}

void UMFDebug::getTimeLog(std::vector< std::pair<double, std::string> > &events)
{
    //double freq = cvGetTickFrequency()*1e3; //get it in ms
    for(std::vector< std::pair<double, std::string> >::iterator it = this->timeEvents.begin(); it != this->timeEvents.end(); it++)
    {
        events.push_back(std::pair<double, std::string>(it->first, it->second));
    }
}

void UMFDebug::getUniqLog(std::vector< std::pair<double, std::string> > &events)
{
    for(std::map< std::string, std::pair<int, double> >::iterator it = this->eventMap.begin(); it != this->eventMap.end(); it++)
    {
        events.push_back(std::pair<double, std::string>(it->second.second/it->second.first, it->first));
    }
}
#endif //umf debug timing

void UMFDebug::logMsg(std::string msg)
{
#ifdef UMF_ANDROID
	LOGI("UMFD: %s", msg.c_str());
#else
	printf("UMFD: %s\n", msg.c_str());
#endif
}

}
