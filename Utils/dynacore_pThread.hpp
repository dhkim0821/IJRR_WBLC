#ifndef DYNACORE_THREAD
#define DYNACORE_THREAD

#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
 
class dynacore_pThread{
protected:
    pthread_t sejong_thread;
    bool firstLoopFlag;
    bool isRunning;


    void terminate();
    bool isFirstLoop();

public:
    dynacore_pThread();
    virtual ~dynacore_pThread(void);
    virtual void run(void) = 0;

    void start();
};
void *runData(void * arg);
void sigint(int signo);



#endif
