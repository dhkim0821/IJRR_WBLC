#include "dynacore_pThread.hpp"

dynacore_pThread::dynacore_pThread() :
    sejong_thread(),
    firstLoopFlag(false),
    isRunning(false)
{}

dynacore_pThread::~dynacore_pThread()
{
    pthread_cancel(sejong_thread);
    pthread_join(sejong_thread, NULL);
}
void dynacore_pThread::terminate()
{
    printf("terminating thread\n");
    isRunning = false;
}
bool dynacore_pThread::isFirstLoop()
{
    return firstLoopFlag;
}

void *runThread(void * arg)
{
    ((dynacore_pThread*) arg)->run();
    return NULL;
}

void sigint(int signo)
{
    (void) signo;
}
void dynacore_pThread::start()
{
    if(!isRunning){
        sigset_t sigset, oldset;
        sigemptyset(&sigset);
        sigaddset(&sigset, SIGINT);
        pthread_sigmask(SIG_BLOCK, &sigset, &oldset);
        pthread_create(&sejong_thread, NULL, runThread, this);
        struct sigaction s;
        s.sa_handler = sigint;
        sigemptyset(&s.sa_mask);
        s.sa_flags = 0;
        sigaction(SIGINT, &s, NULL);
        pthread_sigmask(SIG_SETMASK, &oldset, NULL);

        isRunning = true;
    }
}

