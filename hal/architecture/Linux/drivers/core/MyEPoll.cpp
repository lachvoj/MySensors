#include <sys/epoll.h>
#include <unistd.h>

#include "MyEPoll.h"

MyEPoll::MyEPoll()
{
    epollFd = epoll_create1(0);
    events.resize(64);
}

MyEPoll::~MyEPoll()
{
    close(epollFd);
}

void MyEPoll::addDescriptor(int fd)
{
    struct epoll_event event;
    event.data.fd = fd;
    event.events = EPOLLIN;
    epoll_ctl(epollFd, EPOLL_CTL_ADD, fd, &event);
}

void MyEPoll::removeDescriptor(int fd)
{
    epoll_ctl(epollFd, EPOLL_CTL_DEL, fd, nullptr);
}

void MyEPoll::run()
{
    epoll_wait(epollFd, events.data(), events.size(), -1);
}

MyEPoll myEpoll;
