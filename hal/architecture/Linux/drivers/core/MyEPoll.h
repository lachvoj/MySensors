#ifndef MYEPOLL_H
#define MYEPOLL_H

#include <functional>
#include <vector>

class MyEPoll
{
  public:
    MyEPoll();
    ~MyEPoll();

    void addDescriptor(int fd);
    void removeDescriptor(int fd);
    void run();

  private:
    int epollFd;
    std::vector<struct epoll_event> events;
};

extern MyEPoll myEpoll;

#endif // MYEPOLL_H