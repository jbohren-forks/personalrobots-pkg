#ifndef TEST_TIMER_H
#define TEST_TIMER_H

#include <ctime>
#include <string>
#include <cstdio>

class Timer
{
public:
    Timer(std::string name = "Timer") : m_name(name)
    {
        m_start = clock();
    }

    ~Timer()
    {
        clock_t end = clock();
        float seconds = (float)(end - m_start) / CLOCKS_PER_SEC;
        printf("%s: %f s\n", m_name.c_str(), seconds);
    }
    
private:
    std::string m_name;
    clock_t m_start;
};

#endif
