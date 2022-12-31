#ifndef __WIN_TOOL_BRIDGE_SERVER_H__
#define __WIN_TOOL_BRIDGE_SERVER_H__


#include <stdint.h>

#include <thread>


class WtbServer : public std::thread
{
public:
    WtbServer(uint16_t port);

private:
    void run();
};



#endif
