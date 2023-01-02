#include "wtb_server.h"

#include <unistd.h>
#include <iostream>
#include <chrono>
#include <functional>


#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <signal.h>
#ifndef _WIN32
#include <netinet/in.h>
# ifdef _XOPEN_SOURCE_EXTENDED
#  include <arpa/inet.h>
# endif
#include <sys/socket.h>
#endif

#include <event2/bufferevent.h>
#include <event2/buffer.h>
#include <event2/listener.h>
#include <event2/util.h>
#include <event2/event.h>


#ifdef __cplusplus
extern "C" {
#endif



static void
conn_writecb(struct bufferevent *bev, void *user_data)
{
	struct evbuffer *output = bufferevent_get_output(bev);
	if (evbuffer_get_length(output) == 0) {
		// printf("flushed answer\n");
		// bufferevent_free(bev);
	}
}


static void
conn_readcb(struct bufferevent *bev, void *user_data)
{
    struct evbuffer *read_buffer = bufferevent_get_input(bev);

    uint8_t buffer[1024] = {0};
    uint32_t size = 0;

    while ((size = evbuffer_copyout(read_buffer, buffer, sizeof(buffer))) > 0)
    {
        struct WTB_PACKAGE *package = NULL;
        size = wtb_package_unpack(buffer, size, &package);

        if (NULL != package)
        {
			printf("wtb receive mgs head: %x, len: %d, type: %d, check: %d\r\n",
				package->magic, package->payload_len, package->payload_type, package->payload_check
			);
        }

        printf("wtb server drain bytes size: %d\r\n", size);
        evbuffer_drain(read_buffer, size);
    }
}


static void
conn_eventcb(struct bufferevent *bev, short events, void *user_data)
{
	if (events & BEV_EVENT_EOF) {
		printf("Connection closed.\n");
	} else if (events & BEV_EVENT_ERROR) {
		printf("Got an error on the connection: %s\n",
		    strerror(errno));/*XXX win32*/
	}
	/* None of the other events can happen here, since we haven't enabled
	 * timeouts */
	bufferevent_free(bev);
}


static void
listener_cb(struct evconnlistener *listener, evutil_socket_t fd,
    struct sockaddr *sa, int socklen, void *user_data)
{
	struct event_base *base = evconnlistener_get_base(listener);
	struct bufferevent *bev;

	bev = bufferevent_socket_new(base, fd, BEV_OPT_CLOSE_ON_FREE);
	if (!bev) {
		fprintf(stderr, "Error constructing bufferevent!");
		event_base_loopbreak(base);
		return;
	}
	bufferevent_setcb(bev, conn_readcb, conn_writecb, conn_eventcb, user_data);
	bufferevent_enable(bev, EV_WRITE | EV_READ | EV_CLOSED);
	// bufferevent_disable(bev, EV_READ);

    printf("get new connector\r\n");

	// bufferevent_write(bev, MESSAGE, strlen(MESSAGE));
}


#ifdef __cplusplus
}
#endif

WtbServer::WtbServer(uint16_t port)
    : std::thread(std::bind(&WtbServer::run, this, port))
{

}


void WtbServer::run(uint16_t port)
{
	struct event_base *base;
	struct evconnlistener *listener;

	struct sockaddr_in sin = {0};

	base = event_base_new();
	if (!base) {
		fprintf(stderr, "Could not initialize libevent!\n");
		return;
	}

	sin.sin_family = AF_INET;
	sin.sin_port = htons(port);

	listener = evconnlistener_new_bind(base, listener_cb, (void *)this,
	    LEV_OPT_REUSEABLE|LEV_OPT_CLOSE_ON_FREE, -1,
	    (struct sockaddr*)&sin,
	    sizeof(sin));

	if (!listener) {
		fprintf(stderr, "Could not create a listener!\n");
		return;
	}

	event_base_dispatch(base);

	evconnlistener_free(listener);
	event_base_free(base);

	printf("done\n");

}
