#include "libevent_tcp_link.h"
#include "snp.h"
#include "snp_defs_p.h"
#include "snp_buffer.h"

#include <netinet/in.h>
# ifdef _XOPEN_SOURCE_EXTENDED
#  include <arpa/inet.h>
# endif
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <event2/bufferevent.h>
#include <event2/buffer.h>
#include <event2/listener.h>
#include <event2/util.h>
#include <event2/event.h>
#include <event2/thread.h>
#include <event2/bufferevent_ssl.h>


static int32_t __snp_link_read(void *handle, struct SNP_BUFFER *buffer)
{
	uint8_t data[SNP_DEFAULT_BUFFER_SIZE] = {0};
	struct bufferevent *bev = (struct bufferevent *)handle;
	struct evbuffer *input = bufferevent_get_input(bev);

	int32_t read_size = evbuffer_copyout(input, data, sizeof(data));
	read_size = snp_buffer_write(buffer, data, read_size);

	evbuffer_drain(input, read_size);

	return 0;
}


static int32_t __snp_link_write(void *handle, struct SNP_BUFFER *buffer)
{
	struct bufferevent *bev = (struct bufferevent *)handle;
	uint8_t *data = NULL;

	int32_t write_size = snp_buffer_copyout_ptr(buffer, &data, SNP_DEFAULT_BUFFER_SIZE);
	bufferevent_write(bev, data, write_size);

	snp_buffer_drain(buffer, write_size);

	return 0;
}


/**
 * @brief tcp连接事件回调
 */
static void __link_event_cb(struct bufferevent *bev, short events, void *arg)
{
	if (events & BEV_EVENT_EOF)
	{
		SNP_NOTICE("connect closed\r\n");
	}
	else if (events & BEV_EVENT_ERROR)
	{
		SNP_NOTICE("Got an error (%d) on the connection: %s\r\n",
			EVUTIL_SOCKET_ERROR(), evutil_socket_error_to_string(EVUTIL_SOCKET_ERROR()));
	}
	else if (events & BEV_EVENT_CONNECTED)
	{
		SNP_NOTICE("connect server success\r\n");

		return;
	}
	else
	{
		SNP_NOTICE("unsupported event type: %x\r\n", events);
	}

	struct SNP_LINK *link = (struct SNP_LINK *)arg;

	snp_link_setup_rw_cb(link, NULL, NULL, NULL);

	bufferevent_free(bev);
}


/**
 * @brief 协议栈定时执行任务
 */
static void __snp_timer_event_cb(evutil_socket_t fd, short what, void *arg)
{
	struct SNP *snp = (struct SNP *)arg;

	snp_exec(snp, 10);
}


/**
 * @brief 创建并返回一个libevent的调度对象
 * @return 创建完成的调度对象指针 NULL 创建失败
 */
void *libevent_base_create(void)
{
	struct event_base *base = NULL;
	base = event_base_new();
	return base;
}

/**
 * @brief 为目标协议栈创建一个tcp link
 * @param base libevent的调度器对象 struct event_base
 * @param snp 目标协议栈对象
 * @param ip 目标ip
 * @param port 目标端口
 */
void libevent_tcp_client_link_create(void *base, struct SNP *snp, char *ip, uint16_t port)
{
	struct bufferevent *bev = NULL;

	bev = bufferevent_socket_new(base, -1, BEV_OPT_CLOSE_ON_FREE);

	struct sockaddr_in serv = {0};
	serv.sin_family = AF_INET;
	serv.sin_port = htons(port);
	inet_pton(AF_INET, ip, &serv.sin_addr.s_addr);

	int ret = bufferevent_socket_connect(bev, (struct sockaddr *)&serv, sizeof(serv));

	SNP_NOTICE("libevent tcp client link create ret: %d\r\n", ret);

	struct SNP_LINK *_new_link = snp_create_physical_node(snp, __snp_link_read, __snp_link_write, bev);

	bufferevent_setcb(bev, NULL, NULL, __link_event_cb, _new_link);
	bufferevent_enable(bev, EV_READ | EV_WRITE);
}


/**
 * @brief snp协议栈tcp服务器监听
 */
static void snp_tcp_listener_cb(struct evconnlistener *listener, evutil_socket_t fd,
	struct sockaddr *sa, int socklen, void *snp_handle)
{
	struct event_base *base = evconnlistener_get_base(listener);

	struct bufferevent *bev = NULL;
	bev = bufferevent_socket_new(base, fd, BEV_OPT_CLOSE_ON_FREE);

	struct SNP_LINK *_new_link = snp_create_physical_node(snp_handle, __snp_link_read, __snp_link_write, bev);

	printf("create bev: %p\r\n", bev);

	bufferevent_setcb(bev, NULL, NULL, __link_event_cb, _new_link);
	bufferevent_enable(bev, EV_READ | EV_WRITE);
}


/**
 * @brief 创建一个tcp服务器连接
 * @param base libevent的调度器对象 struct event_base
 * @param snp 目标协议栈对象
 * @param port 监听端口号
 */
void libevent_tcp_server_link_create(void *base, struct SNP *snp, uint16_t port)
{
	struct evconnlistener *listener = NULL;
	struct sockaddr_in sin = {0};

	sin.sin_family = AF_INET;
	sin.sin_port = htons(port);

	listener = evconnlistener_new_bind(base, snp_tcp_listener_cb, (void *)snp,
		LEV_OPT_REUSEABLE | LEV_OPT_CLOSE_ON_FREE, -1,
		(struct sockaddr*)&sin,
		sizeof(sin));

	if (NULL == listener)
	{
		SNP_ERROR("Could not create a snp tcp link listener!\r\n");
	}
}


/**
 * @brief 执行基于libevent的tcp client 连接 该函数退出后，传递进入的base对象会被释放，再次使用将导致错误
 * @param base libevent 调度对象
 * @param snp 协议栈对象 若传入，则会将该协议栈挂到libevent上进行执行
 */
void libevent_tcp_link_exec(void *base, struct SNP *snp)
{
	if (NULL != snp)
	{
		struct timeval start_tv = {
			.tv_sec = 0,
			.tv_usec = 10 * 1000
		};
		struct event *snp_timer_event = event_new(base, -1, EV_PERSIST, __snp_timer_event_cb, snp);

		evtimer_add(snp_timer_event, &start_tv);
	}

	event_base_dispatch(base);

	event_base_free(base);
}
