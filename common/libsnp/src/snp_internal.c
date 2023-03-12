#include "snp_internal.h"
#include "snp_queue.h"

#ifdef SNP_RESOURCE_PRE_MALLOC

/**
 * @brief 预分配的snp节点资源
 */
struct SNP_NODE __pre_malloc_snp_node[SNP_NODE_PRE_MALLOC_NUM];
struct SNP_NODE_LIST pre_malloc_snp_nodes;

/**
 * @brief 预分配的连接资源
 */
struct SNP_LINK __pre_malloc_snp_link[SNP_LINK_PRE_MALLOC_NUM];
struct SNP_LINK_LIST pre_malloc_snp_links;


struct SNP_MSGS_PUB __pre_malloc_snp_msgs_pubs[SNP_MSGS_PUB_PRE_MALLOC_NUM];
struct SNP_MSGS_PUB_LIST pre_malloc_snp_msgs_pubs;

#endif

/**
 * @brief 内部初始化接口
 */
void snp_internal_init(void)
{
	int i = 0;

#ifdef SNP_RESOURCE_PRE_MALLOC

	TAILQ_INIT(&pre_malloc_snp_msgs_pubs);
	for (i = 0; i < SNP_MSGS_PUB_PRE_MALLOC_NUM; i++)
	{
		TAILQ_INSERT_TAIL(&pre_malloc_snp_msgs_pubs, &__pre_malloc_snp_msgs_pubs[i], MSGS);
	}

	TAILQ_INIT(&pre_malloc_snp_nodes);
	for (i = 0; i < SNP_NODE_PRE_MALLOC_NUM; i++)
	{
		TAILQ_INSERT_TAIL(&pre_malloc_snp_nodes, &__pre_malloc_snp_node[i], NODE);
	}

	LIST_INIT(&pre_malloc_snp_links);
	for (i = 0; i < SNP_LINK_PRE_MALLOC_NUM; i++)
	{
		LIST_INSERT_HEAD(&pre_malloc_snp_links, &__pre_malloc_snp_link[i], LINK);
	}

#endif
}


/**
 * @brief 获取一个消息发布资源
 * 
 * @return struct SNP_MSGS_PUB* 获取到的消息发布资源 
 */
struct SNP_MSGS_PUB *snp_internal_get_new_msgs_pub(void)
{
#ifdef SNP_RESOURCE_PRE_MALLOC
	struct SNP_MSGS_PUB *__new_msgs_pub = TAILQ_FIRST(&pre_malloc_snp_msgs_pubs);
	if (NULL != __new_msgs_pub)
	{
		TAILQ_REMOVE(&pre_malloc_snp_msgs_pubs, __new_msgs_pub, MSGS);
	}
#else
	struct SNP_MSGS_PUB *__new_msgs_pub = (struct SNP_MSGS_PUB *)snp_malloc(sizeof(struct SNP_MSGS_PUB));
#endif

	return __new_msgs_pub;
}


/**
 * @brief 释放一个消息发布资源
 * 
 * @param msgs_pub 
 */
void snp_internal_release_msgs_pub(struct SNP_MSGS_PUB *msgs_pub)
{
#ifdef SNP_RESOURCE_PRE_MALLOC
	TAILQ_INSERT_TAIL(&pre_malloc_snp_msgs_pubs, msgs_pub, MSGS);
#else

#endif
}


/**
 * @brief 获取一个新的节点资源
 * 
 * @return struct SNP_NODE* 得到的节点对象指针
 */
struct SNP_NODE *snp_internal_get_new_node(void)
{
#ifdef SNP_RESOURCE_PRE_MALLOC
	struct SNP_NODE *__new_node = TAILQ_FIRST(&pre_malloc_snp_nodes);
	if (NULL != __new_node)
	{
		TAILQ_REMOVE(&pre_malloc_snp_nodes, __new_node, NODE);
	}
#else
	struct SNP_NODE *__new_node = (struct SNP_NODE *)snp_malloc(sizeof(struct SNP_NODE));
#endif

	return __new_node;
}


/**
 * @brief 释放一个节点资源
 * 
 * @param node 待释放的节点对象指针
 */
void snp_internal_release_node(struct SNP_NODE *node)
{
#ifdef SNP_RESOURCE_PRE_MALLOC
	TAILQ_INSERT_TAIL(&pre_malloc_snp_nodes, node, NODE);
#else

#endif

}


/**
 * @brief 获取一个新的连接资源
 * 
 * @return struct SNP_LINK* 得到的连接对象指针
 */
struct SNP_LINK *snp_internal_get_new_link(void)
{
#ifdef SNP_RESOURCE_PRE_MALLOC
	struct SNP_LINK *__new_link = LIST_FIRST(&pre_malloc_snp_links);
	if (NULL != __new_link)
	{
		LIST_REMOVE(__new_link, LINK);
	}
#else
	struct SNP_LINK *_new_link = (struct SNP_LINK *)snp_malloc(sizeof(struct SNP_LINK));
#endif

	return __new_link;
}


/**
 * @brief 释放一个连接资源
 * 
 * @param link 待释放的连接对象指针
 */
void snp_internal_release_link(struct SNP_LINK *link)
{
#ifdef SNP_RESOURCE_PRE_MALLOC
	LIST_INSERT_HEAD(&pre_malloc_snp_links, link, LINK);
#else

#endif

}
