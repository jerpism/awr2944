#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/DebugP.h>

#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/tcpip.h"
#include "lwip/dhcp.h"
#include "lwip/sys.h"
#include "lwip/api.h"
#include "lwip/sockets.h"

#include <networking/enet/core/include/per/cpsw.h>
#include <networking/enet/utils/include/enet_apputils.h>
#include <networking/enet/utils/include/enet_board.h>
#include <networking/enet/core/include/enet.h>
#include "ti_board_config.h"
#include "ti_enet_open_close.h"
#include "ti_enet_config.h"
#include "ti_enet_lwipif.h"
#include "app_cpswconfighandler.h"


#define SOCK_HOST_SERVER_IP6  ("FE80::12:34FF:FE56:78AB")

#define SOCK_HOST_SERVER_PORT  (8888)

#define APP_SOCKET_MAX_RX_DATA_LEN (1024U)

#define APP_SOCKET_NUM_ITERATIONS (1U)

#define APP_SEND_DATA_NUM_ITERATIONS (5U)

#define MAX_IPV4_STRING_LEN (16U)

#define R5F_CACHE_LINE_SIZE  (32)

#define UTILS_ALIGN(x,align)  ((((x) + ((align) - 1))/(align)) * (align))

static uint8_t gBuff[512];


struct App_hostInfo_t
{
    struct sockaddr_in socketAddr;
};

LwipifEnetApp_Handle hlwipIfApp = NULL;
struct netif *g_pNetif[ENET_SYSCFG_NETIF_COUNT];
static uint8_t gRxDataBuff[APP_SOCKET_MAX_RX_DATA_LEN];

static const uint8_t BROADCAST_MAC_ADDRESS[ENET_MAC_ADDR_LEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };


static struct App_hostInfo_t gHostInfo;
static char   gHostServerIp4[MAX_IPV4_STRING_LEN] = "";

static int32_t gSock = -1;


static inline int32_t App_isNetworkUp(struct netif* netif_)
{
    return (netif_is_up(netif_) && netif_is_link_up(netif_) && !ip4_addr_isany_val(*netif_ip4_addr(netif_)));
}


static void App_netifStatusChangeCb(struct netif *pNetif)
{
    if (netif_is_up(pNetif))
    {
        DebugP_log("Enet IF UP Event. Local interface IP:%s\r\n",
                    ip4addr_ntoa(netif_ip4_addr(pNetif)));
    }
    else
    {
        DebugP_log("Enet IF DOWN Event\r\n");
    }
    return;
}

static void App_netifLinkChangeCb(struct netif *pNetif)
{
    if (netif_is_link_up(pNetif))
    {
        DebugP_log("Network Link UP Event\r\n");
    }
    else
    {
        DebugP_log("Network Link DOWN Event\r\n");
    }
    return;
}


static void App_setupNetif()
{
    ip4_addr_t ipaddr, netmask, gw;
 

    ip4_addr_set_zero(&gw);
    ip4_addr_set_zero(&ipaddr);
    ip4_addr_set_zero(&netmask);

    // 192.168.1.10
    ipaddr.addr = (192U) | (168U << 8) | (1U << 16) | (10U << 24);

    DebugP_log("Starting lwIP, local interface IP is not dhcp-enabled\r\n");
    hlwipIfApp = LwipifEnetApp_getHandle();
    sys_lock_tcpip_core();
    for (uint32_t i = 0U; i < ENET_SYSCFG_NETIF_COUNT; i++)
    {
        /* Open the netif and get it populated*/
        g_pNetif[i] = LwipifEnetApp_netifOpen(hlwipIfApp, NETIF_INST_ID0 + i, &ipaddr, &netmask, &gw);
        netif_set_status_callback(g_pNetif[i], App_netifStatusChangeCb);
        netif_set_link_callback(g_pNetif[i], App_netifLinkChangeCb);
        netif_set_up(g_pNetif[NETIF_INST_ID0 + i]);
    }
    sys_unlock_tcpip_core();
    LwipifEnetApp_startSchedule(hlwipIfApp, g_pNetif[ENET_SYSCFG_DEFAULT_NETIF_IDX]);

}


static void App_tcpipInitCompleteCb(void *pArg)
{
    sys_sem_t *pSem = (sys_sem_t*)pArg;
    EnetAppUtils_assert(pArg != NULL);

    /* init randomizer again (seed per thread) */
    srand((unsigned int)sys_now()/1000);

    App_setupNetif();



    sys_sem_signal(pSem);
}


static void App_setupNetworkStack()
{
    sys_sem_t pInitSem;
    const err_t err = sys_sem_new(&pInitSem, 0);
    EnetAppUtils_assert(err == ERR_OK);

    tcpip_init(App_tcpipInitCompleteCb, &pInitSem);

    /* wait for TCP/IP initialization to complete */
    sys_sem_wait(&pInitSem);
    sys_sem_free(&pInitSem);

    return;
}


static void Appsocket_fillHostSocketInfo(struct App_hostInfo_t* pHostInfo)
{
    ip_addr_t ipAddr;
    int32_t addr_ok;
    memset(&pHostInfo->socketAddr, 0, sizeof(pHostInfo->socketAddr));
    char *ip = "192.168.1.2";
    struct sockaddr_in*  pAddr = &pHostInfo->socketAddr;
    IP_SET_TYPE_VAL(dstaddr, IPADDR_TYPE_V4);
    addr_ok = ip4addr_aton(ip, ip_2_ip4(&ipAddr));
    pAddr->sin_len = sizeof(pHostInfo->socketAddr);
    pAddr->sin_family = AF_INET;
    pAddr->sin_port = PP_HTONS(SOCK_HOST_SERVER_PORT);
    inet_addr_from_ip4addr(&pAddr->sin_addr, ip_2_ip4(&ipAddr));
    EnetAppUtils_assert(addr_ok);

    return;
}


static void open_socket(void *pArg){
        int32_t ret = 0;
        struct sockaddr* pAddr = pArg;
        uint32_t len = 0, buf_len = 0;
        struct timeval opt = {0};
        
        /* create the socket */
        gSock = lwip_socket(pAddr->sa_family, SOCK_DGRAM, 0);
        if (gSock < 0)
        {
            EnetAppUtils_print("ERR: unable to open socket\r\n");
            return;
        }

        /* set recv timeout (100 ms) */
        opt.tv_sec = 0;
        opt.tv_usec = 100 * 1000;
        ret = lwip_setsockopt(gSock, SOL_SOCKET, SO_RCVTIMEO, &opt, sizeof(opt));
        if (ret != 0)
        {
            ret = lwip_close(gSock);
            EnetAppUtils_print("ERR: set sockopt failed\r\n");
            return;
        }

        DebugP_log("Socket %d open\r\n",gSock);

}


int32_t udp_send_data(void *buff, size_t n){
    int32_t ret = 0;

    // FIXME: this is definitely not right but it does seem to work for some reason
    // so leave as is for now
    struct sockaddr *pAddr = &gHostInfo.socketAddr;
    CacheP_wbInv(buff, n, CacheP_TYPE_ALL);
    ret = lwip_sendto(gSock, buff, n, 0, pAddr, sizeof(*pAddr));

    if(ret != n){
        ret = lwip_close(gSock);
        DebugP_log("Socket write failed\r\n");

    }

    return ret;
}

void AppSocket_showMenu(void)
{
    return;
    ip_addr_t ipAddr;
    int32_t addr_ok = 0;
    EnetAppUtils_print(" UDP socket Menu: \r\n");
    // Broadcast for 192.168.1.0/24 
    char *ip = "192.168.1.2";
    ip4addr_aton(ip, ip_2_ip4(&ipAddr));

    return;
    do
    {
        EnetAppUtils_print(" Enter server IPv4 address:(example: 192.168.101.100)\r\n");
        DebugP_scanf("%s", gHostServerIp4);
        addr_ok = ip4addr_aton(gHostServerIp4, ip_2_ip4(&ipAddr));
        TaskP_yield();
    } while (addr_ok != 1);
}


// This should ideally be started in a new thread but for now don't do that
void network_init(void *args){
    int32_t ret = 0;

    EnetAppUtils_enableClocks(ENET_CPSW_2G, 0);

    DebugP_log("Initializing enet\r\n");
    EnetApp_driverInit();

    DebugP_log("Opening enet\r\n");
    ret = EnetApp_driverOpen(ENET_CPSW_2G, 0);
    DebugP_assert(ret == 0);

    DebugP_log("Getting handle...\r\n");
    Enet_Handle handle = Enet_getHandle(ENET_CPSW_2G, 0);
    if(handle == NULL){
        DebugP_logError("Got NULL for handle\r\n");
    }else{
        DebugP_log("Handle is %p\r\n", handle);
    }

    EnetApp_addMCastEntry(ENET_CPSW_2G,
                        0,
                        EnetSoc_getCoreId(),
                        BROADCAST_MAC_ADDRESS,
                        CPSW_ALE_ALL_PORTS_MASK);

    DebugP_log("Setting up network stack\r\n");
    App_setupNetworkStack();


    while (false == App_isNetworkUp(netif_default))
    {
        DebugP_log("Waiting for network UP ...\r\n");
        ClockP_sleep(2);
    }

    DebugP_log("Network is UP!\r\n");

    AppSocket_showMenu();
    DebugP_log("opening socket...\r\n");
    Appsocket_fillHostSocketInfo(&gHostInfo);
    open_socket(&gHostInfo);
    DebugP_log("Socket open\r\n");

    DebugP_log("Sending a test buffer\r\n");

    uint8_t test[8] = {1,2,3,4,5,6,7,8};
    udp_send_data(test, 8);

    DebugP_log("Test buffer sent. Bye from network init\r\n");
  //  vTaskDelete(NULL);
}