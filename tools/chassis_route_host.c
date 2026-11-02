/* Appended to the supplied tests/heading/harness.c in a temporary build.
 * Only UART/plant are mocked. Production route, heading, motion and IMU run.
 */
#include "route.h"
static uint8_t simulation_tx[65536];
static uint32_t simulation_tx_size;
uint8_t RouteUART_Send(const uint8_t *data, uint16_t count)
{
    uint16_t i;
    if (simulation_tx_size + count > sizeof(simulation_tx)) return 0;
    for (i=0; i<count; i++) simulation_tx[simulation_tx_size++]=data[i];
    return 1;
}
void sim_init(void)
{
    sensor_sign=-1;
    init(0);
    Route_Init();
}
void sim_step(uint32_t ms) { advance(ms, 2); }
void sim_feed(const uint8_t *data, uint16_t count) { Route_ReceiveBytes(data, count); }
uint32_t sim_read(uint8_t *data)
{
    uint32_t i, n=simulation_tx_size;
    for (i=0; i<n; i++) data[i]=simulation_tx[i];
    simulation_tx_size=0;
    return n;
}
uint32_t sim_tick(void) { return tick; }
float sim_metric(uint32_t key)
{
    uint8_t i;
    float fastest=0;
    for(i=1;i<=4;i++) if(af(speed[i])>fastest) fastest=af(speed[i]);
    switch(key) {
    case 0:return fastest;
    case 1:return (float)chassis_heading.settled_ms;
    case 2:return true_forward;
    case 3:return true_left;
    case 4:return (float)max_command;
    case 5:return (float)chassis_motion.completed_count;
    case 6:return chassis_heading.translation_rpm;
    case 7:return chassis_heading.remaining_mm;
    case 8:return (float)route_status.state;
    case 9:return (float)velocity_commands;
    case 10:return yaw;
    case 11:return (float)chassis_heading_config.imu_yaw_sign;
    }
    return -1;
}
void sim_fault(uint32_t kind)
{
    if(kind==1) sensor_enabled=0;
    if(kind==2) stall_address=4;
    if(kind==3) Route_Stop();
}
#ifdef SIM_START_AREA
uint32_t sim_select(uint32_t area) { return Route_RequestStartArea((uint8_t)area); }
#endif
