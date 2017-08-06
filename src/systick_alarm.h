#ifndef SYSTICK_ALARM_H
#define SYSTICK_ALARM_H
#define SYSTICK_RELOAD_PERIOD (8999u)
void systick_setup(void);
int is_systick_timeout(void);
void set_systick_timeout(uint32_t timeout);
#endif
