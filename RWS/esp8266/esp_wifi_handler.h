/*
 * esp_wifi_handler.h
 *
 * Created: 10/17/2017 10:15:46 PM
 *  Author: Gaal Alexandru
 */ 

#ifndef WIFI_HANDLER_H_
#define WIFI_HANDLER_H_

void esp_init(void);
void esp_check_current_setup(void);
void esp_wifi_setup(void);
void esp_state_machine(void);

#endif /* WIFI_HANDLER_H_ */