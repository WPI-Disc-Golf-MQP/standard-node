/*
 * rosserial Standard Node Interface
 */

#include <Arduino.h>
#include <ros.h>
#include "HardwareSerial.h"
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <functional>
#include <vector>

#ifndef NODE_NAME
    #define NODE_NAME String("std_node")
#endif
#ifndef STATUS_FREQ
    #define STATUS_FREQ 1500 // ms
#endif

enum MODULE_STATUS {
    INITIALIZING_MODULE = 0,
    IDLE = 1,
    IN_PROGRESS = 2,
    COMPLETE = 3,
    INVALID_REPEAT = 4,
    ERROR_REBOOT = 5};

enum REQUEST {
    INITIALIZING = 0,
    STOP = 1,
    START = 2,
    VERIFY_COMPLETE = 3,
    CALIBRATE = 4,
    REBOOT = 5};



HardwareSerial hserial(PA_15, PA_2); // NUCLEO-F303K8 RX, TX
#define Serial1 hserial // This will overwrite the current Serial1 serial port and will use hserial port.
#define USE_STM32_HW_SERIAL
#define __STM32F3xxxx__

ros::NodeHandle nh;
unsigned long last_status = 0;

String get_node_tag() { return String("[")+NODE_NAME+String("] "); };
void loginfo(String msg) { nh.loginfo((get_node_tag()+msg).c_str()); };
void logwarn(String msg) { nh.logwarn((get_node_tag()+msg).c_str()); };
void logerr(String msg) { nh.logerror((get_node_tag()+msg).c_str()); };

struct MODULE {
    String module_name;
    std_msgs::Int8 _state_msg;
    std_msgs::Int8 _status_msg;

    std::function<bool(void)> verify_complete_callback;
    std::function<void(void)> start_callback, idle_callback, calibrate_callback;

    String _state_name;
    String _status_name;
    String _request_name;
    ros::Publisher state_pub;
    ros::Publisher status_pub;
    ros::Subscriber<std_msgs::Int8, MODULE> request_sub;

    std_msgs::Int8 onetime_status;

    MODULE(String _module_name, 
        std::function<void(void)> _start_callback, 
        std::function<bool(void)> _verify_complete_callback, 
        std::function<void(void)> _idle_callback, 
        std::function<void(void)> _calibrate_callback) : 
        _state_name(_module_name + "_state"),
        _status_name(_module_name + "_status"),
        _request_name(_module_name + "_request"),
        state_pub(_state_name.c_str(), &_state_msg), 
        status_pub(_status_name.c_str(), &_status_msg), 
        request_sub(_request_name.c_str(), &MODULE::process_request_callback, this) {
        module_name = _module_name;
        verify_complete_callback = _verify_complete_callback;
        start_callback = _start_callback;
        idle_callback = _idle_callback;
        calibrate_callback = _calibrate_callback;
    }

    void init() {
        nh.advertise(state_pub);
        nh.advertise(status_pub);
        nh.subscribe(request_sub);
    }

    void publish_status(MODULE_STATUS new_status) {
        _status_msg.data = new_status;
        status_pub.publish(&_status_msg);
        if (new_status == MODULE_STATUS::COMPLETE) {
            _status_msg.data = MODULE_STATUS::IDLE;
            status_pub.publish(&_status_msg);
        }
        loginfo("Status Published, "+String(module_name)+", "+String(new_status));
    }

    void publish_state(int new_state) {
        if (_state_msg.data != new_state) {
            _state_msg.data = new_state;
            state_pub.publish(&_state_msg);
        }
    }

    void process_request_callback(const std_msgs::Int8& msg) {
        bool verify_result;
        switch (msg.data) {
            case REQUEST::START:
                _status_msg.data = MODULE_STATUS::IN_PROGRESS; 
                status_pub.publish(&_status_msg);
                start_callback(); 
                break;
            case REQUEST::VERIFY_COMPLETE:
                verify_result = verify_complete_callback();
                onetime_status.data = verify_result ? MODULE_STATUS::COMPLETE : MODULE_STATUS::IN_PROGRESS;
                status_pub.publish(&onetime_status);
                break;
            case REQUEST::STOP:
                _status_msg.data = MODULE_STATUS::IDLE; 
                status_pub.publish(&_status_msg);
                idle_callback();
                break;
            case REQUEST::CALIBRATE:
                _status_msg.data = MODULE_STATUS::IN_PROGRESS;
                status_pub.publish(&_status_msg);
                calibrate_callback();
                break;
            default: 
                return;
        }
        status_pub.publish(&_status_msg);
        loginfo("Request Received, "+String(module_name)+", "+String(msg.data));
    }
};

std::vector<MODULE*> modules;

void init_std_node() {
    nh.initNode();
    nh.setSpinTimeout(100);
    last_status = millis();

    // std::for_each(modules.begin(), modules.end(), [](MODULE &module) {
    //     module.init();
    // });
}

MODULE* init_module(String _module_name, 
        std::function<void(void)> _start_callback, 
        std::function<bool(void)> _verify_complete_callback, 
        std::function<void(void)> _idle_callback, 
        std::function<void(void)> _calibrate_callback) { 
    MODULE* module = new MODULE(_module_name, _start_callback, _verify_complete_callback, _idle_callback, _calibrate_callback);
    modules.emplace_back(module);
    module->init();
    return module;
}
            
void publish_status() {
    std::for_each(modules.begin(), modules.end(), [](MODULE* module) {
        module->status_pub.publish(&module->_status_msg);
    });
    last_status = millis();
}

void periodic_status() {
    if (millis() - last_status > STATUS_FREQ) {
        publish_status();
    }
}

