#include <iostream>

#include "qrc.h"
#include "qrc_msg_management.h"
#include "qrc_utils.hpp"
#include "config_msg.h"


/*
This file should get info from the streamlit app and write messages to Tiny Frame

TODO:
1. Send a message from here to mcb and see the output
2. Convert streamlit parameters into streamlit
3. 
*/
// using namespace qti_amr_robot_control;
// using namespace robot_base;


int main() {
    std::cout << "Hello, World!" << std::endl;
    init_qrc_management();
    std::cout << "INITIALIZED" << std::endl;

    // send a message.. 
    struct qrc_pipe_s* pipe_ = qrc_get_pipe(CONFIG_PIPE);
    if (pipe_ == nullptr) {
        std::cerr << "get qrc pipe: " << CONFIG_PIPE << " failed" << std::endl;
        return 1;
    }

    // do we need a type and all that? yes..
    config_tbm_s msg{};
    msg.bottom_motor = 1;
    msg.la = 1;
    msg.stepper = 1;
    msg.top_motor = 1;

    if (!QrcUtils::send_message(pipe_, &msg, sizeof(msg))) {
        std::cerr << "MotionManager: set speed failed" << std::endl;
        return 1;
    }



    return 0;
}
