#include <iostream>

#include "qrc_msg_management.h"
#include "keybpwm_msg.h"
#include "motor_control.hpp"

#include <pybind11/pybind11.h>

namespace py = pybind11;

static void keybpwm_msg_parse(struct qrc_pipe_s *pipe, struct pwm_msg_s *msg);
static void keybpwm_qrc_msg_cb(struct qrc_pipe_s *pipe,void * data, size_t len, bool response);


// int add(int i, int j){
// 	return i +j;
// }

// PYBIND11_MODULE(motor_control, m) {
//     m.doc() = "pybind11 example plugin"; // optional module docstring

//     m.def("add", &add, "A function that adds two numbers");
// }

PYBIND11_MODULE(motor_control, m) {
	py::class_<MotorControl>(m,"MotorControl")
		.def(py::init<>())
		.def("sendMotorMessage", &MotorControl::sendMotorMessage);
}


/*qrc message callback*/
static void keybpwm_qrc_msg_cb(struct qrc_pipe_s *pipe,void * data, size_t len, bool response)
{
	struct pwm_msg_s *msg;

	if (pipe == NULL || data ==NULL)
		{
			return;
		}
	if (len == sizeof(struct pwm_msg_s))
		{
			msg = (struct pwm_msg_s *)data;
			keybpwm_msg_parse(pipe, msg);
		}
	 else
	 {
		printf("ERROR: keybpwm_qrc_msg_cb msg size mismatch \n");
	 }
}

/* you can modify this function to fit your requirement */
static void keybpwm_msg_parse(struct qrc_pipe_s *pipe, struct pwm_msg_s *msg)
{
	struct pwm_msg_s reply_msg;

	// switch (msg->type)
	// 	{
	// 	case GET_HELLO:
	// 		{
	// 		//send hello to RB5
	// 		memcpy(reply_msg.data, "hello, RB5", strlen("hello, RB5") * sizeof(char));
	// 		reply_msg.type = PRINT_HELLO;
	// 		qrc_write(pipe, (uint8_t *)&reply_msg, sizeof(struct pwm_msg_s), false);
	// 		break;
	// 		}
	// 	case PRINT_HELLO:
	// 		{
	// 			printf("\n Get message, type is PRINT_HELLO : (%s) \n", msg->data);
	// 		break;
	// 		}
	// 	case SET_VALUE:
	// 		{
	// 			printf("\n Get message, type is SET_VALUE : (%d) \n", msg->value);
	// 		break;
	// 		}
	// 	default:
	// 		printf("keybpwm_msg_parse: unknow message type \n");
	// }
}

MotorControl::~MotorControl(){
	std::cout << "Destructor" << std::endl;
	deinit_qrc_management();
}

bool MotorControl::init(){
	std::cout << "Actually init" << std::endl;
	if (!init_qrc_management()){
		std::cout << "init_qrc_management failed" << std::endl;
		return false;
	}
	std::cout << "Initialized QRC" << std::endl;

	char pipe_name[] = PWM_PIPE;
	// struct qrc_pipe_s *pipe;
	
	m_pipe =  qrc_get_pipe(pipe_name);
	if (m_pipe == NULL){
		std::cout << "qrc_get_pipe failed" << std::endl;
		return false;
	}

	/* register msg callback */
	if (!qrc_register_message_cb(m_pipe, keybpwm_qrc_msg_cb)){
		std::cout << "qrc_register_message_cb failed" << std::endl;
		return -1;
	}
	return true;
}

void MotorControl::sendMotorMessage(int motor, int on_off, double duty, double frequency){
	struct pwm_msg_s msg;
	msg.type = SET_MOTOR;
	msg.motor = motor;
	msg.on_off = on_off;
	msg.duty = duty;
	msg.freq = frequency;
	qrc_write(m_pipe, (uint8_t *)&msg, sizeof(struct pwm_msg_s), false);
	std::cout << "Sent command to motor: " << msg.motor << std::endl;
}

MotorControl::MotorControl(){
	init();
}

int main() {

	//this should all be cpp test code.. no core implementation can exist here

	std::cout << "Started main.cpp" << std::endl;
	// if (!init_qrc_management()){
	// 	std::cout << "init_qrc_management failed" << std::endl;
	// }
	// std::cout << "Initialized QRC" << std::endl;
	MotorControl mc;

	sleep(3);

	mc.sendMotorMessage(1,1,5.33223,1000.4995);

	sleep(3);
	// char pipe_name[] = PWM_PIPE; //does this need to be initialized somewhere else
	// struct qrc_pipe_s *pipe;
	
	// pipe =  qrc_get_pipe(pipe_name);
	// if (pipe == NULL){
	// 	std::cout << "qrc_get_pipe failed" << std::endl;
	// 	return -1;
	// }

	// /* register msg callback */
	// if (!qrc_register_message_cb(pipe, keybpwm_qrc_msg_cb)){
	// 	std::cout << "qrc_register_message_cb failed" << std::endl;
	// 	return -1;
	// }



	// sleep(3);
	// struct pwm_msg_s msg;
	// msg.type = SET_MOTOR;
	// msg.motor = 1;
	// msg.on_off = 1;
	// msg.duty = 5.33223;
	// msg.freq = 1000.45;
	// qrc_write(pipe, (uint8_t *)&msg, sizeof(struct pwm_msg_s), false);
	// std::cout << "Sent command to motor: " << msg.motor << std::endl;


	// sleep(5);

	/*
	sendMotorMessage(motor,on_off,duty,freq)
	
	*/


	// msg.type = PRINT_HELLO;
	// memset(msg.data, '\0', 32);
	// memcpy(msg.data, "hello,this is RB5", strlen("hello,this is RB5") * sizeof(char));
	// qrc_write(pipe, (uint8_t *)&msg, sizeof(struct pwm_msg_s), false);
	// std::cout << "send message type  PRINT_HELLO done " << std::endl;
	
	// sleep(3);
	// msg.type = GET_HELLO;
	// qrc_write(pipe, (uint8_t *)&msg, sizeof(struct pwm_msg_s), false);
	// std::cout << "send message type  GET_HELLO done " << std::endl;
	
	// sleep(3);
	// msg.type = SET_VALUE;
	// msg.value = 1111;
	// qrc_write(pipe, (uint8_t *)&msg, sizeof(struct pwm_msg_s), false);
	// std::cout << "send message type  SET_VALUE done "<<"value ="<<msg.value << std::endl;

	// sleep(3);
	// msg.type = SET_VALUE;
	// msg.value = 444;
	// memset(msg.data, '\0', 32);
	// memcpy(msg.data, "new message", strlen("new message") * sizeof(char));
	// qrc_write(pipe, (uint8_t *)&msg, sizeof(struct pwm_msg_s), false);
	// std::cout << "send message type newwww SET_VALUE done "<<"value ="<<msg.value << std::endl;



	// deinit_qrc_management();

	return 0;
}
