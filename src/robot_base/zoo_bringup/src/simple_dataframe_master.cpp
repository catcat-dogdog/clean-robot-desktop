#include "simple_dataframe_master.h"
#include "data_holder.h"
#include <stdio.h>

#include "transport.h"

#include "base_driver.h"

Simple_dataframe::Simple_dataframe(Transport* _trans): trans(_trans){
    recv_state = STATE_RECV_FIX;
}

Simple_dataframe::~Simple_dataframe(){
}

bool Simple_dataframe::init(){
    trans->set_timeout(5000);
    return true;
}

bool Simple_dataframe::data_recv(unsigned char c){
    //printf("%02x ", c);
    switch (recv_state){
    case STATE_RECV_FIX:
        if (c == FIX_HEAD1){
            //ROS_INFO("HEAD1 = %d",c);
            memset(&active_rx_msg,0, sizeof(active_rx_msg));
            active_rx_msg.head.head1 = c;
            active_rx_msg.check += c;

            recv_state = STATE_RECV_FIX2;
        }
        else
            recv_state = STATE_RECV_FIX;
        break;
    case STATE_RECV_FIX2:
        if (c == FIX_HEAD2){
            //ROS_INFO("HEAD2 = %d",c);
            active_rx_msg.head.head2 = c;
            active_rx_msg.check += c;
            recv_state = STATE_RECV_ID;
        }
        else
            recv_state = STATE_RECV_FIX;
        break;
    case STATE_RECV_ID:
        if (c < ID_MESSGAE_MAX){
            //ROS_INFO("CMD = %d",c);
            active_rx_msg.head.msg_id = c;
            active_rx_msg.check += c;
            recv_state = STATE_RECV_LEN;
        }
        else
            recv_state = STATE_RECV_FIX;
        break;
    case STATE_RECV_LEN:
        active_rx_msg.head.length =c;
        active_rx_msg.check += c;
        if (active_rx_msg.head.length==0){
            recv_state = STATE_RECV_CHECK;
        }
        else{
            //ROS_INFO("LEN = %d",c);
            recv_state = STATE_RECV_DATA;
        }
            
        break;
    case STATE_RECV_DATA:
		//printf("%02x ", c);
        active_rx_msg.data[active_rx_msg.recv_count++] = c;
        active_rx_msg.check += c;
        //ROS_INFO("DATA = %d",c);
        if (active_rx_msg.recv_count >=active_rx_msg.head.length){
            recv_state  = STATE_RECV_CHECK;
        }
        break;
    case STATE_RECV_CHECK:
        recv_state = STATE_RECV_FIX;
        //ROS_INFO("CHECK = %d",c);
        //ROS_INFO("active_rx_msg.check = %d",active_rx_msg.check);
        if (active_rx_msg.check == c){
            //printf("\r\n");
            return true;
        }
        break;
    default:
        recv_state = STATE_RECV_FIX;
    }

    return false;
}

bool Simple_dataframe::data_parse(){
    MESSAGE_ID id = (MESSAGE_ID)active_rx_msg.head.msg_id;
    Data_holder* dh = Data_holder::get();
    switch (id){
        
    case ID_SET_ROBOT_CHASSIS_TYPE:
        break;
    case ID_CLEAR_ODOM:
        break;
    case ID_SET_VELOCITY:
        break;
    case ID_GET_ODOM:
        memcpy(&dh->odom, active_rx_msg.data, sizeof(dh->odom));
        break;;
    case ID_SINGLE_SERVO:
        break;
    case ID_MULTPLE_SERVO:
        break; 
    case ID_UL_SENSOR:
        memcpy(&dh->ul_sensor, active_rx_msg.data, sizeof(dh->ul_sensor));
        break;
    case ID_SENSRO_STATUS:
        memcpy(&dh->sensor_status, active_rx_msg.data, sizeof(dh->sensor_status));
        break; 
    case ID_GET_MOTOR_ENCODER:
        memcpy(&dh->encoder, active_rx_msg.data, sizeof(dh->encoder));
        break;
    case ID_GET_SERVO_INFO:
        memcpy(&dh->servo_pos, active_rx_msg.data, sizeof(dh->servo_pos));
        break;     
    default:
        break;
    }

    return true;
}

bool Simple_dataframe::send_message(const MESSAGE_ID id){
    Message msg(id);

    send_message(&msg);

    return true;
}

bool Simple_dataframe::send_message(const MESSAGE_ID id, unsigned char* data, unsigned char len){
    Message msg(id, data, len);

    // ROS_INFO("cmd=%d",id);
    send_message(&msg);

    return true;
}

bool Simple_dataframe::send_message(Message* msg){
    if (trans == 0)
        return true;
    
    Buffer data((unsigned char*)msg, (unsigned char*)msg+sizeof(msg->head)+msg->head.length+1);

    // ROS_INFO("---------------------------------------------");
    // for(int i=0; i < data.size(); i++)
    // {
    //     ROS_INFO("data=%d",data[i]);
    // }
    
    trans->write(data);

    return true;
}

bool Simple_dataframe::interact(const MESSAGE_ID id){
	int i;
    //printf("make command:id=%d\r\n", id);

    Data_holder* dh = Data_holder::get();
    switch (id){
    
    case ID_SET_ROBOT_CHASSIS_TYPE:
        send_message(id, (unsigned char*)&dh->chassiss_type, sizeof(dh->chassiss_type));
        break;
    
    case ID_CLEAR_ODOM:
        send_message(id, (unsigned char*)&dh->clear_odom, sizeof(dh->clear_odom));
        break;
    case ID_SET_VELOCITY:
        send_message(id, (unsigned char*)&dh->velocity, sizeof(dh->velocity));
        break;
    case ID_GET_ODOM:
        send_message(id);
        break;
    
    case ID_SINGLE_SERVO:
        send_message(id, (unsigned char*)&dh->single_servo, sizeof(dh->single_servo));
        break;
    case ID_MULTPLE_SERVO:
        send_message(id, (unsigned char*)&dh->multiple_servo, sizeof(dh->multiple_servo));
        break;
    case ID_UL_SENSOR:
        send_message(id);
        break;
    case ID_SENSRO_STATUS:
        send_message(id);
        break;
    case ID_GET_MOTOR_ENCODER:
        send_message(id);
        break;
    case ID_GET_SERVO_INFO:
        send_message(id, (unsigned char*)&dh->servo_ids, sizeof(dh->servo_ids));
        break;
    default:
        break;
    }

    if (!recv_proc())
        return false;

    return true;
}

bool Simple_dataframe::recv_proc(){
    int i=0;
    bool got=false;
    while(true){
        Buffer data = trans->read();

        if(data.size() == 0)
        {
            break;
        }
        
        for (int i=0;i<data.size();i++){
            if (data_recv(data[i])){
                got = true;
                // std::cout << "ok" << std::endl;
                break;
            }
        }

        if (got)
            break;
    }

    if (!data_parse())
        return false;

    return true;
}
