
#pragma once

#include <time.h>
#include "ros/node.h"
#include "pr2_power_board/PowerBoardCommand.h"
#include "rosthread/mutex.h"

class Interface 
{
  public:

    struct ifreq interface;
    int recv_sock;
    int send_sock;
    Interface(const char* ifname);
    ~Interface() {Close();}
    void Close();
    int Init(sockaddr_in *port_address, sockaddr_in *broadcast_address);
    int InitReceive();
    void AddToReadSet(fd_set &set, int &max_sock) const;
    bool IsReadSet(fd_set set) const;
    sockaddr_in ifc_address;
};


class Device 
{
  public:
    time_t message_time;
    PowerMessage pmsg;  //last power message recived from device
    TransitionMessage tmsg;
    Device() {};
    ~Device() { };
};


class PowerBoard : public ros::node
{
  public:
    PowerBoard();
    bool commandCallback( pr2_power_board::PowerBoardCommand::request &req_,
                          pr2_power_board::PowerBoardCommand::response &res_);

    void collectMessages();
    void sendDiagnostic();
    int collect_messages();
    int process_message(const PowerMessage *msg);
    int process_transition_message(const TransitionMessage *msg);
    const char* master_state_to_str(char state);
    const char* cb_state_to_str(char state);
    int list_devices(void);
    int send_command(const char* input) ;

  private:
    pr2_power_board::PowerBoardCommand::request req_;
    pr2_power_board::PowerBoardCommand::response res_;
    ros::thread::mutex library_lock_;
};
