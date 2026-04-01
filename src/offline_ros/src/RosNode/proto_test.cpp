#include <ros/ros.h>
#include <iostream>
#include <fstream>
// proto自动生成的头文件
#include "test.pb.h"

using namespace std;
using namespace testproto;

int main(int argc, char **argv)
{
    // 1. 初始化ROS
    ros::init(argc, argv, "protobuf_test_node");
    ros::NodeHandle nh;

    ROS_INFO("=== Protobuf 测试程序启动 ===");

    // ==================== 2. 序列化：创建并填充消息 ====================
    TestData send_msg;
    send_msg.set_id(1001);
    send_msg.set_name("ros_protobuf_test");
    send_msg.set_value(99.9f);
    
    // 添加数组数据
    send_msg.add_data(1.1);
    send_msg.add_data(2.2);
    send_msg.add_data(3.3);

    // 序列化为字符串
    string serialize_data;
    send_msg.SerializeToString(&serialize_data);

    ROS_INFO("序列化完成，数据大小：%zu 字节", serialize_data.size());

    // ==================== 3. 反序列化：解析消息 ====================
    TestData recv_msg;
    if (recv_msg.ParseFromString(serialize_data))
    {
        ROS_INFO("反序列化成功！");
        cout << "\n===== 解析结果 =====" << endl;
        cout << "ID:    " << recv_msg.id() << endl;
        cout << "Name:  " << recv_msg.name() << endl;
        cout << "Value: " << recv_msg.value() << endl;
        cout << "Data数组：";
        for (int i = 0; i < recv_msg.data_size(); i++)
        {
            cout << recv_msg.data(i) << " ";
        }
        cout << endl << "====================" << endl;
    }
    else
    {
        ROS_ERROR("反序列化失败！");
        return -1;
    }

    ROS_INFO("=== Protobuf 测试完成 ===");
    return 0;
}