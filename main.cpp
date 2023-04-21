#include "boost_serial.h"

// 将串口的Tx和Rx短接，然后运行这个程序，可以看到接收到的数据

int main() {
    uint8_t write_data[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0xFF};
    size_t write_length = sizeof(write_data) / sizeof(uint8_t);
    
    BoostSerial boost_serial;
    boost_serial.SetReadCallback([](const uint8_t* data, size_t length) {
        printf("Received: ");
        for(int i=0;i<length;i++){
            printf("%02X, ", data[i]);
        }
        printf("\n");
    });
    boost_serial.Open("/dev/ttyUSB1", 9600);

    boost_serial.Write(write_data, write_length);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    boost_serial.Write(write_data, write_length);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    boost_serial.Close();

    return 0;
}