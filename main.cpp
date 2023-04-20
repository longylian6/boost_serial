#include <iostream>
#include <string>
#include <functional>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/write.hpp>
#include <boost/functional.hpp>
#include <boost/thread.hpp>

class BoostSerial {
public:
    BoostSerial() : asio_ios_(), asio_port_(asio_ios_), timer_(asio_ios_), 
                    is_open_(false), callback_(nullptr) {}
    ~BoostSerial() { close(); }

    void open(const std::string& port_name, size_t baud_rate) {
        if (is_open_) {
            printf("Port %s is already open\n", port_name.c_str());
            return;
        }
        printf("Opening port %s\n", port_name.c_str());
        try {
            asio_port_.open(port_name);
        } catch (boost::system::system_error& e) {
            printf("Error opening port %s: %s\n", port_name.c_str(), e.what());
            return;
        }
        if (!asio_port_.is_open()) {
            printf("Error opening port %s\n", port_name.c_str());
            return;
        }
        is_open_ = true;
        asio_port_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
        asio_port_.set_option(boost::asio::serial_port_base::character_size(8));
        asio_port_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        asio_port_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        asio_port_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

        doRead();
    }

    inline bool isOpen() const { return is_open_; }

    void close() {
        if (!is_open_) return;
        
        asio_port_.cancel();
        asio_ios_.stop();

        asio_ios_.reset();
        asio_port_.close();

        is_open_ = false;
    }

    void setCallback(std::function<void(const uint8_t*, size_t)> callback) {
        callback_ = callback;
    }

    void run() {
        if (!is_open_) return;
        asio_ios_.run();
    }

    bool write(const uint8_t* data, size_t length) {
        if (!is_open_) return false;
        boost::asio::write(asio_port_, boost::asio::buffer(data, length));
        printf("Sent: ");
        for(int i=0;i<length;i++){
            printf("%02X, ", data[i]);
        }
        printf("\n");
        return true;
    }

private:
    boost::asio::io_service asio_ios_;
    boost::asio::serial_port asio_port_;
    boost::asio::deadline_timer timer_;
    bool is_open_;
    enum { max_read_length = 1024 };
    uint8_t data_[max_read_length];
    std::function<void(const uint8_t*, size_t)> callback_;

private:
    void doRead() {
        if (!is_open_) return;
        boost::asio::async_read(asio_port_, boost::asio::buffer(data_, max_read_length), 
            boost::bind(&BoostSerial::readCallback, this, boost::placeholders::_1, boost::placeholders::_2));
    }

    bool readComplete(const boost::system::error_code& error, size_t bytes_transferred) {
        if (error) {
            printf("Error reading serial port: %s in %s\n", error.message().c_str(), __func__);
            return false;
        }
        timer_.expires_from_now(boost::posix_time::milliseconds(10));
        timer_.async_wait(boost::bind(&BoostSerial::doRead, this));
        return true;
    }

    void readCallback(const boost::system::error_code& error, size_t bytes_transferred) {
        if (error) {
            printf("Error reading serial port: %s in %s\n", error.message().c_str(), __func__);
            return;
        }
        if (bytes_transferred > 0 && callback_) {
            callback_(data_, bytes_transferred);
        }
        doRead();
    }
};

int main() {
    uint8_t write_data[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0xFF};
    size_t write_length = sizeof(write_data) / sizeof(uint8_t);

    BoostSerial boost_serial;
    boost_serial.setCallback([](const uint8_t* data, size_t length) {
        printf("Received: ");
        for(int i=0;i<length;i++){
            printf("%02X, ", data[i]);
        }
        printf("\n");
    });

    boost_serial.open("/dev/ttyUSB0", 9600);
    if (!boost_serial.isOpen()) {
        printf("Error opening serial port\n");
        return 1;
    }

    boost::thread thread([&boost_serial]() {
        printf("Boost serial thread started\n");
        boost_serial.run();
        printf("Boost serial thread finished\n");
    });

    boost::this_thread::sleep(boost::posix_time::milliseconds(500));
    
    boost_serial.write(write_data, write_length);

    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));

    boost_serial.close();

    return 0;
}