#include <iostream>
#include <string>
#include <functional>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/write.hpp>
#include <boost/functional.hpp>

class SerialPortReader {
public:
    SerialPortReader(boost::asio::io_service& io, const std::string& port_name, size_t baud_rate) :
            port_(io, port_name), timer_(io), connected_(false) {
        port_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
        port_.set_option(boost::asio::serial_port_base::character_size(8));
        port_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        port_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        port_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    }

    void connect(std::function<void(const uint8_t*, size_t)> callback) {
        callback_ = callback;
        do_read();
    }

private:
    boost::asio::serial_port port_;
    boost::asio::deadline_timer timer_;
    enum { max_read_length = 1024 };
    char data_[max_read_length];
    std::function<void(const uint8_t*, size_t)> callback_;
    bool connected_;

private:
    void do_read() {
        if (!connected_) return;
        boost::asio::async_read(port_, boost::asio::buffer(data_, max_read_length), 
                boost::bind(&SerialPortReader::read_complete, this, boost::placeholders::_1, boost::placeholders::_2), 
                boost::bind(&SerialPortReader::read_callback, this, boost::placeholders::_1, boost::placeholders::_2));
    }

    bool read_complete(const boost::system::error_code& error, size_t bytes_transferred) {
        if (error || !connected_) return false;
        timer_.expires_from_now(boost::posix_time::milliseconds(10));
        timer_.async_wait(boost::bind(&SerialPortReader::do_read, this));
        return false;
    }

    void read_callback(const boost::system::error_code& error, size_t bytes_transferred) {
        if (error || !connected_) return;

        uint8_t* data_copy = new uint8_t[bytes_transferred];
        std::copy(data_, data_ + bytes_transferred, data_copy);

        callback_(data_copy, bytes_transferred);
        do_read();

        delete[] data_copy;
    }
};

int main() {
    boost::asio::io_service io;
    SerialPortReader reader(io, "/dev/ttyUSB0", 9600);
    reader.connect([](const uint8_t* data, size_t length) {
        std::cout << "Received: ";
        for(int i=0;i<length;i++){
            std::cout << std::hex << int(data[i]) << "_";
        }
        std::cout << std::endl;
    });
    io.run();
    return 0;
}