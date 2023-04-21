
#include <boost/asio.hpp>
#include <mutex>
#include <thread>
#include <boost/asio/buffer.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/write.hpp>
#include <boost/bind.hpp>
#include <boost/functional.hpp>

const size_t Boost_Serial_Max_Read_Buf_Len = 50;

class BoostSerial {
private:
    typedef std::function<void(const uint8_t*, const size_t&)> ReadCallback;

public:
    BoostSerial();
    ~BoostSerial();

    void Open(const std::string& port, uint32_t baudrate);
    bool IsOpen() const;
    void Close();
    void SetReadCallback(ReadCallback read_callback);
    size_t Write(const uint8_t* tx_buf, const size_t& len);

private:
    void AsynRead();
    void OnAsioDataRecv(const boost::system::error_code& err, size_t bytes_transferred);

private:
    std::string port_;
    uint32_t baudrate_;
    boost::asio::io_service asio_ios_;
    boost::asio::serial_port asio_port_;
    std::unique_ptr<std::thread> recv_thread_ptr_;
    ReadCallback read_callback_;
    uint8_t asio_read_buf_[Boost_Serial_Max_Read_Buf_Len];
    size_t boost_buffer_len_;
};

BoostSerial::BoostSerial()
        : asio_ios_(),
          asio_port_(asio_ios_),
          recv_thread_ptr_(nullptr),
          read_callback_(nullptr) {
    // printf("[BoostSerial] Constructor\n");
}

BoostSerial::~BoostSerial() {
    if (asio_port_.is_open()) {
        Close();
    }
    // printf("[BoostSerial] Destructor\n");
}

void BoostSerial::Open(const std::string& port, uint32_t baudrate) {
    port_ = port;
    baudrate_ = baudrate;
    if (asio_port_.is_open()) {
        printf("[Serial] Port %s is already open.\n", port_.c_str());
        return;
    }
    // configure boost::asio::serial_port
    printf("[Serial] Open port %s with baudrate %d\n", port_.c_str(), baudrate_);
    try {
        asio_port_.open(port_);
    } catch (...) {
    }
    if (!asio_port_.is_open()) {
        printf("[Serial] Failed to open port %s.\n", port_.c_str());
        return;
    }
    // set options for serial port
    asio_port_.set_option(boost::asio::serial_port::baud_rate(baudrate_));
    asio_port_.set_option(boost::asio::serial_port::character_size(8));
    asio_port_.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
    asio_port_.set_option(
            boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
    asio_port_.set_option(
            boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
    // push the first read request
    AsynRead();
    // create thread that will read incoming data asynchronously
    recv_thread_ptr_.reset(new std::thread([this]() {
        this->asio_ios_.run();
        printf("[Serial] Exit receive thread\n");
    }));
}

bool BoostSerial::IsOpen() const {
    return asio_port_.is_open();
}

void BoostSerial::Close() {
    if (!asio_port_.is_open()) {
        printf("[Serial] Port %s is already close\n", port_.c_str());
        return;
    }
    printf("[Serial] Close serial port %s\n", port_.c_str());
    // finish async read thread and delete it
    asio_port_.cancel();  // cancel pending async processes
    asio_ios_.stop();     // stop io_service
    if (recv_thread_ptr_.get() != nullptr) {
        recv_thread_ptr_->join();         // join thread
        recv_thread_ptr_.reset(nullptr);  // delete thread
    }
    asio_ios_.reset();              // reset io_service for reopening
    asio_port_.close();             // close serial port
}

void BoostSerial::SetReadCallback(ReadCallback read_callback) {
    if (read_callback_) {
        printf("[Serial] Read callback is already set\n");
    } else {
        read_callback_ = read_callback;
        printf("[Serial] Set read callback\n");
    }
}

size_t BoostSerial::Write(const uint8_t* tx_buf, const size_t& len) {
    if (!asio_port_.is_open()) {
        printf("[Serial] Port %s is not open, write fail.\n", port_.c_str());
        return 0;
    }
    return boost::asio::write(asio_port_, boost::asio::buffer(tx_buf, len));
}

void BoostSerial::AsynRead() {
    // reads a certain amount of data before the asynchronous operation completes
    boost_buffer_len_ = 2;
    boost::asio::async_read(asio_port_, boost::asio::buffer(asio_read_buf_, boost_buffer_len_), 
        boost::bind(&BoostSerial::OnAsioDataRecv, this,boost::placeholders::_1, boost::placeholders::_2));
}

void BoostSerial::OnAsioDataRecv(const boost::system::error_code& error, size_t bytes_transferred) {
    if (error) {
        std::string err_msg = std::string("[Serial] %%%%%%%%% serial - ") +
                              std::to_string(error.value()) + std::string(" - ") + error.message();
        printf("%s\n", err_msg.c_str());
        return;
    }

    if (bytes_transferred > 0 && read_callback_) {
        read_callback_(asio_read_buf_, bytes_transferred);
    }
    // asyn read again
    AsynRead();
}

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
    boost_serial.Open("/dev/ttyUSB0", 9600);

    boost_serial.Write(write_data, write_length);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    boost_serial.Write(write_data, write_length);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    boost_serial.Close();

    return 0;
}