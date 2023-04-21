#ifndef BOOST_SERIAL_H
#define BOOST_SERIAL_H

#include <boost/asio.hpp>
#include <thread>

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
    void OnAsioDataRecv(const boost::system::error_code& error, size_t bytes_transferred);

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

#endif  // BOOST_SERIAL_H