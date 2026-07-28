#include "free_dog_sdk_cpp/unitreeConnectBoost.hpp"

namespace FDSC {
UnitreeConnection::UnitreeConnection(const std::string &settings)
    : sendPort_(sendPortHigh), localIp_(localIpWifi) {
  if (settings == "LOW_WIRED_DEFAULTS") {
    sendPort_ = sendPortLow;
    localIp_ = localIpEth;
    addr_ = addrLow;
  } else if (settings == "LOW_WIFI_DEFAULTS") {
    sendPort_ = sendPortLow;
    localIp_ = localIpWifi;
    addr_ = addrLow;
  } else if (settings == "HIGH_WIRED_DEFAULTS") {
    sendPort_ = sendPortHigh;
    localIp_ = localIpEth;
    addr_ = addrHigh;
  } else if (settings == "SIM_DEFAULTS") {
    sendPort_ = sendPortHigh;
    localIp_ = sim_addr;
    addr_ = sim_addr;
  } else {
    sendPort_ = sendPortHigh;
    localIp_ = localIpWifi;
    addr_ = addrHigh;
  }
  initialize();
}

UnitreeConnection::~UnitreeConnection() {
  stopRecv();
}

void UnitreeConnection::startRecv() {
  runRecv_.store(true);
  recvThread_ = std::thread(&UnitreeConnection::recvThread, this);
  recvThread_.detach(); // Run in the background
}

void UnitreeConnection::stopRecv() {
  runRecv_.store(false);
  io_context->run();
  if (recvThread_.joinable()) {
    recvThread_.join();
  }
}

void UnitreeConnection::send(const std::vector<uint8_t> &cmd) {
  if (socket_ && socket_->is_open()) {
    std::lock_guard<std::mutex> lock(data_mutex);
    // 仿真主控制仍经 ROS/Gazebo 通道工作；此 UDP 旁路在无实体机器狗网段时可能不可达。
    // 不能让可选旁路的发送失败抛出异常并终止 junior_ctrl。
    boost::system::error_code error;
    socket_->send_to(boost::asio::buffer(cmd), *receiver_endpoint_, 0, error);
    if (error) {
      static std::atomic<bool> warned{false};
      if (!warned.exchange(true)) {
        std::cerr << "[WARN] Optional FreeDog UDP send disabled: "
                  << error.message() << '\n';
      }
    }
  }
}

std::vector<std::vector<uint8_t>> UnitreeConnection::getData() {
  std::lock_guard<std::mutex> lock(data_mutex);
  std::vector<std::vector<uint8_t>> ret = data_all;
  data_all.clear();
  return ret;
}

void UnitreeConnection::getData(std::vector<std::vector<uint8_t>> &ret) {
  std::lock_guard<std::mutex> lock(data_mutex);
  ret = data_all;
  data_all.clear();
}

void UnitreeConnection::initialize() {
  io_context = std::make_shared<boost::asio::io_context>();
  socket_ = std::make_shared<boost::asio::ip::udp::socket>(*io_context);
  receiver_endpoint_ = std::make_shared<boost::asio::ip::udp::endpoint>(
      boost::asio::ip::address::from_string(addr_), sendPort_);
  socket_->open(boost::asio::ip::udp::v4());
}

void UnitreeConnection::recvThread() {
  while (runRecv_.load()) {
    std::vector<uint8_t> recv_buffer(2048);
    std::vector<uint8_t> data_;
    boost::system::error_code error;
    try {
      std::size_t bytesRead = socket_->receive_from(
          boost::asio::buffer(recv_buffer), *receiver_endpoint_, 0, error);

      if (error && error != boost::asio::error::message_size) {
        std::cout << "error in udp boost " << std::endl;
        throw boost::system::system_error(error);
      }
      if (bytesRead > 0) {
        recv_buffer.resize(bytesRead);
        data_.insert(data_.end(), recv_buffer.begin(), recv_buffer.end());
        std::lock_guard<std::mutex> lock(data_mutex);
        data_all.push_back(data_);
      }
    }
    catch (const std::exception &e) {
      std::cerr << "Error in receive_from: " << e.what() << '\n';
    }
  }
}
} // namespace FDSC
