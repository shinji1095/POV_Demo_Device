/**
 * Copyright 2021 (C) Hailo Technologies Ltd.
 * All rights reserved.
 *
 * Hailo Technologies Ltd. ("Hailo") disclaims any warranties, including, but not limited to,
 * the implied warranties of merchantability and fitness for a particular purpose.
 * This software is provided on an "AS IS" basis, and Hailo has no obligation to provide maintenance,
 * support, updates, enhancements, or modifications.
 *
 * You may use this software in the development of any project.
 * You shall not reproduce, modify or distribute this software without prior written permission.
 **/

 #include <rclcpp/rclcpp.hpp>
 #include <sensor_msgs/msg/image.hpp>
 #include <opencv2/opencv.hpp>
 #include <cv_bridge/cv_bridge.hpp>
 
 #include "hailo/hailort.hpp"
 #include "pov_hailo8_node/pov_labels.hpp"
 #include "pov_demo_msg/msg/pov_result.hpp"
 
 constexpr int WIDTH = 640;
 constexpr int HEIGHT = 640;
 
 using hailort::Device;
 using hailort::Hef;
 using hailort::Expected;
 using hailort::make_unexpected;
 using hailort::ConfiguredNetworkGroup;
 using hailort::VStreamsBuilder;
 using hailort::InputVStream;
 using hailort::OutputVStream;
 using hailort::MemoryView;
 using hailort::ActivatedNetworkGroup;
 
 class HailoClassifierNode : public rclcpp::Node {
 public:
     HailoClassifierNode() 
         : Node("hailo_classifier_node"), 
           device_(nullptr), 
           network_group_(nullptr) {
         // パラメータの宣言と取得
         declare_parameter<std::string>("hef_file", "");
         hef_file_ = get_parameter("hef_file").as_string();
         if (hef_file_.empty()) {
             RCLCPP_ERROR(this->get_logger(), "HEF file path is not provided.");
             throw std::runtime_error("HEF file path is required.");
         }
 
         // Hailo-8デバイスの初期化
         initialize_hailo();
 
         // ROS 2サブスクライバとパブリッシャの設定
         image_sub_ = create_subscription<sensor_msgs::msg::Image>(
             "/camera/camera/color/image_raw", 10,
             std::bind(&HailoClassifierNode::image_callback, this, std::placeholders::_1));
 
         result_pub_ = create_publisher<pov_demo_msg::msg::PovResult>(
             "/pov_demo/hailo8/results", 10);
 
         RCLCPP_INFO(this->get_logger(), "Hailo Classifier Node initialized.");
     }
 
 private:
     void initialize_hailo() {
         // PCIeデバイスのスキャン
         auto all_devices = Device::scan_pcie();
         if (!all_devices || all_devices->empty()) {
             RCLCPP_ERROR(this->get_logger(), "Failed to scan PCIe devices: %d", all_devices.status());
             throw std::runtime_error("No PCIe devices found.");
         }
 
         // デバイスの作成
         auto device = Device::create_pcie(all_devices.value()[0]);
         if (!device) {
             RCLCPP_ERROR(this->get_logger(), "Failed to create PCIe device: %d", device.status());
             throw std::runtime_error("Device creation failed.");
         }
         device_ = std::move(device.value());
 
         // ネットワークグループの設定
         auto network_group = configure_network_group(*device_, hef_file_);
         if (!network_group) {
             RCLCPP_ERROR(this->get_logger(), "Failed to configure network group: %d", network_group.status());
             throw std::runtime_error("Network group configuration failed.");
         }
         network_group_ = std::move(network_group.value());
 
         // 仮想ストリームの設定
         auto input_vstream_params = network_group_->make_input_vstream_params(
             true, HAILO_FORMAT_TYPE_UINT8, HAILO_DEFAULT_VSTREAM_TIMEOUT_MS, HAILO_DEFAULT_VSTREAM_QUEUE_SIZE);
         auto output_vstream_params = network_group_->make_output_vstream_params(
             false, HAILO_FORMAT_TYPE_FLOAT32, HAILO_DEFAULT_VSTREAM_TIMEOUT_MS, HAILO_DEFAULT_VSTREAM_QUEUE_SIZE);
 
         auto input_vstreams = VStreamsBuilder::create_input_vstreams(*network_group_, input_vstream_params.value());
         auto output_vstreams = VStreamsBuilder::create_output_vstreams(*network_group_, output_vstream_params.value());
 
         if (!input_vstreams || !output_vstreams) {
             RCLCPP_ERROR(this->get_logger(), "Failed to create vstreams: input=%d, output=%d",
                          input_vstreams.status(), output_vstreams.status());
             throw std::runtime_error("VStream creation failed.");
         }
 
         vstreams_ = std::make_pair(input_vstreams.release(), output_vstreams.release());
 
         // ネットワークグループのアクティベート
         auto activated_network_group = network_group_->activate();
         if (!activated_network_group) {
             RCLCPP_ERROR(this->get_logger(), "Failed to activate network group: %d", activated_network_group.status());
             throw std::runtime_error("Network group activation failed.");
         }
         activated_network_group_ = std::move(activated_network_group.release());
 
         print_net_banner(vstreams_);
     }
 
     Expected<std::shared_ptr<ConfiguredNetworkGroup>> configure_network_group(Device &device, const std::string &hef_file) {
         auto hef = Hef::create(hef_file);
         if (!hef) {
             return make_unexpected(hef.status());
         }
 
         auto configure_params = hef->create_configure_params(HAILO_STREAM_INTERFACE_PCIE);
         if (!configure_params) {
             return make_unexpected(configure_params.status());
         }
 
         auto network_groups = device.configure(hef.value(), configure_params.value());
         if (!network_groups || network_groups->size() != 1) {
             return make_unexpected(HAILO_INTERNAL_FAILURE);
         }
 
         return std::move(network_groups->at(0));
     }
 
     void print_net_banner(std::pair<std::vector<InputVStream>, std::vector<OutputVStream>> &vstreams) {
         RCLCPP_INFO(this->get_logger(), "---------------------------------------------------------------");
         RCLCPP_INFO(this->get_logger(), "Dir  Name");
         RCLCPP_INFO(this->get_logger(), "---------------------------------------------------------------");
         for (auto &value : vstreams.first) {
             RCLCPP_INFO(this->get_logger(), "IN:  %s", info_to_str<InputVStream>(value).c_str());
         }
         RCLCPP_INFO(this->get_logger(), "---------------------------------------------------------------");
         for (auto &value : vstreams.second) {
             RCLCPP_INFO(this->get_logger(), "OUT: %s", info_to_str<OutputVStream>(value).c_str());
         }
         RCLCPP_INFO(this->get_logger(), "---------------------------------------------------------------");
     }
 
     template <typename T>
     std::string info_to_str(T &stream) {
         std::string result = stream.get_info().name;
         result += " (";
         result += std::to_string(stream.get_info().shape.height) + ", ";
         result += std::to_string(stream.get_info().shape.width) + ", ";
         result += std::to_string(stream.get_info().shape.features) + ")";
         return result;
     }
 
     void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
         try {
             // 画像をOpenCV形式に変換
             cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "rgb8");
             cv::Mat rgb_frame = cv_ptr->image;
 
             // 画像のリサイズ
             if (rgb_frame.rows != HEIGHT || rgb_frame.cols != WIDTH) {
                 cv::resize(rgb_frame, rgb_frame, cv::Size(WIDTH, HEIGHT), cv::INTER_AREA);
             }
 
             // 推論の実行
             auto status = infer(rgb_frame);
             if (status != HAILO_SUCCESS) {
                 RCLCPP_ERROR(this->get_logger(), "Inference failed: %d", status);
                 return;
             }
         } catch (const cv_bridge::Exception &e) {
             RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
         }
     }
 
     hailo_status infer(cv::Mat &rgb_frame) {
         // 入力データの書き込み
         auto input_status = vstreams_.first[0].write(MemoryView(rgb_frame.data, HEIGHT * WIDTH * 3));
         if (input_status != HAILO_SUCCESS) {
             return input_status;
         }
 
         // 出力データの読み込み
         std::vector<float> data(vstreams_.second[0].get_frame_size());
         auto output_status = vstreams_.second[0].read(MemoryView(data.data(), data.size()));
         if (output_status != HAILO_SUCCESS) {
             return output_status;
         }
 
         // 後処理と結果のパブリッシュ
         auto result = classification_post_process<float>(data);
         RCLCPP_INFO(this->get_logger(), "Detected class: %s, prob: %.6f", result.class_name.c_str(), result.prob);
 
         result_pub_->publish(result);
 
         return HAILO_SUCCESS;
     }
 
     template <typename T>
     pov_demo_msg::msg::PovResult classification_post_process(std::vector<T> &logits, bool do_softmax = false, float threshold = 0.3) {
        pov_demo_msg::msg::PovResult result;
         int max_idx;
         static PovLabels obj;
         std::vector<T> softmax_result(logits);
         if (do_softmax) {
             softmax_result = softmax(logits);
             max_idx = argmax(softmax_result);
         } else {
             max_idx = argmax(logits);
         }
         if (softmax_result[max_idx] < threshold) {
             result.class_name = "N/A";
             result.prob = 0.0;
         } else {
             result.class_name = obj.pov_labelstring(max_idx);
             result.prob = softmax_result[max_idx];
         }
         return result;
     }
 
     template <typename T, typename A>
     int argmax(std::vector<T, A> const &vec) {
         return static_cast<int>(std::distance(vec.begin(), std::max_element(vec.begin(), vec.end())));
     }
 
     template <typename T, typename A>
     std::vector<T, A> softmax(std::vector<T, A> const &vec) {
         std::vector<T, A> result;
         float m = -INFINITY;
         float sum = 0.0;
 
         for (const auto &val : vec) m = (val > m) ? val : m;
         for (const auto &val : vec) sum += expf(val - m);
         for (const auto &val : vec) result.push_back(expf(val - m) / sum);
 
         return result;
     }
 
     std::unique_ptr<Device> device_;
     std::shared_ptr<ConfiguredNetworkGroup> network_group_;
     std::pair<std::vector<InputVStream>, std::vector<OutputVStream>> vstreams_;
     std::unique_ptr<ActivatedNetworkGroup> activated_network_group_;
     std::string hef_file_;
     rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
     rclcpp::Publisher<pov_demo_msg::msg::PovResult>::SharedPtr result_pub_;
 };
 
 int main(int argc, char *argv[]) {
     rclcpp::init(argc, argv);
     try {
         auto node = std::make_shared<HailoClassifierNode>();
         rclcpp::spin(node);
     } catch (const std::exception &e) {
         std::cerr << "Error: " << e.what() << std::endl;
         rclcpp::shutdown();
         return 1;
     }
     rclcpp::shutdown();
     return 0;
 }