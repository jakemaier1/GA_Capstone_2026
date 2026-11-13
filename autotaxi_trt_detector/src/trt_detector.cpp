#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

class TrtLogger : public nvinfer1::ILogger
{
public:
  void log(Severity severity, const char * msg) noexcept override
  {
    if (severity <= Severity::kWARNING) {
      std::cerr << "[TensorRT] " << msg << std::endl;
    }
  }
};

template<typename T>
struct TrtDeleter
{
  void operator()(T * object) const
  {
    if (object != nullptr) {
      object->destroy();
    }
  }
};

template<typename T>
using TrtPtr = std::unique_ptr<T, TrtDeleter<T>>;

static void check_cuda(cudaError_t result, const std::string & action)
{
  if (result != cudaSuccess) {
    throw std::runtime_error(
      action + ": " + std::string(cudaGetErrorString(result)));
  }
}

class TrtDetector : public rclcpp::Node
{
public:
  TrtDetector()
  : Node("trt_detector")
  {
    model_path_ = declare_parameter<std::string>(
      "model_path",
      "/root/share/GA_Capstone_2026/cv/airport_fp16.engine");

    camera_topic_ = declare_parameter<std::string>(
      "camera_topic",
      "/ascamera_hp60c/camera_publisher/rgb0/image");

    confidence_threshold_ = declare_parameter<double>(
      "confidence_threshold", 0.35);

    nms_threshold_ = declare_parameter<double>(
      "nms_threshold", 0.45);

    load_engine();

    auto qos = rclcpp::SensorDataQoS();
    qos.keep_last(1);

    subscription_ = create_subscription<sensor_msgs::msg::Image>(
      camera_topic_,
      qos,
      std::bind(&TrtDetector::image_callback, this, std::placeholders::_1));

    last_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(
      get_logger(),
      "TensorRT detector ready. Listening on: %s",
      camera_topic_.c_str());
  }

  ~TrtDetector() override
  {
    if (stream_ != nullptr) {
      cudaStreamDestroy(stream_);
    }

    if (device_input_ != nullptr) {
      cudaFree(device_input_);
    }

    if (device_output_ != nullptr) {
      cudaFree(device_output_);
    }

    cv::destroyAllWindows();
  }

private:
  void load_engine()
  {
    std::ifstream file(model_path_, std::ios::binary);

    if (!file.good()) {
      throw std::runtime_error("Could not open engine: " + model_path_);
    }

    file.seekg(0, std::ios::end);
    const size_t size = static_cast<size_t>(file.tellg());
    file.seekg(0, std::ios::beg);

    std::vector<char> engine_data(size);
    file.read(engine_data.data(), size);

    runtime_.reset(nvinfer1::createInferRuntime(logger_));

    if (!runtime_) {
      throw std::runtime_error("Could not create TensorRT runtime.");
    }

    engine_.reset(runtime_->deserializeCudaEngine(
      engine_data.data(), size));

    if (!engine_) {
      throw std::runtime_error("Could not load TensorRT engine.");
    }

    context_.reset(engine_->createExecutionContext());

    if (!context_) {
      throw std::runtime_error("Could not create TensorRT context.");
    }

    for (int i = 0; i < engine_->getNbBindings(); ++i) {
      if (engine_->bindingIsInput(i)) {
        input_index_ = i;
      } else {
        output_index_ = i;
      }
    }

    if (input_index_ < 0 || output_index_ < 0) {
      throw std::runtime_error("TensorRT engine bindings were not found.");
    }

    const auto input_dims = engine_->getBindingDimensions(input_index_);
    const auto output_dims = engine_->getBindingDimensions(output_index_);

    if (input_dims.nbDims != 4 || output_dims.nbDims != 3) {
      throw std::runtime_error("Unexpected TensorRT input/output shape.");
    }

    input_height_ = input_dims.d[2];
    input_width_ = input_dims.d[3];

    output_channels_ = output_dims.d[1];
    output_candidates_ = output_dims.d[2];

    input_elements_ = 1 * input_dims.d[1] * input_height_ * input_width_;
    output_elements_ = output_channels_ * output_candidates_;

    bindings_.resize(engine_->getNbBindings(), nullptr);

    check_cuda(
      cudaMalloc(
        reinterpret_cast<void **>(&device_input_),
        input_elements_ * sizeof(float)),
      "Could not allocate GPU input memory");

    check_cuda(
      cudaMalloc(
        reinterpret_cast<void **>(&device_output_),
        output_elements_ * sizeof(float)),
      "Could not allocate GPU output memory");

    check_cuda(cudaStreamCreate(&stream_), "Could not create CUDA stream");

    bindings_[input_index_] = device_input_;
    bindings_[output_index_] = device_output_;

    RCLCPP_INFO(
      get_logger(),
      "Loaded TensorRT engine: %dx%d input, %d classes, %d candidates",
      input_width_,
      input_height_,
      output_channels_ - 4,
      output_candidates_);
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr message)
  {
    try {
      auto cv_image = cv_bridge::toCvCopy(
        message,
        sensor_msgs::image_encodings::BGR8);

      detect_and_display(cv_image->image);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Detection error: %s", error.what());
    }
  }

  void detect_and_display(const cv::Mat & frame)
  {
    const float scale = std::min(
      static_cast<float>(input_width_) / frame.cols,
      static_cast<float>(input_height_) / frame.rows);

    const int resized_width = static_cast<int>(std::round(frame.cols * scale));
    const int resized_height = static_cast<int>(std::round(frame.rows * scale));

    const int pad_x = (input_width_ - resized_width) / 2;
    const int pad_y = (input_height_ - resized_height) / 2;

    cv::Mat resized;
    cv::resize(frame, resized, cv::Size(resized_width, resized_height));

    cv::Mat letterboxed(
      input_height_,
      input_width_,
      CV_8UC3,
      cv::Scalar(114, 114, 114));

    resized.copyTo(
      letterboxed(cv::Rect(pad_x, pad_y, resized_width, resized_height)));

    cv::Mat blob;
    cv::dnn::blobFromImage(
      letterboxed,
      blob,
      1.0 / 255.0,
      cv::Size(input_width_, input_height_),
      cv::Scalar(),
      true,
      false,
      CV_32F);

    check_cuda(
      cudaMemcpyAsync(
        device_input_,
        blob.ptr<float>(),
        input_elements_ * sizeof(float),
        cudaMemcpyHostToDevice,
        stream_),
      "Could not copy image to GPU");

    if (!context_->enqueueV2(bindings_.data(), stream_, nullptr)) {
      throw std::runtime_error("TensorRT inference failed.");
    }

    std::vector<float> output(output_elements_);

    check_cuda(
      cudaMemcpyAsync(
        output.data(),
        device_output_,
        output_elements_ * sizeof(float),
        cudaMemcpyDeviceToHost,
        stream_),
      "Could not copy detections from GPU");

    check_cuda(cudaStreamSynchronize(stream_), "CUDA stream failed");

    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    std::vector<int> class_ids;

    const int class_count = output_channels_ - 4;

    for (int i = 0; i < output_candidates_; ++i) {
      int class_id = 0;
      float confidence = 0.0F;

      for (int class_index = 0; class_index < class_count; ++class_index) {
        const float score = output[
          (4 + class_index) * output_candidates_ + i];

        if (score > confidence) {
          confidence = score;
          class_id = class_index;
        }
      }

      if (confidence < confidence_threshold_) {
        continue;
      }

      const float center_x = output[i];
      const float center_y = output[output_candidates_ + i];
      const float width = output[2 * output_candidates_ + i];
      const float height = output[3 * output_candidates_ + i];

      const float left = (center_x - width / 2.0F - pad_x) / scale;
      const float top = (center_y - height / 2.0F - pad_y) / scale;
      const float right = (center_x + width / 2.0F - pad_x) / scale;
      const float bottom = (center_y + height / 2.0F - pad_y) / scale;

      const int x1 = std::max(0, static_cast<int>(left));
      const int y1 = std::max(0, static_cast<int>(top));
      const int x2 = std::min(frame.cols, static_cast<int>(right));
      const int y2 = std::min(frame.rows, static_cast<int>(bottom));

      if (x2 <= x1 || y2 <= y1) {
        continue;
      }

      boxes.emplace_back(x1, y1, x2 - x1, y2 - y1);
      scores.push_back(confidence);
      class_ids.push_back(class_id);
    }

    std::vector<int> indices;

    cv::dnn::NMSBoxes(
      boxes,
      scores,
      static_cast<float>(confidence_threshold_),
      static_cast<float>(nms_threshold_),
      indices);

    cv::Mat annotated = frame.clone();

    const std::array<std::string, 5> class_names = {
      "Runway_36_Direction_Right",
      "Runway_36_Red",
      "Taxiway_B",
      "Taxiway_B_Direction_Left",
      "Terminal_A_Direction_Right"
    };

    for (const int index : indices) {
      const auto & box = boxes[index];

      cv::rectangle(
        annotated,
        box,
        cv::Scalar(0, 255, 0),
        2);

      std::string label;

      if (class_ids[index] >= 0 &&
          class_ids[index] < static_cast<int>(class_names.size())) {
        label = class_names[class_ids[index]];
      } else {
        label = "Class_" + std::to_string(class_ids[index]);
      }

      label += " " + std::to_string(scores[index]).substr(0, 4);

      cv::putText(
        annotated,
        label,
        cv::Point(box.x, std::max(20, box.y - 8)),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        cv::Scalar(0, 255, 0),
        2);
    }

    const auto now = std::chrono::steady_clock::now();
    const double elapsed =
      std::chrono::duration<double>(now - last_time_).count();

    const double fps = elapsed > 0.0 ? 1.0 / elapsed : 0.0;
    last_time_ = now;

    cv::putText(
      annotated,
      "FPS: " + std::to_string(fps).substr(0, 5),
      cv::Point(10, 30),
      cv::FONT_HERSHEY_SIMPLEX,
      0.9,
      cv::Scalar(0, 255, 0),
      2);

    cv::imshow("AutoTaxi TensorRT Detection", annotated);
    cv::waitKey(1);
  }

  std::string model_path_;
  std::string camera_topic_;

  double confidence_threshold_;
  double nms_threshold_;

  TrtLogger logger_;
  TrtPtr<nvinfer1::IRuntime> runtime_{nullptr};
  TrtPtr<nvinfer1::ICudaEngine> engine_{nullptr};
  TrtPtr<nvinfer1::IExecutionContext> context_{nullptr};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

  int input_index_{-1};
  int output_index_{-1};

  int input_width_{416};
  int input_height_{416};

  int output_channels_{0};
  int output_candidates_{0};

  size_t input_elements_{0};
  size_t output_elements_{0};

  float * device_input_{nullptr};
  float * device_output_{nullptr};

  cudaStream_t stream_{nullptr};
  std::vector<void *> bindings_;

  std::chrono::steady_clock::time_point last_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<TrtDetector>());
  } catch (const std::exception & error) {
    std::cerr << "Fatal error: " << error.what() << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}
