# Kế hoạch chi tiết: Tối ưu hóa GPU Decoding & GPU Preprocessing qua GStreamer (Không cần OpenCV CUDA)

Tài liệu này mô tả chi tiết phương án kết hợp giải mã luồng video bằng GPU (NVDEC) và thực hiện các bước tiền xử lý (Lật ảnh 180 độ + Chuyển màu xám) trực tiếp trên nhân phần cứng tăng tốc GPU/VIC của Jetson thông qua GStreamer pipeline.

Giải pháp này được thiết kế để hoạt động ngay trên các phiên bản OpenCV tiêu chuẩn (chưa được compile hỗ trợ CUDA) của Jetson, đồng thời giải quyết triệt để vấn đề bất đồng bộ gây đơ ảnh và sụt giảm FPS (1.4 FPS).

---

## 1. Nguyên nhân và Giải pháp Kỹ thuật

Qua kiểm tra thực tế trên Jetson Orin Nano:
1.  **GStreamer NVDEC (`nvv4l2decoder`) hoạt động tốt:** Cho phép giải mã phần cứng bằng GPU.
2.  **OpenCV CUDA không khả dụng (`cv2.cuda` trả về 0):** Do OpenCV mặc định chưa được compile hỗ trợ CUDA.
3.  **Lỗi giật/đơ ảnh (1.4 FPS):** Do lệnh đọc camera đồng bộ `cap_.read()` khóa luồng chính của ROS 2 Executor khi dùng `WallTimer`.

### Giải pháp tối ưu:
*   **GStreamer GPU/VIC Pipeline:** Chúng ta tích hợp khâu **Lật ảnh 180 độ** (`nvvidconv flip-method=2`) và **Chuyển màu xám** (`nvvidconv ! video/x-raw, format=GRAY8`) trực tiếp vào chuỗi lệnh GStreamer chạy trên GPU/VIC của Jetson. Khi OpenCV nhận ảnh từ camera, ảnh đó đã được xử lý xong và ở dạng ảnh xám (Grayscale).
*   **Threaded Capture (Luồng đọc độc lập):** Giải phóng việc đọc camera khỏi ROS 2 Executor bằng cách chạy vòng lặp `cap_.read()` trên một luồng C++ riêng biệt (`std::thread`). Luồng này sẽ tự động đọc frame ngay khi camera giải mã xong và gửi (publish) trực tiếp sang ROS 2.

---

## 2. Kiến trúc luồng dữ liệu tối giản (GStreamer GPU/VIC, Threaded)

```
[Camera SIYI: Luồng RTSP H.264] 
      │ (Truyền qua Ethernet)
      ▼
[Jetson: GStreamer nvv4l2decoder (GPU)] -> Giải mã video
      │
      ▼
[Jetson: nvvidconv flip-method=2 (GPU/VIC)] -> Lật ảnh 180 độ
      │
      ▼
[Jetson: nvvidconv format=GRAY8 (GPU/VIC)] -> Chuyển sang ảnh xám 1 channel
      │
      ▼ (Chuyển dữ liệu ảnh xám thô 1280x720 về CPU)
[Capture Thread (Luồng riêng): Đọc liên tục & Publish]
      │
      ▼ (Publish qua ROS 2 mono8)
[Node: ArucoFractalTracker]
      │
      ▼ (detect trực tiếp trên ảnh gốc xám 1280x720)
[Tìm các góc Marker]
      │
      ▼
[poseEstimation với Camera Matrix gốc]
      │
      ▼
[Tính toán Pose 3D chính xác]
```

---

## 3. Chi tiết triển khai mã nguồn

### Bước 3.1: Tại Node `RtspPublisher`
File chỉnh sửa: `rtsp_publisher.cpp`

1.  **Khai báo luồng riêng và biến điều khiển:**
    Trong file `rtsp_publisher.hpp`:
    ```cpp
    std::thread capture_thread_;
    std::atomic<bool> thread_running_{false};
    void capture_loop();
    ```

2.  **Khởi chạy luồng trong Constructor:**
    Thay thế hoàn toàn bộ định thời `timer_` bằng việc kích hoạt luồng riêng:
    ```cpp
    thread_running_ = true;
    capture_thread_ = std::thread(&RtspPublisher::capture_loop, this);
    ```

3.  **Cập nhật GStreamer Pipeline trong `open_capture()`:**
    Thay đổi lệnh `cap_.open` để sử dụng GPU/VIC của Jetson thực hiện giải mã, lật ảnh và chuyển sang định dạng ảnh xám (`GRAY8`):
    ```cpp
    std::string pipeline = 
        "rtspsrc location=" + rtsp_url_ + " latency=0 ! "
        "rtph264depay ! h264parse ! "
        "nvv4l2decoder ! "  // Giải mã bằng GPU NVDEC
        "nvvidconv flip-method=" + (flip_180_ ? "2" : "0") + " ! " // Lật 180 độ bằng GPU/VIC
        "video/x-raw, format=GRAY8 ! " // Đưa ra định dạng ảnh xám 1 channel
        "videoconvert ! appsink drop=true sync=false";

    cap_.open(pipeline, cv::CAP_GSTREAMER);
    ```

4.  **Vòng lặp đọc Camera trong luồng riêng (Capture Loop):**
    ```cpp
    void RtspPublisher::capture_loop() {
      while (rclcpp::ok() && thread_running_) {
        if (!cap_.isOpened()) {
          open_capture();
          std::this_thread::sleep_for(std::chrono::milliseconds(500));
          continue;
        }

        cv::Mat gray_frame;
        // GStreamer cung cấp thẳng ảnh xám đã lật cho CPU
        bool ret = cap_.read(gray_frame); 
        if (!ret || gray_frame.empty()) {
          continue;
        }

        // Publish ngay lập tức qua ROS 2 (mono8)
        auto stamp = this->get_clock()->now();
        std_msgs::msg::Header header;
        header.stamp = stamp;
        header.frame_id = frame_id_;
        
        // Vì ảnh đã là ảnh xám 1 channel (GRAY8), OpenCV Mat sẽ có kiểu CV_8UC1.
        // Ta publish với định dạng "mono8"
        auto img_msg = cv_bridge::CvImage(header, "mono8", gray_frame).toImageMsg();
        image_pub_->publish(*img_msg);

        // Publish CameraInfo
        camera_info_msg_.header.stamp = stamp;
        info_pub_->publish(camera_info_msg_);
      }
    }
    ```

5.  **Giải phóng luồng trong Destructor:**
    ```cpp
    RtspPublisher::~RtspPublisher() {
      thread_running_ = false;
      if (capture_thread_.joinable()) {
        capture_thread_.join();
      }
      if (cap_.isOpened()) {
        cap_.release();
      }
    }
    ```

---

### Bước 3.2: Tại Node `ArucoFractalTracker`
File chỉnh sửa: `aruco_fractal_tracker_node.cpp`

1.  **Nhận ảnh xám `mono8` trực tiếp:**
    Nhận ảnh xám trực tiếp từ ROS 2:
    ```cpp
    // Nhận trực tiếp ảnh mono8 từ ROS 2 dưới dạng ảnh xám
    cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
    
    // Bỏ dòng cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY) trên CPU.
    // Chạy detect thẳng trên cv_ptr->image
    if (detector_.detect(cv_ptr->image)) {
        // logic tính toán pose và vẽ hình debug...
    }
    ```

---

## 4. Kế hoạch kiểm thử & Đánh giá hiệu năng

1.  **Kiểm tra tính tương thích của Pipeline GStreamer:**
    Chạy thử trước bằng công cụ `gst-launch-1.0` trên terminal của Jetson để xác minh tính ổn định của chuỗi lệnh giải mã + lật + đổi màu:
    ```bash
    gst-launch-1.0 rtspsrc location=rtsp://192.168.168.14:8554/main.264 latency=0 ! rtph264depay ! h264parse ! nvv4l2decoder ! nvvidconv flip-method=2 ! 'video/x-raw, format=GRAY8' ! videoconvert ! fakesink
    ```
2.  **Đo đạc FPS và hiện tượng đơ ảnh:**
    Chạy node mới và sử dụng lệnh:
    ```bash
    ros2 topic hz /siyi/image_raw
    ```
    Xác nhận FPS đạt ổn định ở 25-30 FPS và hình ảnh không còn hiện tượng bị đơ chu kỳ.
3.  **Đo đạc tải CPU:**
    Dùng `tegrastats` xác nhận tải CPU giảm mạnh do các bước giải mã, lật ảnh và chuyển màu đều đã chuyển sang GPU/VIC phần cứng.
