Code gồm 2 phần chính
- Code firmware trash bin:
  . define chân :
  
          #define MOTOR_IN1   26
          #define MOTOR_IN2   27
          #define ENCODER_A   34
          #define ENCODER_B   35
          
          #define SERVO1_PIN  21
          #define SERVO2_PIN  18
          
          // Proximity sensor (ACTIVE LOW at home)
          #define SENSOR_PIN  14
  
  . Điều khiển các động cơ (1 DC encoder và 2 Servo MG996). Có 1 cảm biến tiệm cận làm mốc ở thùng 1
  
  . Encoder ~23550 xung / vòng (sau tất cả các hộp giảm tốc)
  
  . Code 1 webserver để điều khiển trashbin thủ công
  
  . Cách truy cập: Phát mạng wifi với SSID: ROBOT, Password: Robotic@123 -> ESP32 tự động kết nối vào wifi này và sẽ có địa chỉ IP là x.y.z.100 (mặc định)
  
  . Kết nối vào wifi ROBOT và truy cập địa chỉ x.y.z.100 để điều khiển

- Phần detect vật vào thùng rác:
  . Dùng 1 camera cắm vào máy ubuntu, máy PC này cần kết nối chung đến wifi ROBOT kia để có thể gửi API điều khiển trashbin qua wifi
  
  . Cắm camera vào máy tính và chạy file test_detect_2.py để chạy phân loại rác
