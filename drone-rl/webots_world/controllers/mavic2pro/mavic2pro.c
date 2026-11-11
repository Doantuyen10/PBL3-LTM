#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <stdio.h>
#include <string.h>

int main(int argc, char **argv) {
  // 1. Khởi tạo Robot
  wb_robot_init();

  // Lấy thời gian bước (timestep) của môi trường
  int time_step = (int)wb_robot_get_basic_time_step();

  // Lấy tên robot để phân biệt vai trò (IoT Node hay UAV Relay)
  const char *robot_name = wb_robot_get_name();
  printf("=== KHỞI TẠO CONTROLLER (C) CHO: %s ===\n", robot_name);

  // ==========================================
  // LUỒNG XỬ LÝ DÀNH CHO IOT NODE
  // ==========================================
  if (strstr(robot_name, "IoT") != NULL || strstr(robot_name, "iot") != NULL) {
    // Lấy thiết bị Emitter
    WbDeviceTag emitter = wb_robot_get_device("emitter");
    if (emitter) {
      wb_emitter_set_channel(emitter, 1); // Đặt Kênh 1 gửi sóng lên UAV
    }

    int packet_counter = 0;
    int step_count = 0;

    while (wb_robot_step(time_step) != -1) {
      step_count++;
      
      // Gửi dữ liệu định kỳ mỗi 1 giây (1000 ms)
      if (step_count % (1000 / time_step) == 0) {
        packet_counter++;
        char buffer[128];
        snprintf(buffer, sizeof(buffer), "Ping tu %s - Goi tin #%d", robot_name, packet_counter);

        // Gửi gói tin qua Emitter
        if (emitter) {
          wb_emitter_send(emitter, buffer, strlen(buffer) + 1);
          printf("[%s] -> DA GUI: %s\n", robot_name, buffer);
        }
      }
    }
  } 
  // ==========================================
  // LUỒNG XỬ LÝ DÀNH CHO UAV RELAY
  // ==========================================
  else {
    // Lấy thiết bị Receiver và Emitter trên UAV
    WbDeviceTag receiver = wb_robot_get_device("receiver");
    WbDeviceTag emitter = wb_robot_get_device("emitter");

    if (receiver) {
      wb_receiver_set_channel(receiver, 1); // Đặt Kênh 1 trùng với IoT Node
      wb_receiver_enable(receiver, time_step); // BẮT BUỘC: Bật Receiver
    }

    if (emitter) {
      wb_emitter_set_channel(emitter, 2);   // Đặt Kênh 2 gửi về Base Station
    }

    while (wb_robot_step(time_step) != -1) {
      // Lắng nghe hàng chờ gói tin gửi tới
      if (receiver) {
        while (wb_receiver_get_queue_length(receiver) > 0) {
          // Lấy dữ liệu gói tin
          const char *data = wb_receiver_get_data(receiver);
          
          printf("[%s RELAY] <- NHẬN THÀNH CÔNG: %s\n", robot_name, data);

          // (Tùy chọn) Chuyển tiếp gói tin qua Kênh 2 về Base Station
          if (emitter) {
            char relay_buffer[256];
            snprintf(relay_buffer, sizeof(relay_buffer), "RELAY_BY_%s -> %s", robot_name, data);
            wb_emitter_send(emitter, relay_buffer, strlen(relay_buffer) + 1);
          }

          // Chuyển sang gói tin tiếp theo trong queue
          wb_receiver_next_packet(receiver);
        }
      }
    }
  }

  // Dọn dẹp tài nguyên khi dừng mô phỏng
  wb_robot_cleanup();
  return 0;
}