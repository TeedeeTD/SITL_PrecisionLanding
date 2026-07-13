# Sơ đồ Máy trạng thái (FSM) - Precision Landing

Dưới đây là lưu đồ luồng hoạt động (Flow Diagram) của State Machine bên trong `offboard_precland_controller.cpp`. Sơ đồ được vẽ bằng Mermaid, hiển thị trực quan các trạng thái và điều kiện chuyển đổi.

```mermaid
stateDiagram-v2
    [*] --> IDLE

    IDLE --> START : Call /precland/start
    
    START --> SEARCH : Khởi tạo thành công

    SEARCH --> HORIZONTAL_APPROACH : Nhìn thấy Aruco Marker (Target Found)
    SEARCH --> FALLBACK : Hết thời gian tìm kiếm (Search Timeout) / Quá Max Search

    HORIZONTAL_APPROACH --> DESCENT : Lọt vào tâm (Aligned) & Độ cao > final_alt
    HORIZONTAL_APPROACH --> TARGET_LOST : Mất dấu mục tiêu (Target Lost)
    HORIZONTAL_APPROACH --> FINAL_APPROACH : (Guarded Commit) Độ cao < abort_alt nhưng chưa lọt tâm

    TARGET_LOST --> HORIZONTAL_APPROACH : Tìm lại được mục tiêu
    TARGET_LOST --> FALLBACK : Hết thời gian chờ (Loss Timeout)

    DESCENT --> FINAL_APPROACH : Độ cao <= final_alt HOẶC Chạm đất (Landed)
    DESCENT --> SEARCH : Văng khỏi tâm (Drift) & Độ cao > abort_alt
    DESCENT --> HORIZONTAL_APPROACH : Văng khỏi tâm (Drift) & Độ cao <= abort_alt

    FINAL_APPROACH --> DONE : Disarm thành công (Motors Stopped)
    FINAL_APPROACH --> FALLBACK : Disarm thất bại sau 2s (Escalate to AUTO.LAND)

    FALLBACK --> [*] : Chuyển giao quyền cho PX4 (RTL / Auto Land)
    DONE --> IDLE : Hoàn tất hạ cánh
```

### Giải thích các Nhánh rẽ chính:
1. **Trục chính (Happy Path):** 
   `START` → `SEARCH` → `HORIZONTAL_APPROACH` (Căn giữa) → `DESCENT` (Vừa hạ vừa bám) → `FINAL_APPROACH` (Rơi mù) → `DONE`.
2. **Nhánh An toàn (Failsafe - Abort Alt):**
   - Đang hạ (`DESCENT`) mà bị gió tạt bay ra khỏi tâm:
     - Nếu còn cao (Z > `abort_alt`): Vọt lên `SEARCH` tìm lại.
     - Nếu đã sát đất (Z <= `abort_alt`): Chấp nhận rủi ro, trả về `HORIZONTAL_APPROACH` để ráng căn giữa rồi đáp, tuyệt đối không vọt lên nữa.
3. **Nhánh Kẹt (Fallback):**
   - Khi tìm mục tiêu quá số lần cho phép (`max_search`) hoặc thời gian chờ quá lâu.
   - Khi đã hạ cánh chạm đất nhưng lệnh Disarm thất bại (Delay 2s).
   - Chuyển quyền cho PX4 (gọi RTL hoặc AUTO.LAND tùy cấu hình).
