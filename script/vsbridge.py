from ultralytics import YOLO
import cv2
import serial  # 新增

# ===================== 串口配置 =====================
ser = serial.Serial(
    port='COM3',        # ⚠ 改成你的串口，比如 COM3 或 /dev/ttyUSB0
    baudrate=115200,    # 波特率
    timeout=1
)
# ===================================================

# ===================== 模型 & 摄像头 =====================
model = YOLO("best.pt")
cap = cv2.VideoCapture(2)

cv2.namedWindow("火箭检测 + 坐标显示", cv2.WINDOW_NORMAL)
cv2.resizeWindow("火箭检测 + 坐标显示", 1280, 720)
# =====================================================

while True:
    ret, frame = cap.read()
    if not ret:
        print("读取摄像头失败")
        break

    results = model(frame, conf=0.1)

    annotated_frame = results[0].plot()

    # ============= 输出坐标 + 串口发送 ====================
    for r in results:
        for box in r.boxes:
            x1, y1, x2, y2 = box.xyxy[0]

            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            cls_name = r.names[int(box.cls[0])]

            # 控制台输出
            print(f"✅ 检测到：{cls_name} | 中心坐标 [{cx},{cy}]")

            # 🔥 串口发送
            data = f"[{cx},{cy}]\n"
            ser.write(data.encode('utf-8'))
    # ===================================================

    cv2.imshow("火箭检测 + 坐标显示", annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放资源
cap.release()
ser.close()   # 关闭串口
cv2.destroyAllWindows()