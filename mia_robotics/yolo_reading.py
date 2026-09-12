from ultralytics import YOLO
from std_msgs.msg import String
from std_msgs.msg import Float32
model = YOLO("/mia_robotics/best.pt") 

def get_detection(frame, camera_node):
    h, w, _ = frame.shape
    img_cx = w / 2
    results = model.predict(frame, conf=0.5)
    names = model.names
   
    for result in results:
        boxes = result.boxes  # Bounding boxes
        for box in boxes:
            x1, y1, x2, y2 = box.xyxy.tolist()[0]
            conf = box.conf[0].item()           
            cls = box.cls[0].item()
            class_id = int(box.cls[0].item())
            class_name = names[class_id]
            obj_cx = (x1 + x2) / 2
            offset_x = float(obj_cx - img_cx)

            print(f"Detected Class {int(cls)}")

            msg = String()
            msg.data = class_name
            camera_node.target_pub.publish(msg)

            error_msg = Float32()
            error_msg.data = offset_x
            camera_node.target_x_error_pub.publish(error_msg)


            



    annotated_frame = results[0].plot()

    return annotated_frame
        
