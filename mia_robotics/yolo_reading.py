from ultralytics import YOLO
from std_msgs.msg import String
model = YOLO("/mia_robotics/best.pt") 

def get_detection(frame, camera_node):

    results = model.predict(frame, conf=0.5)
    names = model.names
   
    for result in results:
        boxes = result.boxes  # Bounding boxes
        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0]    
            conf = box.conf[0]              
            cls = box.cls[0]    
            class_id = int(box.cls[0].item())
            class_name = names[class_id]
            print(f"Detected Class {int(cls)}")

            msg = String()
            msg.data = class_name
            camera_node.target_pub.publish(msg)
            



    annotated_frame = results.plot()

    return annotated_frame
        
