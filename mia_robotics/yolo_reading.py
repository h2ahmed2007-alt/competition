from ultralytics import YOLO

model = YOLO("/mia_robotics/best.pt") 

def get_detection(frame):

    results = model.predict(frame, conf=0.5)
   
   for result in results:
        boxes = result.boxes  # Bounding boxes
        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0]    
            conf = box.conf[0]              
            cls = box.cls[0]                
            print(f"Detected Class {int(cls)}")


    annotated_frame = results[0].plot()

    return annotated_frame
        
