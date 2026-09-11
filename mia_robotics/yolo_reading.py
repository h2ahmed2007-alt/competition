from ultralytics import YOLO

model = YOLO("best.pt") 

def get_detection(frame):

    results = model(frame)
    annotated_frame = results[0].plot()

    return annotated_frame
        