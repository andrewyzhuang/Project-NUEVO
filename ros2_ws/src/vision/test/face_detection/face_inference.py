from ultralytics import YOLO

# 1. Load your custom-trained model 
model = YOLO("best_Gender_classification.pt")

# 2. Run inference on your test image
results = model.predict(source="test_5.jpg", imgsz=224)

# 3. Process and visualize results
for result in results:
    top1_idx = result.probs.top1
    top1_conf = result.probs.top1conf.item()
    print(f"\nPredicted Class: {result.names[top1_idx]} with {top1_conf:.4f} confidence")
    
    # 👁️ OPTION A: This pops open a window right on your screen showing the image
    result.show()
    
    # 💾 OPTION B: This saves a new image file with the prediction text stamped on it
    result.save(filename="visual_result.png")