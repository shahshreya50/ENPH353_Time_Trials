#!/usr/bin/env python3
import cv2
import time
from pathlib import Path
from cv_bridge import CvBridge
import read_clues

def benchmark_all_runs(base_data_path):
    data_dir = Path(base_data_path)
    bridge = CvBridge()
    
    total_images = 0
    correct_predictions = 0
    run_stats = {}

    # 1. Iterate through all potential run directories
    # Use sorted() to keep the output organized
    folders = sorted(data_dir.glob("run_outputs_*"))
    
    for run_folder in folders:
        # 2. Explicit check for directories containing clues.csv
        clues_file = run_folder / 'clues.csv'
        if not run_folder.is_dir() or not clues_file.exists():
            # Skip folders that don't have the CSV
            continue
            
        print(f"\n--- Processing Valid Run: {run_folder.name} ---")
        run_correct = 0
        run_total = 0

        # 3. Iterate through images
        for file_path in sorted(run_folder.glob("image_car*.png")):
            parts = file_path.stem.split('_')
            
            # Ensure the file follows the naming convention: image_carX_GROUNDTRUTH
            if len(parts) >= 3:
                car_label = parts[1]
                ground_truth = parts[2]
                
                cv_img = cv2.imread(str(file_path))
                if cv_img is None:
                    continue

                run_total += 1
                
                # 4. Model Selection logic
                if "car7" in car_label:
                    prediction = read_clues.read_clueboard_8(cv_img)
                else:
                    prediction = read_clues.read_clueboard(cv_img)

                # 5. Evaluation (Stripping and Uppercasing for robustness)
                is_correct = (prediction.strip().upper() == ground_truth.strip().upper())
                
                if is_correct:
                    run_correct += 1
                
                status = "✅" if is_correct else f"❌ (GT: {ground_truth} | Pred: {prediction})"
                print(f"  {car_label}: {status}")

        if run_total > 0:
            run_stats[run_folder.name] = (run_correct, run_total)
            total_images += run_total
            correct_predictions += run_correct

    # 6. Final Global Report
    print("\n" + "="*50)
    print(f"{'RUN NAME':<35} | {'SCORE':<7} | {'ACC'}")
    print("-" * 50)
    
    for run, (c, t) in run_stats.items():
        acc = (c/t)*100
        print(f"{run:<35} | {c}/{t:<4} | {acc:.1f}%")
    
    if total_images > 0:
        overall_acc = (correct_predictions / total_images) * 100
        print("=" * 50)
        print(f"OVERALL ACCURACY: {overall_acc:.2f}% ({correct_predictions}/{total_images})")
    else:
        print("No valid directories with 'clues.csv' and images were found.")

if __name__ == '__main__':
    # Adjust this path based on your workspace structure
    DATA_PATH = Path(__file__).parent.parent.parent.parent / 'data'
    benchmark_all_runs(DATA_PATH)