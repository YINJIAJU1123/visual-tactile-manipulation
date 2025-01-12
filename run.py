import subprocess
import time

scripts = [
    ["python", "npy_png.py"],
    ["python", "demo.py", "--checkpoint_path", "logs/log_kn/checkpoint.tar"],
    ["python", "ending_coordinate.py"]
]

for i, script in enumerate(scripts):
    print(f"Executing: {' '.join(script)}")

    if i == 1:  
        process = subprocess.Popen(script)
        time.sleep(10)  

        ending_process = subprocess.Popen(scripts[2])
        print("Forced execution of ending_coordinate.py")
        
        #process.wait()  
        
    else:
        result = subprocess.run(script, capture_output=True, text=True)

        print("Output:", result.stdout)
        print("Error:", result.stderr)

        if result.returncode != 0:
            print(f"Error occurred while executing: {' '.join(script)}")
            break  
