import mujoco
import mujoco.viewer
import time

model = mujoco.MjModel.from_xml_path('/home/hantian/Desktop/piper_controller/mujoco_exp/asserts/piper.xml')
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.sync()
    
    start_time = time.time()
    while viewer.is_running():

        current_time = time.time() - start_time
        
        mujoco.mj_forward(model, data)
        
        mujoco.mj_step(model, data)
        
        viewer.sync()
        
        time.sleep(1/60)  # ~60 Hz

print("Viewer closed")