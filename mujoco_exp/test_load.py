import mujoco
import mujoco.viewer
import time

model = mujoco.MjModel.from_xml_path('./asserts/mujoco_model/piper_description.xml')
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.sync()
    
    start_time = time.time()
    while viewer.is_running():

        current_time = time.time() - start_time
        
        mujoco.mj_forward(model, data)
        
        mujoco.mj_step(model, data)
        
        viewer.sync()
        
        time.sleep(0.016)  # ~60 Hz

print("Viewer closed")