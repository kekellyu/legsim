import os
import numpy as np
import skvideo.io
from base64 import b64encode
from IPython.display import HTML
import mujoco
import myosuite
from myosuite.utils import gym

os.environ["MUJOCO_GL"] = "glfw"

def show_video(video_path, video_width=400):
    video_file = open(video_path, "r+b").read()
    video_url = f"data:video/mp4;base64,{b64encode(video_file).decode()}"
    return HTML(f"""<video autoplay width={video_width} controls><source src="{video_url}"></video>""")

env = gym.make('myoLegWalk-v0')

env.reset()
frames = []
for _ in range(200):
    frames.append(env.sim.renderer.render_offscreen(
                        width=400,
                        height=400,
                        camera_id=0))
    env.step(env.action_space.sample())  # take a random action
env.close()

os.makedirs('videos', exist_ok=True)
skvideo.io.vwrite('videos/MyoLeg.mp4', np.asarray(frames), outputdict={"-pix_fmt": "yuv420p"})

video_path = 'videos/MyoLeg.mp4'
video_file = open(video_path, "r+b").read()
video_url = f"data:video/mp4;base64,{b64encode(video_file).decode()}"
html_code = f"""<video autoplay width=400 controls><source src="{video_url}"></video>"""
with open("video.html", "w") as f:
    f.write(html_code)

print("Video saved as 'videos/MyoLeg.mp4' and HTML file 'video.html'")
