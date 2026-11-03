"""MUJOCO_GL=egl python scripts/render_preview.py"""
from pathlib import Path
from PIL import Image
import mujoco

root=Path(__file__).resolve().parents[1]
m=mujoco.MjModel.from_xml_path(str(root/'centaur.xml'));d=mujoco.MjData(m)
mujoco.mj_resetDataKeyframe(m,d,0);mujoco.mj_forward(m,d)
with mujoco.Renderer(m,height=850,width=1000) as renderer:
    images=[]
    for azimuth in [-40,145]:
        camera=mujoco.MjvCamera();camera.lookat[:]=[.04,0,.55]
        camera.distance=1.9;camera.azimuth=azimuth;camera.elevation=-12
        options=mujoco.MjvOption();options.geomgroup[3]=0;options.sitegroup[:]=0
        renderer.update_scene(d,camera,scene_option=options)
        renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=False
        images.append(Image.fromarray(renderer.render()))
out=Image.new('RGB',(2000,850));out.paste(images[0],(0,0));out.paste(images[1],(1000,0))
out.save(root/'Assembly_Preview.png')

