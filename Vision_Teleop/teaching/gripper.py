"""Attach the upstream Panda hand as a fixed-open tool, without new control DOFs."""
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


def attach_fixed_hand(scene, settings, root, rod_task=None, rod_collision_variant='original'):
    source = root/settings["xml"]
    tree = ET.parse(source)
    hand = tree.find("worldbody/body")
    tree.find("compiler").set("meshdir",str(source.parent/"assets"))
    # The standalone world pose is not a flange mounting transform.
    hand.set("quat","1 0 0 0")
    for tag in ("actuator","tendon","equality"):
        element = tree.find(tag)
        if element is not None: tree.getroot().remove(element)
    opening = float(settings["opening_m"])
    if not 0 < opening <= .08:
        raise ValueError("Panda tool opening must be in (0, .08] m")
    for name,sign,color in (("left_finger",1,"1 .65 .05 1"),("right_finger",-1,".15 .5 1 1")):
        finger = hand.find(f"body[@name='{name}']")
        for joint in list(finger.findall("joint")): finger.remove(joint)
        finger.set("pos",f"0 {sign*opening/2} 0.0584")
        for geom in finger.findall("geom"):
            if geom.get("class") == "visual":
                geom.attrib.pop("material",None)
                geom.set("rgba",color)
    ET.SubElement(hand,"site",name="tcp",pos="0 0 0.1034",size="0.004",rgba="0 1 0 1")
    if rod_task is not None:
        from .rod_scene import add_held_rod
        add_held_rod(hand, rod_task)
    parent = mujoco.MjSpec.from_file(str(scene))
    if rod_collision_variant != 'original':
        if rod_task is None or rod_collision_variant != 'wrist2_mesh_parts':
            raise ValueError('Unknown or non-task collision variant')
        from .rod_scene import compile_wrist2_candidate
        compile_wrist2_candidate(parent)
    child = mujoco.MjSpec.from_string(ET.tostring(tree.getroot(),encoding="unicode"))
    parent.attach(child,prefix="tool_",site=parent.site(settings["mount_site"]))
    if rod_task is not None:
        from .rod_scene import add_task_scene
        add_task_scene(parent, rod_task)
    return parent.compile()


# Palm columns are width, wrist-to-knuckles, normal. Tool columns are
# normal, jaw opening (toward yellow finger), approach (toward fingertips).
PALM_TO_TOOL = np.array([[0.,1.,0.],[0.,0.,1.],[1.,0.,0.]])


def guide_arrows(image, xyz, target_hand_rotation, calibration, side):
    """Project desired physical directions into each original camera image."""
    import cv2
    c = {k:np.asarray(calibration[k],float) for k in ("K1","D1","K2","D2","R","T")}
    points = np.array([xyz,xyz+.07*target_hand_rotation[:,1],xyz+.06*target_hand_rotation[:,0]])
    if side:
        points = points@c["R"].T+c["T"].reshape(3)
    if not np.isfinite(points).all() or np.any(points[:,2]<=0): return
    xy = cv2.projectPoints(points,np.zeros(3),np.zeros(3),c["K2" if side else "K1"],c["D2" if side else "D1"])[0][:,0]
    # The displayed image is resized, but projections use the calibrated size.
    xy *= np.array(image.shape[1::-1])/np.array(calibration["image_size"])
    if not np.isfinite(xy).all() or np.max(np.abs(xy))>10000: return
    xy = np.rint(xy).astype(int)
    for end,color in ((1,(0,230,0)),(2,(0,190,255))):
        cv2.arrowedLine(image,tuple(xy[0]),tuple(xy[end]),color,2,tipLength=.25)
