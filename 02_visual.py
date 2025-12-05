import genesis as gs
import cv2

gs.init(backend=gs.gpu)


scene = gs.Scene(
    show_viewer    = True,
    viewer_options = gs.options.ViewerOptions(
        res           = (1280, 960),
        camera_pos    = (3.5, 0.0, 2.5),
        camera_lookat = (0.0, 0.0, 0.5),
        camera_fov    = 40,
        max_FPS       = None,
    ),
    vis_options = gs.options.VisOptions(
        show_world_frame = False, # visualize the coordinate frame of `world` at its origin
        world_frame_size = 1.0, # length of the world frame in meter
        show_link_frame  = False, # do not visualize coordinate frames of entity links
        show_cameras     = False, # do not visualize mesh and frustum of the cameras added
        plane_reflection = True, # turn on plane reflection
        ambient_light    = (0.1, 0.1, 0.1), # ambient light setting
    ),
    renderer = gs.renderers.Rasterizer(), # using rasterizer for camera rendering
)



plane = scene.add_entity(gs.morphs.Plane())
franka_1 = scene.add_entity(
    gs.morphs.MJCF(
        file  = 'xml/franka_emika_panda/panda.xml',
        pos   = (0, 0, 0),
        euler = (0, 0, 90), # we follow scipy's extrinsic x-y-z rotation convention, in degrees,
        # quat  = (1.0, 0.0, 0.0, 0.0), # we use w-x-y-z convention for quaternions,
        scale = 1.0,
    ),
)

# ranka_1 = scene.add_entity(gs.morphs.URDF(file='urdf/panda_bullet/panda.urdf', pos = (1, 1, 0), scale = 1.0, fixed=True))

cam = scene.add_camera(
    res    = (640, 480),
    pos    = (3.5, 0.0, 2.5),
    lookat = (0, 0, 0.5),
    fov    = 30,
    GUI    = False
)

scene.build()

# cam_pose = scene.viewer.camera_pose

# print(f"cam_pose: {cam_pose}")
# scene.viewer.set_camera_pose(cam_pose)

cam.start_recording()

import numpy as np
for i in range(1000):
    
    # rgb, depth, segmentation, normal = cam.render(depth=True, segmentation=True, normal=True)
    # cv2.waitKey(1)
    scene.step()
    # cam.render()

    # change camera position
    cam.set_pose(
        pos    = (3.0 * np.sin(i / 60), 3.0 * np.cos(i / 60), 2.5),
        lookat = (0, 0, 0.5),
    )
    
    cam.render()

cam.stop_recording(save_to_filename='video.mp4', fps=60)