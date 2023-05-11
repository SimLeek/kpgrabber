import open3d as o3d
import time
from displayarray import breakpoint_display
import numpy as np
import cv2

def visualize(mesh):
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()

    t0 = t1 = time.time()
    capture_state = 0
    orb = cv2.ORB_create(edgeThreshold=15, patchSize=31, nlevels=8, fastThreshold=20, scaleFactor=1.2, WTA_K=2,
                         scoreType=cv2.ORB_FAST_SCORE, firstLevel=0, nfeatures=1000)
    """
    def rotate(vis):
        ctr = vis.get_view_control()
        nonlocal capture_state, orb
        '''nonlocal t0, t1
        t1 = time.time()
        dt = t1-t0
        t0 = t1
        print(dt)
        ctr.rotate(100*dt,100*dt)'''





        front_pose = np.array(
            [1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, 5,
             0, 0, 0, 1]).reshape(4, 4)

        back_pose = np.array(
            [-1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, -1, 5,
             0, 0, 0, 1]).reshape(4, 4)

        left_pose = np.array(
            [0, 0, 1, 0,
             0, 1, 0, 0,
             1, 0, 0, 5,
             0, 0, 0, 1]).reshape(4, 4)

        right_pose = np.array(
            [0, 0, -1, 0,
             0, 1, 0, 0,
             -1, 0, 0, 5,
             0, 0, 0, 1]).reshape(4, 4)

        top_pose = np.array(
            [1, 0, 0, 0,
             0, 0, 1, 0,
             0, 1, 0, 5,
             0, 0, 0, 1]).reshape(4, 4)

        bottom_pose = np.array(
            [1, 0, 0, 0,
             0, 0, -1, 0,
             0, -1, 0, 5,
             0, 0, 0, 1]).reshape(4, 4)

        def set_pose(pose):
            cam = ctr.convert_to_pinhole_camera_parameters()
            cam.extrinsic = pose  # where T is your matrix
            ctr.convert_from_pinhole_camera_parameters(cam)

        if capture_state == 0:
            set_pose(front_pose)
        elif capture_state == 1:
            set_pose(back_pose)
        elif capture_state == 2:
            set_pose(left_pose)
        elif capture_state == 3:
            set_pose(right_pose)
        elif capture_state == 4:
            set_pose(top_pose)
        elif capture_state == 5:
            set_pose(bottom_pose)
        else:
            capture_state = -1
        capture_state += 1

        # cv orb image:
        vis.update_renderer()
        time.sleep(.5)
        img = vis.capture_screen_float_buffer(do_render=True)

        img = np.asarray(img)

        img = np.flip(img, axis=2)  # convert from rgb to bgr for opencv
        breakpoint_display(img)

        # breakpoint_display(img)
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_gray = (img_gray * 255).astype(np.uint8, copy=False)
        kp = orb.detect(img_gray)
        img2_kp = cv2.drawKeypoints(img_gray, kp, None, color=(0, 255, 0),
                                    flags=cv2.DrawMatchesFlags_DEFAULT)
        breakpoint_display(img2_kp)

        #time.sleep(.5)
    """


    #vis.register_animation_callback(rotate)

    #mesh.compute_vertex_normals()
    vis.add_geometry(mesh)
    ro = vis.get_render_option()
    ro.mesh_shade_option = o3d.visualization.MeshShadeOption.Color
    vis.run()
    vis.destroy_window()

def load_and_visualize(filename):
    mesh = o3d.io.read_triangle_mesh(filename)
    if len(mesh.textures)>1:
        mesh.textures = [mesh.textures[1], mesh.textures[1]]
    #mesh = o3d.io.read_triangle_model("ring.obj")
    visualize(mesh)
