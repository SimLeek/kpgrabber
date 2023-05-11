import open3d as o3d
import time
from displayarray import breakpoint_display, display
import numpy as np
import cv2
import OpenGL.GL as gl
import open3d.core

def visualize(mesh, view):
    vis = o3d.visualization.VisualizerWithEditing()
    #vis.create_window(window_name=f"view{view}", visible=False)
    vis.create_window(window_name=f"view{view}")

    t0 = t1 = time.time()
    orb = cv2.ORB_create(edgeThreshold=15, patchSize=31, nlevels=8, fastThreshold=20, scaleFactor=1.2, WTA_K=2,
                         scoreType=cv2.ORB_FAST_SCORE, firstLevel=0, nfeatures=1000)

    times = 1

    def rotate(vis):
        ctr = vis.get_view_control()
        nonlocal orb, times
        if times==0:
            return
        times-=1
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

        cam_pos = None
        fovy = 0.0
        current_pose = None

        def set_pose(pose):
            nonlocal cam_pos, fovy, current_pose
            current_pose = pose
            cam = ctr.convert_to_pinhole_camera_parameters()
            cam.extrinsic = pose  # where T is your matrix
            ctr.convert_from_pinhole_camera_parameters(cam)
            #print(f"fov: {ctr.get_field_of_view()}")
            # fov is 60
            # This is FOVY. Please document that shit Open3D. I shouldn't need to pull up a window to figure that out.
            fovy = ctr.get_field_of_view()

            rot = pose[:3, :3]
            pos = pose[:3, 3]
            cam_pos = -np.matmul(rot, pos).squeeze()

        if view == 0:
            set_pose(front_pose)
        elif view == 1:
            set_pose(back_pose)
        elif view == 2:
            set_pose(left_pose)
        elif view == 3:
            set_pose(right_pose)
        elif view == 4:
            set_pose(top_pose)
        elif view == 5:
            set_pose(bottom_pose)
        else:
            raise KeyError(f"View {view} not available.")

        # cv orb image:
        vis.update_renderer()
        #time.sleep(.5)
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

        rays = []
        for k in kp:
            #cam_pos
            #fovy
            width = img_gray.shape[1]
            height = img_gray.shape[0]
            aspect_ratio = width/height
            fovx = fovy*aspect_ratio
            theta = ((k.pt[1]-width/2)/(width/2))*(fovx/2) *2*np.pi/360.0 + np.pi/4 #x
            phi = ((k.pt[0]-height/2)/(height/2))*(fovy/2) *2*np.pi/360.0 + np.pi/2#y
            r = 1
            x,y,z = cam_pos
            z2 = r*np.sin(phi)*np.cos(theta)
            x2 = r*np.sin(phi)*np.sin(theta)  # swapping xyz so we look forward
            y2 = r*np.cos(phi)
            rot = current_pose[:3, :3]
            #todo: add next rot so spherical coords are pointing at object from x or y axis and not z
            pos2 = np.asarray([[x2],[y2],[z2]])
            true_dir = np.matmul(rot, pos2).squeeze()
            '''roll90 = np.asarray([
                [1,0,0],
                [0,0,1],
                [0,-1,0]
            ])
            true_dir = np.matmul(roll90, true_dir).squeeze()'''
            x2,y2,z2 = true_dir
            rays.append([x,y,z,x2,y2,z2])

        rays = o3d.core.Tensor(rays,
                               dtype=o3d.core.Dtype.Float32)

        scene = o3d.t.geometry.RaycastingScene()  # this requires 0.14.0 or prerelease open3d-0.13.0+dd076ba
        tmesh = o3d.t.geometry.TriangleMesh.from_legacy(mesh)
        cube_id = scene.add_triangles(tmesh)
        ans = scene.cast_rays(rays)

        pcl = []

        #"""
        for a, r in zip(ans['t_hit'], rays):
            if a.item()!=float('inf'):
                xyz = r[0:3].numpy()+r[3:6].numpy()*(float(a.item()))
                x,y,z = xyz
                pcl.append([x,y,z])

        """
        for r in rays:
            xyz = r[0:3].numpy() + r[3:6].numpy() * (10.0)
            x, y, z = xyz
            pcl.append([x, y, z])
        """
        display(img2_kp)

        pclg = o3d.geometry.PointCloud()
        if len(pcl)==0:
            raise RuntimeError("No points hit")
        pcln = np.asarray(pcl)
        pclg.points = o3d.utility.Vector3dVector(pcln)
        vis2 = o3d.visualization.VisualizerWithEditing()
        # vis.create_window(window_name=f"view2{view}", visible=False)
        vis2.create_window(window_name=f"view2{view}")
        vis2.add_geometry(pclg, reset_bounding_box=True)
        vis2.run()
        vis2.destroy_window()
        #o3d.visualization.draw_geometries([pclg])


        #time.sleep(.5)

    vis.register_animation_callback(rotate)

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
    for i in range(0,6):
        visualize(mesh, view=i)
