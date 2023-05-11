import tkinter
from tkinter import filedialog
from vis3d_workaround import load_and_visualize
root = tkinter.Tk()
root.filename =  filedialog.askopenfilename(
    initialdir = ".",title = "Select 3D file",
    filetypes = (
        ("Object Files","*.obj"),
        ("Polygon Files","*.ply"),
        ("StereoLithography","*.stl"),
        ("Object File Format","*.off"),
        ("GL Transmission Format","*.gltf"),
        ("GL Transmission Format", "*.glb"),
    )
)
load_and_visualize(root.filename)
