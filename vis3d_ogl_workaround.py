# so apparently opengl isn't fucking clearing anything, and gives the same image no matter what after the first
# glReadPixels call, unless you close the entire fucking program and reopen it, so that's what we're doing. It's
# terrible, but so is base libraries being this fucking broken. And no, glClear does fucking nothing.

