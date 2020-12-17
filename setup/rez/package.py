name = "gagnon2016_dynamic_lapped_texture"

uuid = "ba65a0c2-b993-4212-a965-4a701d1be618"

description = "Texturing Fluid with rigid patches"

version = "1.0.0"


authors = [ "Jonathan Gagnon" ]

requires = ['cmake', 'opencv', 'houdini-17']

def commands():
    env.HOUDINI_DSO_PATH.append("@/dso_^:@/dso:{root}/dso/")
    env.REZDEAD_HOUDINI_DSO_PATH.append("@/dso_^:@/dso:{root}/dso/")
    #we should move this to {root}/otl
    #env.HOUDINI_OTLSCAN_PATH.append( "@/otls_^:@/otls:{root}/otl/;$HFS/houdini/otls")
    env.HOUDINI_PATH.append("{root}")
    env.REZDEAD_DEFORMABLE_PATCHES_FLUID_VERSION.append("{root}")

