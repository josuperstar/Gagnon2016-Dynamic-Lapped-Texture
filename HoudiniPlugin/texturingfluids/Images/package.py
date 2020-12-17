name = "rd_image"

uuid = "f00246e4-6da5-4dec-9c1d-8ca527d1cba9"

description = "Research and Development Image Library"

version = "1.1.0"

authors = [ "Jonathan Gagnon" ]

#eventually QT, OpenCV ?
#requires = [ "houtools-1+<2", "x264-1+<2" ]

def commands():

    #setenv("HOUTOOLS_BASE_LOCATION" , root )
    env.PATH.append("{root}")
