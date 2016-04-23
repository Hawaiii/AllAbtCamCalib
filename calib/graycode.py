import cv2
import numpy as np

def make_greycode_seq(screen_res, add_invert=False):
    """
    Returns a list of images (np array) to show and 
    a dictionary where keys are graycode sequence and values 
    are pixel locations.
    Args:
        screen_res: (screen_width, screen_height)
    """
    gc_seq = []
    gc_dict = {}

    width = screen_res[0]
    height = screen_res[1]

    # Vertical strips
    splits = [0, width]
    im = 255*np.ones((height, width), dtype=np.uint8)
    gc_seq.append(np.copy(im))

    while len(splits) < width+1:
        i = 0
        while i < len(splits)-1:
            half = (splits[i+1]+splits[i]+1)/2
            #[splits[i], half-1] stays the same, [half, splits[i+1]-1] gets inverted
            if half < splits[i+1] and half < width:
                im[:,half:splits[i+1]] = ~im[:, half:splits[i+1]]

            if splits[i] + 1 < splits[i+1]:
                splits.insert(i+1, half)
                i += 2
            else:
                i += 1
        gc_seq.append(np.copy(im))
        # print splits
        # import pdb; pdb.set_trace()
        
    # Horizontal strips
    splits = [0, height]
    im = 255*np.ones((height, width), dtype=np.uint8)
    gc_seq.append(np.copy(im))

    while len(splits) < height+1:
        i = 0
        while i < len(splits)-1:
            half = (splits[i+1]+splits[i]+1)/2
            #[splits[i], half-1] stays the same, [half, splits[i+1]-1] gets inverted
            if half < splits[i+1] and half < width:
                im[half:splits[i+1],:] = ~im[half:splits[i+1],:]

            if splits[i] + 1 < splits[i+1]:
                splits.insert(i+1, half)
                i += 2
            else:
                i += 1
        gc_seq.append(np.copy(im))
        #print splits

    # Build Dictionary
    n = len(gc_seq)
    for x in range(width):
        for y in range(height):
            code = ""
            for i in range(n):
                code += str( gc_seq[i][y,x]/255 )
            # if code in gc_dict:
            #     gc_dict[code].append((x,y))
            # else:
            #     gc_dict[code] = [(x,y)]
            gc_dict[code] = (x,y)
    print len(gc_dict)

    return gc_seq, gc_dict

def show_im_seq(imgs, time_interval):
    """
    Shows images in a list on screen in order, each showing time_interval seconds.
    """
    # Flash between two boards indefintely
    cv2.namedWindow('calibration', 0)
    # OpenCV has a fullscreen bug
    #cv2.setWindowProperty('calibration', cv2.WND_PROP_FULLSCREEN, cv2.cv.CV_WINDOW_FULLSCREEN)
    for i in xrange(len(imgs)):
        cv2.imshow('calibration',imgs[i])
        cv2.waitKey(1000*time_interval)
        # cv2.waitKey(0)

# gc_seq, gc_dict = make_greycode_seq((3,2))
gc_seq, gc_dict = make_greycode_seq((600,400))
show_im_seq(gc_seq, 1)


    
