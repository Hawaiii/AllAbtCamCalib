import cv2
import numpy as np
import csv

def make_greycode_seq(screen_res, add_invert=False):
    """
    Returns a list of images (np array) to show and 
    a dictionary where keys are graycode sequence and values 
    are pixel locations.
    Args:
        screen_res: (screen_width, screen_height)
    """
    width = screen_res[0]
    height = screen_res[1]

    # Start signal: black, white, black
    print "Start signal: 0, 1, 0..."
    s_seq = []
    s_seq.append(np.zeros((height, width), dtype=np.uint8))
    s_seq.append(255*np.ones((height, width), dtype=np.uint8))
    s_seq.append(np.zeros((height, width), dtype=np.uint8))

    # Vertical strips
    print "Generating vertical strips..."
    splits = [0, width]
    im = 255*np.ones((height, width), dtype=np.uint8)
    v_seq = [np.copy(im)]

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
        v_seq.append(np.copy(im))
    v_dict = record_dict(v_seq, width, height, vertical=True)
    write_dict_csv(v_dict,'v'+str(width)+'.csv')
        
    # Horizontal strips
    print "Generating horizontal strips..."
    splits = [0, height]
    im = 255*np.ones((height, width), dtype=np.uint8)
    h_seq = [np.copy(im)]

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
        h_seq.append(np.copy(im))
    h_dict = record_dict(h_seq, width, height, vertical=False)
    write_dict_csv(h_dict,'h'+str(height)+'.csv')
        
    # End signal: white, black, white
    print "End signal: 1, 0, 1"
    e_seq = []
    e_seq.append(255*np.ones((height, width), dtype=np.uint8))
    e_seq.append(np.zeros((height, width), dtype=np.uint8))
    e_seq.append(255*np.ones((height, width), dtype=np.uint8))
   

    return s_seq + v_seq + h_seq + e_seq

def record_dict(gc_seq, width, height, vertical):
    print "Generating dictionary"
    gc_dict = {}
    n = len(gc_seq)
    if vertical:
        for x in range(width):
           code = ""
           for i in range(n):
               code += str( gc_seq[i][0,x]/255 )
           gc_dict[code] = x
    else:
        for y in range(height):
            code = ""
            for i in range(n):
                code += str( gc_seq[i][y,0]/255 )
            gc_dict[code] = y
    print 'Done. dictionary length ',len(gc_dict)
    return gc_dict

def write_dict_csv(gc_dict, filename):
    csvfile = open(filename, 'w')
    writer = csv.writer(csvfile, delimiter=',')
    for code in gc_dict.keys():
        writer.writerow([code, gc_dict[code]])
    csvfile.close()

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
    
gc_seq = make_greycode_seq((1440,900))
show_im_seq(gc_seq, 1) #Change to 0 for keyboard control


    
