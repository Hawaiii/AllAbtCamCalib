#include "AppleTree.h"

using namespace std;
using namespace cv;

void AppleJuice::ReadImageLists(const opt options){
    string file_name = options.Image_list_dir + options.pose_prefix;
    for (size_t i = 0; i < options.N_poses; i++) {
        vector<Mat> tmp_vector;
        for (size_t j = 0; j < options.N_shoots; j++) {
          /* code */

           Mat tmp_image = imread(file_name + to_string(i) + options.image_prefix + to_string(j)
          , CV_LOAD_IMAGE_GRAYSCALE);
          if(! tmp_image.data)
                throw 
        }

    }

}
