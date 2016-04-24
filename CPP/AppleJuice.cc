#include "AppleTree.h"

using namespace std;
using namespace cv;

void AppleJuice::ReadImageLists(const opt options){
    string file_name = options.Image_list_dir + options.image_prefix;
    for (size_t i = 0; i < options.N_poses; i++) {
        for (size_t j = 0; j < options.N_shoots; j++) {
          /* code */
         Mat image = imread(file_name , CV_LOAD_IMAGE_GRAYSCALE);
        }

    }

}
