#include "AppleTree.h"

using namespace std;
using namespace cv;

void AppleJuice::ReadImageLists(const opt options){
    string base_name = options.Image_list_dir + options.pose_prefix;
    for (size_t i = 0; i < options.N_poses; i++) {
        vector<Mat> tmp_vector;
        string file_name = base_name + (to_string(i) + "/");

        for (size_t j = 0; j < options.N_shoots; j++) {
          /* code */
        string reading_name = file_name + options.image_prefix + to_string(j) + options.image_type;
        std::cout << "/* message */ reading: " << reading_name << std::endl;
         Mat image = imread(reading_name , CV_LOAD_IMAGE_GRAYSCALE);
         if(! image.data ){
              string ErrMessage = "Cannot read image:" + reading_name ;
              throw std::runtime_error( ErrMessage );
            }
            tmp_vector.push_back(image);
        }
        this->ImageLists.push_back(tmp_vector);
    }
    assert(ImageLists.size() == options.N_poses && ImageLists[0].size() == options.N_shoots);
    std::cout << "/* ImageLists loading is done... */" << std::endl;
}
