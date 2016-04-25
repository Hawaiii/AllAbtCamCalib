#include "AppleTree.h"

using namespace std;
using namespace cv;


arma::mat CV2Arma_double(Mat opencv_mat){
  assert(opencv_mat.type() == CV_64FC1);
  DLOG(INFO)<<"CV2Arma_double...input size: " << opencv_mat.rows <<" | " << opencv_mat.cols  <<"\n\n";
  arma::mat ret( reinterpret_cast<double*>(opencv_mat.data), opencv_mat.cols, opencv_mat.rows );
  ret = ret.t();
  assert(ret.n_rows == opencv_mat.rows && ret.n_cols == opencv_mat.cols );
  return ret;
}

Mat Arma2CV_double(arma::mat B){
  cv::Mat ret;
  cv::Mat opencv_mat( B.n_cols, B.n_rows, CV_64FC1, B.memptr() );
  transpose(opencv_mat, ret);
  assert(B.n_rows == ret.rows && B.n_cols == ret.cols);
  return ret;
}


void AppleJuice::ReadImageLists(const opt options, unsigned int display){
    string base_name = options.Image_list_dir + options.pose_prefix;

    for (size_t i = 0; i < options.N_poses; i++) {
        vector<Mat> tmp_vector;
        string file_name = base_name + (to_string(i) + "/");
        arma::cube tmp_blob(options.image_height, options.image_width, options.N_shoots );
        DLOG(INFO) << "Arma Blob size:" << tmp_blob.n_rows <<" | " << tmp_blob.n_cols <<" | " <<
tmp_blob.n_slices <<"\n";

        for (size_t j = 0; j < options.N_shoots; j++) {

             string reading_name = file_name + options.image_prefix + to_string(j) + options.image_type;
             DLOG(INFO) << "/* message */ reading: " << reading_name << std::endl;
             Mat image = imread(reading_name , CV_LOAD_IMAGE_GRAYSCALE);
             if(! image.data ){
              string ErrMessage = "Cannot read image:" + reading_name ;
              throw std::runtime_error( ErrMessage );
            }
            if(display){
              imshow(file_name, image);
              waitKey(1000);
            }
            // Using INTER_AREA Here @Eric
            DLOG(INFO) << "image size:" << image.size() <<" | channels: "<< image.channels() << " | type: "<< image.depth();

            cv::resize(image, image, Size(options.image_width,options.image_height),INTER_AREA);
            image.convertTo(image, CV_64FC1);
            tmp_blob.slice(j) = CV2Arma_double(image);
            tmp_vector.push_back(image);
        }
        this->ImageBlob.push_back(tmp_blob);
        // this->ImageLists.push_back(tmp_vector);
    }
    // assert(ImageLists.size() == options.N_poses && ImageLists[0].size() == options.N_shoots);
    assert(ImageBlob.size() == options.N_poses && ImageBlob[0].n_slices == options.N_shoots);
    std::cout << "/* ImageLists loading is done... */" << std::endl;
}

void AppleJuice::BinarizeAllImages(){
        for (size_t i = 0; i < ImageBlob.n_slices; i++) {
            // Find the  & min map of each blob
            arma::mat min_map = arma::min(ImageBlob[i],2);
            arma::mat max_map = arma::max(ImageBlob[i],2);
            
        }

}
