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


void AppleJuice::ReadImageLists(const opt_ options, unsigned int display){
    DLOG(INFO) << "start AppleJuice::ReadImageLists ....\n";
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
        assert(ImageBlob.size() != 0);
        DLOG(INFO) << "start AppleJuice::BinarizeAllImages() ....\n";

        for (size_t i = 0; i < ImageBlob.size(); i++) {
            arma::ucube tmp_blob(ImageBlob[i].n_rows, ImageBlob[i].n_cols, ImageBlob[i].n_slices);
            tmp_blob.fill(2);

            // Find the  & min map of each blob
            DLOG(INFO) << "Blob size: "<<  ImageBlob[0].n_rows <<" | "<< ImageBlob[0].n_cols << " | " << ImageBlob[0].n_slices <<"\n";
            arma::mat min_map = arma::min(ImageBlob[i],2);
            arma::mat max_map = arma::max(ImageBlob[i],2);
            arma::mat high_threshold_map = max_map * (1 - this->options.suppression_ratio )   + min_map * this->options.suppression_ratio;
            arma::mat low_threshold_map = min_map * (1 - this->options.suppression_ratio )   + max_map * this->options.suppression_ratio;

            arma::umat mask = arma::zeros<arma::umat>(options.image_height, options.image_width);
            mask.elem( find( (max_map - min_map)  >= options.suppression_ratio*255 ) ).ones();
            this->Masks.push_back(mask);
            // mask.save("mask"+to_string(i)+".mat", arma::raw_ascii);


            for (size_t x = 0; x < ImageBlob[i].n_slices; x++) {
              // the ambigulty == 2 !!!!!!!!
              arma::mat tmp_mat = ImageBlob[i].slice(x) - high_threshold_map;
              // if it is greater than high_threshold, assign 1
              tmp_blob.slice(x).elem( find(tmp_mat  >= 0 ) ).ones();
              tmp_mat = ImageBlob[i].slice(x) - low_threshold_map;
              // if it is smaller than low_threshold, assign 0
              tmp_blob.slice(x).elem( find(tmp_mat  <= 0 ) ).zeros();

              // if(DEBUG) tmp_blob.slice(x).save("blob" + to_string(i) + " slice " + to_string(x) + ".mat", arma::raw_ascii);
            }
            this->BinaryBlob.push_back(tmp_blob);
            DLOG(INFO) << "Save Blob: " << i << " done!\n";
        }
      std::cout << "/* message */ AppleJuice::BinarizeAllImages Done..." << std::endl;
    return;
}

void AppleJuice::ExtractControlPts(){

    for (size_t i = 0; i < BinaryBlob.size(); i++) {
      /* loop every pose */
        arma::uvec cords = find(Masks[i] == 1);
        DLOG(INFO) << "Search Region Size: " << cords.size() << endl;
        unsigned int outlier_cnt = 0;
        for (size_t j = 0; j < cords.size(); j++) {
          /* code */
          unsigned int row_ = cords(j) % options.image_height;
          unsigned int col_ = cords(j) / options.image_height;
          arma::umat tmp_string = BinaryBlob[i].subcube(arma::span(row_,row_),arma::span(col_,col_),arma::span::all);
          stringstream ss;
          tmp_string.print(ss);
          string target = ss.str();
          string::iterator end_pos = std::remove(target.begin(), target.end(), ' ');
          target.erase(end_pos, target.end());
          // if(DEBUG)
            // DLOG(INFO) << "target string: " <<"[" << row_ <<" , " << col_ << "]\n"<< target <<"\n";

          if(target.substr(0,3) != "010" && target.substr(target.size()-3,3) != "101")
              DLOG(INFO) << "Outlier...."<< ++outlier_cnt << endl;
          if(DEBUG)
              DLOG(INFO) << endl << "X encoding: " << target.substr(3,options.x_encoding_len) << endl
                         << "Y encoding: " << target.substr(3+options.x_encoding_len,options.y_encoding_len) << endl;
          pair<Point3f,Point3f> pts = SearchPoints(target.substr(3,options.x_encoding_len), target.substr(3+options.x_encoding_len,options.y_encoding_len));
          cout<<pts.first <<" | "<< pts.second<<endl;


        }



      }
}
