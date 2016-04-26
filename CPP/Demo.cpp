
#include "AppleTree.h"


using namespace cv;
void parse_argv(int argc, char** argv) {
  if(argc > 2) {
    printf("Incorrect usage: Demo image_dir\n");
    exit(-1);
  }
}


int main(int argc, char** argv) {

  // parse_argv(argc, argv);
  // Initialize Google's logging library.

  google::InitGoogleLogging(argv[0]);
  opt_ option_(29,2,800,600,0.5,14,11,12,
  "../../calib/h600.csv",
  "../../calib/v800.csv",
  "../data/",
  "./",
  "image",
  "pose",
  ".png"
);
  std::cout<<"starts from here...\n";
  AppleJuice* juice = new AppleJuice();
  juice->options = option_;

  try{
  juice->ReadLookup_table(option_);
  juice->ReadImageLists(option_);
  juice->BinarizeAllImages();
  juice->ExtractControlPts();
} catch (std::exception &e){
  std::cout << e.what() << std::endl;
}
  // juice->ReadLookup_table(option);

   return 0;
  }
