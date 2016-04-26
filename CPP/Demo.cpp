
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
<<<<<<< HEAD
  google::SetLogDestination(0,"./log.txt");
  google::InitGoogleLogging(argv[0]);
  opt_ option_(28,2,800,600,0.5,14,11,11,
=======
  google::InitGoogleLogging(argv[0]);

  opt_ option_(29,2,800,600,0.5,14,11,12, 331.7030/2880*800, 207.3144/1800*600,
  "../../calib/h600.csv",
>>>>>>> origin/master
  "../../calib/v800.csv",
  "../../calib/h600.csv",
  "../data/",
  "./",
  "image",
  "pose",
  ".png"
);
  AppleJuice* juice = new AppleJuice();
  juice->SetOptions(option_);

  try{
  juice->ReadLookup_table(option_);
  juice->ReadImageLists(option_);
  juice->BinarizeAllImages();
  juice->ExtractControlPts();
} catch (std::exception &e){
  std::cout << e.what() << std::endl;
}

   return 0;
  }
